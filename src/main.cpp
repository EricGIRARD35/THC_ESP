#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <limits.h>
#include <TFT_eSPI.h>
#include <Adafruit_ADS1X15.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>


// --- THC pour ESP32 ---
// Note : L'ESP32 fonctionne en 3.3V logic ! 
// Utilisez des convertisseurs de niveau (Level Shifters) pour les signaux 5V (Plasma OK, etc.) si nécessaire.


// Output Pins Moteurs
#define DIR_PIN 26        // Sortie direction axe Z
#define STEP_PIN 25       // Sortie step axe Z
#define STEPPER_STEP_PIN STEP_PIN
#define STEPPER_DIR_PIN DIR_PIN

#define ENABLE_PIN 14     // Entrée THC ENABLE
#define THC_OFF_PIN 16

// --- NOUVELLES ENTREES POUR LE PASSTHROUGH CNC ---
#define CNC_Z_STEP_IN       27 // Entrée STEP Z en provenance de carte CNC
#define CNC_Z_DIR_IN        32 // Entrée DIR Z en provenance de carte CNC 

// Interrupt Pins (Step X/Y venant de CNC Shield ?)
// Sur ESP32, toutes les broches peuvent être des interruptions.
// On utilise les broches 34-39 qui sont "Input Only" (parfait pour des capteurs)

// Affectation pour le bus I²C de l'ADS1115
#define ADS_I2C_SDA_PIN 17 
#define ADS_I2C_SCL_PIN 22

#define XPT2046_CS   15

XPT2046_Touchscreen ts(XPT2046_CS);

unsigned long lastAdcRead = 0;
volatile int32_t adc_raw_value = 0;

Adafruit_ADS1115 ads;
// Si le pin ADDR est connecté à GND -> 0x48
#define ADS1115_ADDR_GND 0x48 
// Canal de l'ADS1115 utilisé pour lire le voltage plasma (ex: AIN0)
#define ADS_PLASMA_CHANNEL 1


TaskHandle_t taskUIHandle = NULL;
TaskHandle_t taskADCHandle = NULL;
TaskHandle_t taskControlHandle = NULL;

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t dataMutex;  

// Variables à protéger
struct DisplayData {
    float fast_voltage;
    float slow_voltage;
    float setpoint;
    long position;
    bool thc_active;
    bool enable_active;
    bool anti_dive_active;
    bool arc_ok;
};

DisplayData display_data = {0}; 


// Pleine échelle de l'ADS1115 (FSR - Full Scale Range) en volts.
// Nous allons configurer l'ADC pour utiliser un gain de 1 (GAIN_1), ce qui donne ±4.096V.
// ASSUREZ-VOUS que la tension maximale après le diviseur est inférieure à 4.096V.
const float ADS_GAIN_V = 4.096; 

// Résolution de l'ADS1115 (2^15 pour les lectures à simple extrémité signées, d'où 32767)
const float ADS_RESOLUTION_MAX = 32767.0; 

// Rapport de votre diviseur de tension (Exemple : si 40:1, alors 1/40 = 0.025)
// C'est le coefficient K tel que V_plasma = V_mesurée / K
const float PLASMA_VOLTAGE_DIVIDER_RATIO = 0.01623; // Vérifiez votre diviseur réel !

// Variables pour la lecture tactile non-bloquante
unsigned long lastTouchTime = 0;
const unsigned long touchInterval = 50; // Lire le tactile toutes les 50 ms

// --- Prototypes des fonctions d'interface tactile ---
void handleTouchInput(uint16_t x, uint16_t y);
// void adjustCurrentSetting(int direction);
void flashButton(int x1, int y1, int x2, int y2, uint16_t color);
void  lv_scr_load();

// EEPROM addresses for parameters (Rien ne change ici)
#define EEPROM_SETPOINT_ADDR 0
#define EEPROM_CORRECTION_FACTOR_ADDR 4
#define EEPROM_STEPS_MM_Z_ADDR 8
#define EEPROM_KP_ADDR 16
#define EEPROM_KI_ADDR 20
#define EEPROM_KD_ADDR 24
#define EEPROM_INITIALIZED_FLAG 28

// Parametres par défaut
const float DEFAULT_SETPOINT = 110.0; //Attention valeur de DEFAULT stocké sur 4 octets mais utilisé en double pour le PID
const float DEFAULT_CORRECTION_FACTOR = 1.0;
const float DEFAULT_STEP_PER_MM = 400;
const float DEFAULT_KP = 2; // attention l'action proportionnel agit dans ce cas comme une action intégrale en agissant sur la vitesse du moteur Z et non sur sa position.
const float DEFAULT_KI = 5; 
const float DEFAULT_KD = 0; // Laisser l'action dérivé a 0

double voltage_correction_factor = 1.0;
#define DEFAULT_VOLTAGEDIVIDER 50.0
#define MM_PER_VOLT_Z 1.0 // A ajuster lors des essais

byte initializedFlag = 0xAA;

// Initialize objects
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TFT_eSPI tft = TFT_eSPI();
// Variable de filtrage
const int speed_filter_size = 1;
float speed_readings[speed_filter_size];
int speed_reading_index = 0;
float sum_speed_readings = 0.0;

// PID variables
double Setpoint = DEFAULT_SETPOINT;
double Input = 0;
double Output = 0;
double smoothedOutput = 0.0;
double Kp = DEFAULT_KP;
double Ki = DEFAULT_KI;
double Kd = DEFAULT_KD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
long z_target = 0;
static long last_z_target = 0;
long z_reference = 0;
const float PID_TO_STEPS = 10.0; //mm/V a ajuster
bool pid_running = false;
volatile bool input_ready = false;
float STEPS_PER_MM_Z = DEFAULT_STEP_PER_MM;

// Display and logging variables
unsigned long last_display_time = 0;
const unsigned long display_interval = 1000;
const float arc_threshold = 10.0;
const unsigned long stabilization_delay = 500;
unsigned long plasma_active_time = 0;
const unsigned long LOG_INTERVAL = 5000;
unsigned long lastLogTime = 0;
bool thc_active = false;
bool thc_off = true;
uint16_t couleurON = TFT_GREEN;
uint16_t couleurOFF = TFT_RED;

// Screen navigation

int currentScreen = 0;
int prevScreen = 0;
const int NB_SCREENS = 6;
int prevCLK; // Previous state of CLK
int prevDT;  // Previous state of DT

// Global variables for timing measurement
unsigned long loopStartTime;
unsigned long loopEndTime;
unsigned long lastLoopLogTime = 0;
const unsigned long LOOP_LOG_INTERVAL = 10000;  // 10 seconds
unsigned long loopExecutionTimeSum = 0;
unsigned int loopCount = 0;
bool just_anti_dive_activated = false;

// --- Global variables for THC status ---
bool arc_voltage_ok = false; // Statut global de la détection d'arc
bool use_accelstepper_run = true;

// Global variables for anti-dive method
float fast_voltage = 0.0;              // Fast filtered voltage (corrected)
float slow_voltage = 0.0;              // Slow filtered voltage (corrected)
float uncorrected_fast = 0.0;          // Uncorrected fast voltage
float uncorrected_slow = 0.0;          // Uncorrected slow voltage
bool anti_dive_active = false;         // Anti-dive state
unsigned long anti_dive_start_time = 0;// Anti-dive start time

// New for improved anti-dive: position history buffer
const int POSITION_HISTORY_INTERVAL = 100; // ms between records
const int POSITION_HISTORY_SIZE = 20; // Enough for ~2 seconds
struct PosHistory {
  unsigned long time;
  long position;
};
PosHistory position_history[POSITION_HISTORY_SIZE];
int position_history_index = 0;
unsigned long last_position_record_time = 0;

// EEPROM write delay
static unsigned long last_eeprom_write = 0;
const unsigned long EEPROM_WRITE_INTERVAL = 1000;

// Temporary variable for voltage correction factor adjustment
float temp_voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;

// === OVERSAMPLING NON-BLOQUANT pour PID ultra-stable ===
#define OVERSAMPLE_TARGET 10  // 10 samples ~10ms @1kHz
static float oversample_sum = 0.0;
static uint8_t oversample_count = 0;
static float last_pid_input = 0.0;  // Dernière moyenne pour low-pass
const float INPUT_ALPHA = 0.7;      // Low-pass fort sur moyenne

// Function declarations
void initializeEEPROM();
void calculateSpeed();
void readAndFilterVoltage();
void managePlasmaAndTHC();
void countStepX();
void countStepY();
// void updateDisplay();

// ========================================
// SIMULATEUR DE SINUSOÏDE POUR TEST PID
// ========================================

// Variables globales pour simulation
bool simulation_mode = false;
unsigned long simulation_start_time = 0;
float simulation_amplitude = 3.0;      // ±10V autour du setpoint
float simulation_frequency = 0.1;       // 0.5 Hz = 2 secondes par cycle
float simulation_offset = 2.0;          // Offset DC pour tester tracking

// ========================================
// FONCTION 1 : Génération Sinusoïde Simple
// ========================================

// ========================================
// PARTIE 1 : CONFIGURATION LVGL
// ========================================

#include <lvgl.h>
#include <TFT_eSPI.h>

// Configuration écran
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define LVGL_BUFFER_SIZE (SCREEN_WIDTH * 40)

// Objets LVGL globaux
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LVGL_BUFFER_SIZE];
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

// Objets UI (équivalents de vos écrans)
lv_obj_t *screen_monitoring;
lv_obj_t *screen_setpoint;
lv_obj_t *screen_correction;
lv_obj_t *screen_kp;
lv_obj_t *screen_ki;
lv_obj_t *screen_steps;

// Labels dynamiques (pour mise à jour temps réel)
lv_obj_t *label_voltage_fast;
lv_obj_t *label_voltage_slow;
lv_obj_t *label_setpoint;
lv_obj_t *label_position;
lv_obj_t *label_thc_state;
lv_obj_t *label_enable_state;
lv_obj_t *label_antidive_state;
lv_obj_t *label_arc_state;

// Objets pour les écrans d'ajustement
lv_obj_t *label_setpoint_val;
lv_obj_t *label_correction_val;
lv_obj_t *label_kp_val;
lv_obj_t *label_ki_val;
lv_obj_t *label_steps_val;


lv_obj_t *screen_settings;
lv_obj_t *label_param_name;
lv_obj_t *label_param_value;
int selected_param = 0;  // 0=Setpoint, 1=Correction, 2=Kp, 3=Ki, 4=Steps

const char* param_names[] = {
    "Setpoint (V)",
    "Correction",
    "Kp",
    "Ki",
    "Steps/mm"
};

void btn_change_param_event(lv_event_t *e);
void btn_adjust_multi_event(lv_event_t *e);
void update_param_value_display();
void create_screen_settings();

// ========================================
// PARTIE 2 : CALLBACKS LVGL (Driver Display + Tactile)
// ========================================

// Fonction appelée par LVGL pour dessiner
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Protection SPI avec timeout plus long pour display
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        tft.startWrite();
        tft.setAddrWindow(area->x1, area->y1, w, h);
        tft.pushColors((uint16_t *)&color_p->full, w * h, true);
        tft.endWrite();
        xSemaphoreGive(spiMutex);
    } else {
        Serial.println("⚠️ SPI timeout dans flush!");
    }

    lv_disp_flush_ready(disp);
}

// Fonction pour lire le tactile
void lv_touchpad_read(lv_indev_drv_t * indev, lv_indev_data_t * data) {
    // Protection SPI
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        
        if (ts.touched()) {
            TS_Point p = ts.getPoint();

            data->state = LV_INDEV_STATE_PR;

            // ✅ MAPPING CORRIGÉ (utiliser vos valeurs de calibration)
            // Remplacez 360, 3900, 340, 3800 par vos valeurs réelles
            data->point.x = map(p.x, 3900, 360, 0, SCREEN_WIDTH);
            data->point.y = map(p.y, 3800, 340, 0, SCREEN_HEIGHT);

            // Clamp sécurité
            data->point.x = constrain(data->point.x, 0, SCREEN_WIDTH - 1);
            data->point.y = constrain(data->point.y, 0, SCREEN_HEIGHT - 1);

            // ✅ DEBUG IMPORTANT : Afficher les touches
            static unsigned long lastDebug = 0;
            if (millis() - lastDebug > 500) {
                Serial.printf("📱 Touch: RAW(%d,%d) -> Screen(%d,%d)\n", 
                             p.x, p.y, data->point.x, data->point.y);
                lastDebug = millis();
            }

        } else {
            data->state = LV_INDEV_STATE_REL;
        }
        
        xSemaphoreGive(spiMutex);
        
    } else {
        // Si timeout SPI, relâcher le touch
        data->state = LV_INDEV_STATE_REL;
    }
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touchX, touchY;
    bool touched = tft.getTouch(&touchX, &touchY);

}

// ========================================
// PARTIE 3 : CALLBACKS BOUTONS
// ========================================

// Callback pour boutons +/-
void btn_adjust_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    // Feedback visuel sur PRESSED
    if (code == LV_EVENT_PRESSED) {
        lv_obj_t *btn = lv_event_get_target(e);
        lv_obj_set_style_bg_opa(btn, LV_OPA_70, LV_STATE_PRESSED);
        Serial.println("🔘 Bouton pressé!");
    }
    
    if (code == LV_EVENT_CLICKED) {
        int direction = (int)(intptr_t)lv_event_get_user_data(e);
        
        Serial.printf("✅ Ajustement: %+d sur écran %d\n", direction, currentScreen);
        
        // ===== LOGIQUE D'AJUSTEMENT INTÉGRÉE =====
        float increment = 0.0;
        char buf[32];
        
        switch(currentScreen) {
            case 0: // Monitoring - lecture seule
                Serial.println("Monitoring screen - no adjustment");
                break;
                
            case 1: // Setpoint
                increment = 1.0;
                Setpoint += direction * increment;
                Setpoint = constrain(Setpoint, 80.0, 200.0);
                EEPROM.put(EEPROM_SETPOINT_ADDR, (float)Setpoint);
                
                if (label_setpoint_val != NULL) {
                    snprintf(buf, sizeof(buf), "%.1f", Setpoint);
                    lv_label_set_text(label_setpoint_val, buf);
                }
                Serial.printf("Setpoint: %.1f V\n", Setpoint);
                break;
                
            case 2: // Voltage correction factor
                increment = 0.01;
                voltage_correction_factor += direction * increment;
                voltage_correction_factor = constrain(voltage_correction_factor, 0.5, 2.0);
                EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, (float)voltage_correction_factor);
                
                if (label_correction_val != NULL) {
                    snprintf(buf, sizeof(buf), "%.2f", voltage_correction_factor);
                    lv_label_set_text(label_correction_val, buf);
                }
                Serial.printf("Correction: %.2f\n", voltage_correction_factor);
                break;
              
            case 3: // Kp
                increment = 0.5;
                    Serial.printf("\n🔍 === CASE 3 (Kp) ===\n");
                Serial.printf("   Kp AVANT tout: %.10f\n", Kp);  // 10 décimales
                Serial.printf("   &Kp = %p\n", &Kp);
                Kp += direction * increment;
                Kp = constrain(Kp, 0.0, 10.0);
                myPID.SetTunings(Kp, Ki, Kd);
                EEPROM.put(EEPROM_KP_ADDR, (float)Kp);
                
                if (label_kp_val != NULL) {
                    snprintf(buf, sizeof(buf), "%.1f", Kp);
                    lv_label_set_text(label_kp_val, buf);
                }
                Serial.printf("Kp: %.1f\n", Kp);
                break;
                
            case 4: // Ki
                increment = 0.1;
                Ki += direction * increment;
                Ki = constrain(Ki, 0.0, 10.0);
                myPID.SetTunings(Kp, Ki, Kd);
                EEPROM.put(EEPROM_KI_ADDR, (float)Ki);
                
                if (label_ki_val != NULL) {
                    snprintf(buf, sizeof(buf), "%.1f", Ki);
                    lv_label_set_text(label_ki_val, buf);
                }
                Serial.printf("Ki: %.1f\n", Ki);
                break;

            case 5: // Steps per mm
                increment = 10.0;
                STEPS_PER_MM_Z += direction * increment;
                STEPS_PER_MM_Z = constrain(STEPS_PER_MM_Z, 200.0, 2000.0);
                EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, (float)STEPS_PER_MM_Z);
                
                if (label_steps_val != NULL) {
                    snprintf(buf, sizeof(buf), "%.0f", STEPS_PER_MM_Z);
                    lv_label_set_text(label_steps_val, buf);
                }
                Serial.printf("Steps/mm: %.0f\n", STEPS_PER_MM_Z);
                break;
        }
    }
}

void btn_nav_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        // Récupérer l'ID
        int target_screen_id = (int)(intptr_t)lv_event_get_user_data(e);
        
        Serial.printf("🔍 Navigation demandée vers ID: %d\n", target_screen_id);
        
        // ✅ LOGS DE DEBUG CRITIQUES
        Serial.printf("   screen_monitoring = %p\n", screen_monitoring);
        Serial.printf("   screen_setpoint = %p\n", screen_setpoint);
        Serial.printf("   screen_correction = %p\n", screen_correction);
        Serial.printf("   screen_kp = %p\n", screen_kp);
        Serial.printf("   screen_ki = %p\n", screen_ki);
        Serial.printf("   screen_steps = %p\n", screen_steps);
        
        lv_obj_t *target_screen = NULL;
        
        // Convertir l'ID en pointeur
        switch(target_screen_id) {
            case 0: 
                target_screen = screen_monitoring;
                break;
            case 1: 
                target_screen = screen_setpoint;
                break;
            case 2: 
                target_screen = screen_correction;
                break;
            case 3: 
                target_screen = screen_kp;
                break;
            case 4: 
                target_screen = screen_ki;
                break;
            case 5: 
                target_screen = screen_steps;
                break;
            case 6: target_screen = screen_settings; break;
        }
        
        // ✅ VÉRIFICATION CRITIQUE AVANT lv_scr_load_anim
        if (target_screen == NULL) {
            Serial.printf("❌ ERREUR FATALE: target_screen est NULL pour ID %d!\n", target_screen_id);
            Serial.println("   Les écrans ont été corrompus ou non créés correctement.");
            return;
        }
        
        // ✅ VÉRIFIER QUE L'OBJET LVGL EST VALIDE
        if (!lv_obj_is_valid(target_screen)) {
            Serial.printf("❌ ERREUR: target_screen %p n'est PAS un objet LVGL valide!\n", target_screen);
            return;
        }
        
        Serial.println("   ✅ Tous les checks OK, lancement animation...");
        
        // Sauvegarder EEPROM
        EEPROM.commit();
        Serial.println("💾 EEPROM sauvegardée");
        
        // Mise à jour currentScreen
        currentScreen = target_screen_id;
        
        Serial.printf("🔄 Navigation vers écran %d (%p)\n", target_screen_id, target_screen);
        
        // ✅ APPEL SÉCURISÉ
        lv_scr_load_anim(target_screen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false);
        
        Serial.println("✅ Animation lancée avec succès!");
    }
}

void create_screen_monitoring() {
    // SÉCURITÉ : Si l'écran existe déjà, on ne le recrée pas
    // if (screen_monitoring != NULL) {
    //     Serial.println("⚠️ Monitoring déjà créé, on ignore.");
    //     return; 
    // }

    screen_monitoring = lv_obj_create(NULL);
    
    // === HEADER ===
    lv_obj_t *header = lv_obj_create(screen_monitoring);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label_title = lv_label_create(header);
    lv_label_set_text(label_title, "THC - MONITORING");
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label_title);

    // === PANEL PRINCIPAL ===
    lv_obj_t *panel = lv_obj_create(screen_monitoring);
    lv_obj_set_size(panel, 460, 200);
    lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_bg_color(panel, lv_color_white(), 0);

    // Voltage Fast (Cyan)
    lv_obj_t *label = lv_label_create(panel);
    lv_label_set_text(label, "Voltage Fast:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 10);
    
    label_voltage_fast = lv_label_create(panel);
    lv_label_set_text(label_voltage_fast, "0.0 V");
    lv_obj_set_style_text_color(label_voltage_fast, lv_color_hex(0xFF0000), 0);
    lv_obj_align(label_voltage_fast, LV_ALIGN_TOP_RIGHT, -10, 10);

    // Voltage Slow
    label = lv_label_create(panel);
    lv_label_set_text(label, "Voltage Slow:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 35);
    
    label_voltage_slow = lv_label_create(panel);
    lv_label_set_text(label_voltage_slow, "0.0 V");
    lv_obj_set_style_text_color(label_voltage_slow, lv_color_hex(0xFF0000), 0);
    lv_obj_align(label_voltage_slow, LV_ALIGN_TOP_RIGHT, -10, 35);

    // Setpoint (Vert)
    label = lv_label_create(panel);
    lv_label_set_text(label, "Setpoint:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 60);
    
    label_setpoint = lv_label_create(panel);
    lv_label_set_text(label_setpoint, "110.0 V");
    lv_obj_set_style_text_color(label_setpoint, lv_color_hex(0xFF0000), 0);
    lv_obj_align(label_setpoint, LV_ALIGN_TOP_RIGHT, -10, 60);

    // Position
    label = lv_label_create(panel);
    lv_label_set_text(label, "Position:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 85);
    
    label_position = lv_label_create(panel);
    lv_label_set_text(label_position, "0 steps");
    lv_obj_set_style_text_color(label_position, lv_color_hex(0xFF0000), 0);
    lv_obj_align(label_position, LV_ALIGN_TOP_RIGHT, -10, 85);

    // === STATUTS (avec LED colorées) ===
    int y_status = 110;
    
    // THC State
    label = lv_label_create(panel);
    lv_label_set_text(label, "THC State:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, y_status);
    
    label_thc_state = lv_label_create(panel);
    lv_label_set_text(label_thc_state, LV_SYMBOL_STOP " INACTIF");
    lv_obj_set_style_text_color(label_thc_state, lv_color_hex(0xFF0000), 0);
    lv_obj_align(label_thc_state, LV_ALIGN_TOP_RIGHT, -10, y_status);

    // Enable
    label = lv_label_create(panel);
    lv_label_set_text(label, "Enable:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, y_status + 25);
    
    label_enable_state = lv_label_create(panel);
    lv_label_set_text(label_enable_state, LV_SYMBOL_STOP " INACTIF");
    lv_obj_align(label_enable_state, LV_ALIGN_TOP_RIGHT, -10, y_status + 25);

    // Anti-Dive
    label = lv_label_create(panel);
    lv_label_set_text(label, "Anti-Dive:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, y_status + 50);
    
    label_antidive_state = lv_label_create(panel);
    lv_label_set_text(label_antidive_state, LV_SYMBOL_STOP " INACTIF");
    lv_obj_align(label_antidive_state, LV_ALIGN_TOP_RIGHT, -10, y_status + 50);

    // Arc Voltage
    label = lv_label_create(panel);
    lv_label_set_text(label, "Arc Voltage:");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, y_status + 75);
    
    label_arc_state = lv_label_create(panel);
    lv_label_set_text(label_arc_state, LV_SYMBOL_CLOSE " NOK");
    lv_obj_set_style_text_color(label_arc_state, lv_color_hex(0xFF0000), 0);
    lv_obj_align(label_arc_state, LV_ALIGN_TOP_RIGHT, -10, y_status + 75);

    // === BOUTONS NAVIGATION ===
    lv_obj_t *btn_next = lv_btn_create(screen_monitoring);
    lv_obj_set_size(btn_next, 150, 50);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_next, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)1);
    
    label = lv_label_create(btn_next);
    lv_label_set_text(label, LV_SYMBOL_RIGHT " Suivant");
    lv_obj_center(label);
}

// Callback pour changer de paramètre
void btn_change_param_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        int direction = (int)(intptr_t)lv_event_get_user_data(e);
        
        selected_param += direction;
        
        // Boucler entre 0 et 4
        if (selected_param < 0) selected_param = 4;
        if (selected_param > 4) selected_param = 0;
        
        // Mettre à jour l'affichage
        lv_label_set_text(label_param_name, param_names[selected_param]);
        update_param_value_display();
        
        Serial.printf("Paramètre sélectionné: %s\n", param_names[selected_param]);
    }
}

// Callback pour ajuster la valeur
void btn_adjust_multi_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        int direction = (int)(intptr_t)lv_event_get_user_data(e);
        char buf[32];
        
        switch(selected_param) {
            case 0: // Setpoint
                Setpoint += direction * 1.0;
                Setpoint = constrain(Setpoint, 80.0, 200.0);
                EEPROM.put(EEPROM_SETPOINT_ADDR, (float)Setpoint);
                snprintf(buf, sizeof(buf), "%.1f", Setpoint);
                break;
                
            case 1: // Correction
                voltage_correction_factor += direction * 0.01;
                voltage_correction_factor = constrain(voltage_correction_factor, 0.5, 2.0);
                EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, (float)voltage_correction_factor);
                snprintf(buf, sizeof(buf), "%.2f", voltage_correction_factor);
                break;
                
            case 2: // Kp
                Kp += direction * 0.5;
                Kp = constrain(Kp, 0.0, 10.0);
                myPID.SetTunings(Kp, Ki, Kd);
                EEPROM.put(EEPROM_KP_ADDR, (float)Kp);
                snprintf(buf, sizeof(buf), "%.1f", Kp);
                break;
                
            case 3: // Ki
                Ki += direction * 0.1;
                Ki = constrain(Ki, 0.0, 10.0);
                myPID.SetTunings(Kp, Ki, Kd);
                EEPROM.put(EEPROM_KI_ADDR, (float)Ki);
                snprintf(buf, sizeof(buf), "%.1f", Ki);
                break;
                
            case 4: // Steps
                STEPS_PER_MM_Z += direction * 10.0;
                STEPS_PER_MM_Z = constrain(STEPS_PER_MM_Z, 200.0, 2000.0);
                EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, (float)STEPS_PER_MM_Z);
                snprintf(buf, sizeof(buf), "%.0f", STEPS_PER_MM_Z);
                break;
        }
        
        lv_label_set_text(label_param_value, buf);
    }
}

void update_param_value_display() {
    char buf[32];
    
    switch(selected_param) {
        case 0: snprintf(buf, sizeof(buf), "%.1f", Setpoint); break;
        case 1: snprintf(buf, sizeof(buf), "%.2f", voltage_correction_factor); break;
        case 2: snprintf(buf, sizeof(buf), "%.1f", Kp); break;
        case 3: snprintf(buf, sizeof(buf), "%.1f", Ki); break;
        case 4: snprintf(buf, sizeof(buf), "%.0f", STEPS_PER_MM_Z); break;
    }
    
    lv_label_set_text(label_param_value, buf);
}

void create_screen_setpoint() {
    screen_setpoint = lv_obj_create(NULL);
    
    // Header
    lv_obj_t *header = lv_obj_create(screen_setpoint);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label = lv_label_create(header);
    lv_label_set_text(label, "AJUSTER CONSIGNE");
    lv_obj_set_style_text_color(label, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label);

    // Affichage valeur
    label = lv_label_create(screen_setpoint);
    lv_label_set_text(label, "Consigne (V):");
    lv_obj_align(label, LV_ALIGN_CENTER, -60, -50);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    
    label_setpoint_val = lv_label_create(screen_setpoint);
    lv_label_set_text(label_setpoint_val, "110.0");
    lv_obj_align(label_setpoint_val, LV_ALIGN_CENTER, -60, 0);
    lv_obj_set_style_text_font(label_setpoint_val, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label_setpoint_val, lv_color_hex(0xFF2F2F), 0);

    // Bouton +
    lv_obj_t *btn_up = lv_btn_create(screen_setpoint);
    lv_obj_set_size(btn_up, 120, 60);
    lv_obj_align(btn_up, LV_ALIGN_RIGHT_MID, -20, -40);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x27AE60), 0);
    lv_obj_add_event_cb(btn_up, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    
    label = lv_label_create(btn_up);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    // Bouton -
    lv_obj_t *btn_down = lv_btn_create(screen_setpoint);
    lv_obj_set_size(btn_down, 120, 60);
    lv_obj_align(btn_down, LV_ALIGN_RIGHT_MID, -20, 40);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0xE74C3C), 0);
    lv_obj_add_event_cb(btn_down, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    
    label = lv_label_create(btn_down);
    lv_label_set_text(label, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    // Boutons navigation
    lv_obj_t *btn_prev = lv_btn_create(screen_setpoint);
    lv_obj_set_size(btn_prev, 140, 50);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(btn_prev, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_prev);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Retour");
    lv_obj_center(label);

    lv_obj_t *btn_home = lv_btn_create(screen_setpoint);
    lv_obj_set_size(btn_home, 140, 50);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_home, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_home);
    lv_label_set_text(label, LV_SYMBOL_HOME " Home");
    lv_obj_center(label);
    
    lv_obj_t *btn_next = lv_btn_create(screen_setpoint);
    lv_obj_set_size(btn_next, 140, 50);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_next, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)2);
    
    label = lv_label_create(btn_next);
    lv_label_set_text(label, "Suivant " LV_SYMBOL_RIGHT);
    lv_obj_center(label);
}

void create_screen_correction() {
    screen_correction = lv_obj_create(NULL);
    
    // Header
    lv_obj_t *header = lv_obj_create(screen_correction);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label = lv_label_create(header);
    lv_label_set_text(label, "CORRECTION VOLTAGE");
    lv_obj_set_style_text_color(label, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label);

    // Affichage valeur
    label = lv_label_create(screen_correction);
    lv_label_set_text(label, "Facteur:");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -50);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    
    label_correction_val = lv_label_create(screen_correction);
    lv_label_set_text(label_correction_val, "1.00");
    lv_obj_align(label_correction_val, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label_correction_val, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label_correction_val, lv_color_hex(0xFFAA00), 0);

    // Boutons +/-
    lv_obj_t *btn_up = lv_btn_create(screen_correction);
    lv_obj_set_size(btn_up, 120, 70);
    lv_obj_align(btn_up, LV_ALIGN_RIGHT_MID, -20, -70);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x27AE60), 0);
    lv_obj_add_event_cb(btn_up, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    
    label = lv_label_create(btn_up);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_down = lv_btn_create(screen_correction);
    lv_obj_set_size(btn_down, 120, 70);
    lv_obj_align(btn_down, LV_ALIGN_RIGHT_MID, -20, 70);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0xE74C3C), 0);
    lv_obj_add_event_cb(btn_down, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    label = lv_label_create(btn_down);
    lv_label_set_text(label, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    // Navigation
    lv_obj_t *btn_prev = lv_btn_create(screen_correction);
    lv_obj_set_size(btn_prev, 140, 50);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(btn_prev, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)1);
    
    label = lv_label_create(btn_prev);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Retour");
    lv_obj_center(label);

    lv_obj_t *btn_home = lv_btn_create(screen_correction);
    lv_obj_set_size(btn_home, 140, 50);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_home, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_home);
    lv_label_set_text(label, LV_SYMBOL_HOME " Home");
    lv_obj_center(label);
    
    lv_obj_t *btn_next = lv_btn_create(screen_correction);
    lv_obj_set_size(btn_next, 140, 50);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_next, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)3);
    
    label = lv_label_create(btn_next);
    lv_label_set_text(label, "Suivant " LV_SYMBOL_RIGHT);
    lv_obj_center(label);
}

void create_screen_kp() {
    screen_kp = lv_obj_create(NULL);
    
    lv_obj_t *header = lv_obj_create(screen_kp);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label = lv_label_create(header);
    lv_label_set_text(label, "REGLAGE Kp");
    lv_obj_set_style_text_color(label, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label);

    label = lv_label_create(screen_kp);
    lv_label_set_text(label, "Kp:");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -50);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    
    label_kp_val = lv_label_create(screen_kp);
    lv_label_set_text(label_kp_val, "2.0");
    lv_obj_align(label_kp_val, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label_kp_val, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label_kp_val, lv_color_hex(0xFF5555), 0);

    lv_obj_t *btn_up = lv_btn_create(screen_kp);
    lv_obj_set_size(btn_up, 120, 70);
    lv_obj_align(btn_up, LV_ALIGN_RIGHT_MID, -20, -70);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x27AE60), 0);
    lv_obj_add_event_cb(btn_up, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    
    label = lv_label_create(btn_up);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_down = lv_btn_create(screen_kp);
    lv_obj_set_size(btn_down, 120, 70);
    lv_obj_align(btn_down, LV_ALIGN_RIGHT_MID, -20, 70);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0xE74C3C), 0);
    lv_obj_add_event_cb(btn_down, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    
    label = lv_label_create(btn_down);
    lv_label_set_text(label, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_prev = lv_btn_create(screen_kp);
    lv_obj_set_size(btn_prev, 140, 50);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(btn_prev, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)2);
    
    label = lv_label_create(btn_prev);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Retour");
    lv_obj_center(label);

    lv_obj_t *btn_home = lv_btn_create(screen_kp);
    lv_obj_set_size(btn_home, 140, 50);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_home, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_home);
    lv_label_set_text(label, LV_SYMBOL_HOME " Home");
    lv_obj_center(label);
    
    lv_obj_t *btn_next = lv_btn_create(screen_kp);
    lv_obj_set_size(btn_next, 140, 50);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_next, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)4);
    
    label = lv_label_create(btn_next);
    lv_label_set_text(label, "Suivant " LV_SYMBOL_RIGHT);
    lv_obj_center(label);
}

void create_screen_ki() {
    screen_ki = lv_obj_create(NULL);
    
    lv_obj_t *header = lv_obj_create(screen_ki);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label = lv_label_create(header);
    lv_label_set_text(label, "REGLAGE Ki");
    lv_obj_set_style_text_color(label, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label);

    label = lv_label_create(screen_ki);
    lv_label_set_text(label, "Ki:");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -50);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    
    label_ki_val = lv_label_create(screen_ki);
    lv_label_set_text(label_ki_val, "5.0");
    lv_obj_align(label_ki_val, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label_ki_val, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label_ki_val, lv_color_hex(0xFF5555), 0);

    lv_obj_t *btn_up = lv_btn_create(screen_ki);
    lv_obj_set_size(btn_up, 120, 70);
    lv_obj_align(btn_up, LV_ALIGN_RIGHT_MID, -20, -70);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x27AE60), 0);
    lv_obj_add_event_cb(btn_up, btn_adjust_event_handler, LV_EVENT_CLICKED,(void*)(intptr_t)1);
    
    label = lv_label_create(btn_up);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_down = lv_btn_create(screen_ki);
    lv_obj_set_size(btn_down, 120, 70);
    lv_obj_align(btn_down, LV_ALIGN_RIGHT_MID, -20, 70);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0xE74C3C), 0);
    lv_obj_add_event_cb(btn_down, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    
    label = lv_label_create(btn_down);
    lv_label_set_text(label, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_prev = lv_btn_create(screen_ki);
    lv_obj_set_size(btn_prev, 140, 50);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(btn_prev, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)3);
    
    label = lv_label_create(btn_prev);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Retour");
    lv_obj_center(label);

    lv_obj_t *btn_home = lv_btn_create(screen_ki);
    lv_obj_set_size(btn_home, 140, 50);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_home, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_home);
    lv_label_set_text(label, LV_SYMBOL_HOME " Home");
    lv_obj_center(label);
    
    lv_obj_t *btn_next = lv_btn_create(screen_ki);
    lv_obj_set_size(btn_next, 140, 50);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_next, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)5);
    
    label = lv_label_create(btn_next);
    lv_label_set_text(label, "Suivant " LV_SYMBOL_RIGHT);
    lv_obj_center(label);
}

void create_screen_steps() {
    screen_steps = lv_obj_create(NULL);
    
    lv_obj_t *header = lv_obj_create(screen_steps);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label = lv_label_create(header);
    lv_label_set_text(label, "STEPS/MM");
    lv_obj_set_style_text_color(label, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label);

    label = lv_label_create(screen_steps);
    lv_label_set_text(label, "Steps/mm:");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -50);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    
    label_steps_val = lv_label_create(screen_steps);
    lv_label_set_text(label_steps_val, "400");
    lv_obj_align(label_steps_val, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label_steps_val, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label_steps_val, lv_color_hex(0xFF5555), 0);

    lv_obj_t *btn_up = lv_btn_create(screen_steps);
    lv_obj_set_size(btn_up, 120, 70);
    lv_obj_align(btn_up, LV_ALIGN_RIGHT_MID, -20, -70);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x27AE60), 0);
    lv_obj_add_event_cb(btn_up, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    
    label = lv_label_create(btn_up);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_down = lv_btn_create(screen_steps);
    lv_obj_set_size(btn_down, 120, 70);
    lv_obj_align(btn_down, LV_ALIGN_RIGHT_MID, -20, 70);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0xE74C3C), 0);
    lv_obj_add_event_cb(btn_down, btn_adjust_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    
    label = lv_label_create(btn_down);
    lv_label_set_text(label, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    lv_obj_t *btn_prev = lv_btn_create(screen_steps);
    lv_obj_set_size(btn_prev, 140, 50);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(btn_prev, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)4);
    
    label = lv_label_create(btn_prev);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Retour");
    lv_obj_center(label);

    lv_obj_t *btn_home = lv_btn_create(screen_steps);
    lv_obj_set_size(btn_home, 140, 50);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_home, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_home);
    lv_label_set_text(label, LV_SYMBOL_HOME " Home");
    lv_obj_center(label);
    
    lv_obj_t *btn_next = lv_btn_create(screen_steps);
    lv_obj_set_size(btn_next, 140, 50);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_next, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)6);
    
    label = lv_label_create(btn_next);
    lv_label_set_text(label, LV_SYMBOL_HOME " Retour");
    lv_obj_center(label);
}


void create_screen_settings() {
    screen_settings = lv_obj_create(NULL);
    
    // === HEADER ===
    lv_obj_t *header = lv_obj_create(screen_settings);
    lv_obj_set_size(header, 480, 40);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    
    lv_obj_t *label = lv_label_create(header);
    lv_label_set_text(label, "REGLAGES RAPIDES");
    lv_obj_set_style_text_color(label, lv_color_hex(0xF1C40F), 0);
    lv_obj_center(label);
    
    // === NOM DU PARAMÈTRE ===
    label_param_name = lv_label_create(screen_settings);
    lv_label_set_text(label_param_name, param_names[0]);
    lv_obj_align(label_param_name, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_text_font(label_param_name, &lv_font_montserrat_24, 0);
    
    // === VALEUR DU PARAMÈTRE ===
    label_param_value = lv_label_create(screen_settings);
    lv_label_set_text(label_param_value, "110.0");
    lv_obj_align(label_param_value, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(label_param_value, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label_param_value, lv_color_hex(0x00FFFF), 0);
    
    // === BOUTONS SÉLECTION PARAMÈTRE (◀ ▶) ===
    lv_obj_t *btn_prev_param = lv_btn_create(screen_settings);
    lv_obj_set_size(btn_prev_param, 80, 60);
    lv_obj_align(btn_prev_param, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_set_style_bg_color(btn_prev_param, lv_color_hex(0x3498DB), 0);
    lv_obj_add_event_cb(btn_prev_param, btn_change_param_event, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    
    label = lv_label_create(btn_prev_param);
    lv_label_set_text(label, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);
    
    lv_obj_t *btn_next_param = lv_btn_create(screen_settings);
    lv_obj_set_size(btn_next_param, 80, 60);
    lv_obj_align(btn_next_param, LV_ALIGN_RIGHT_MID, -20, 0);
    lv_obj_set_style_bg_color(btn_next_param, lv_color_hex(0x3498DB), 0);
    lv_obj_add_event_cb(btn_next_param, btn_change_param_event, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    
    label = lv_label_create(btn_next_param);
    lv_label_set_text(label, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);
    
    // === BOUTONS +/- VALEUR ===
    lv_obj_t *btn_up = lv_btn_create(screen_settings);
    lv_obj_set_size(btn_up, 100, 60);
    lv_obj_align(btn_up, LV_ALIGN_BOTTOM_RIGHT, -20, -80);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x27AE60), 0);
    lv_obj_add_event_cb(btn_up, btn_adjust_multi_event, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    
    label = lv_label_create(btn_up);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);
    
    lv_obj_t *btn_down = lv_btn_create(screen_settings);
    lv_obj_set_size(btn_down, 100, 60);
    lv_obj_align(btn_down, LV_ALIGN_BOTTOM_LEFT, 20, -80);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0xE74C3C), 0);
    lv_obj_add_event_cb(btn_down, btn_adjust_multi_event, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    
    label = lv_label_create(btn_down);
    lv_label_set_text(label, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);
    
    // === BOUTON HOME ===
    lv_obj_t *btn_home = lv_btn_create(screen_settings);
    lv_obj_set_size(btn_home, 140, 50);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_home, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    label = lv_label_create(btn_home);
    lv_label_set_text(label, LV_SYMBOL_HOME " Home");
    lv_obj_center(label);
}



void taskLvglTick(void *pvParameters) {
    for (;;) {
        lv_tick_inc(1);                 // ⬅️ 1 ms
        vTaskDelay(pdMS_TO_TICKS(1));   // ⬅️ tick à 1 kHz
    }
}

void lvgl_setup() {
    Serial.println("Initialisation LVGL...");
    lv_init();
    
    // Buffer display
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LVGL_BUFFER_SIZE);

    // Driver display
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    
    if (disp != NULL) {
        Serial.println("   ✅ Display driver enregistré");
    } else {
        Serial.println("   ❌ ERREUR Display driver");
    }

    // ========================================
    // SECTION TACTILE AVEC DIAGNOSTIC COMPLET
    // ========================================
 lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lv_touchpad_read;
    
    // DIAGNOSTIC : Vérifier avant enregistrement
    Serial.printf("   Driver tactile avant enregistrement:\n");
    Serial.printf("   - Type: %d (devrait être %d)\n", indev_drv.type, LV_INDEV_TYPE_POINTER);
    Serial.printf("   - Callback: %p\n", indev_drv.read_cb);
    
    lv_indev_t *touch_indev = lv_indev_drv_register(&indev_drv);
    
    if (touch_indev != NULL) {
        Serial.println("   ✅ Touch driver enregistré");
        
        // DIAGNOSTIC : Vérifier après enregistrement
        Serial.printf("   - Pointeur indev: %p\n", touch_indev);
        Serial.printf("   - Type après enregistrement: %d\n", touch_indev->driver->type);
        Serial.printf("   - Callback après enregistrement: %p\n", touch_indev->driver->read_cb);
        
        // TEST FORCÉ : Appeler manuellement le callback
        Serial.println("   🧪 Test manuel du callback tactile...");
        lv_indev_data_t test_data;
        lv_touchpad_read(touch_indev->driver, &test_data);
        Serial.printf("   État retourné: %d\n", test_data.state);
    }else {
        Serial.println("   ❌ ERREUR: Touch driver NON enregistré!");
    }

// ✅ CRÉER TOUS LES ÉCRANS
    Serial.println("📄 Création des écrans..."); 
    create_screen_monitoring();
    Serial.printf("   ✅ Monitoring créé: %p\n", screen_monitoring);
    create_screen_setpoint();
    Serial.printf("   ✅ Setpoint créé: %p\n", screen_setpoint);
    create_screen_correction();
    Serial.printf("   ✅ Correction créé: %p\n", screen_correction);
    create_screen_kp();
    Serial.printf("   ✅ Kp créé: %p\n", screen_kp);
    create_screen_ki();
    Serial.printf("   ✅ Ki créé: %p\n", screen_ki);
    create_screen_steps();
    Serial.printf("   ✅ Steps créé: %p\n", screen_steps);
    create_screen_settings();
    Serial.printf("   ✅ Settings créé: %p\n", screen_settings);
        // ✅ VÉRIFICATION CRITIQUE
    if (screen_monitoring == NULL || screen_setpoint == NULL) {
        Serial.println("❌ ERREUR FATALE: Écrans non créés!");
        while(1) { delay(1000); } // Bloquer pour debug
    }
    // Charger l'écran principal
    lv_scr_load(screen_monitoring);
    Serial.println("LVGL initialisé avec succès");
    // ✅ DIAGNOSTIC FINAL
    vTaskDelay(pdMS_TO_TICKS(500));
    //diagnose_lvgl_touch();
}


// ========================================
// PARTIE 7 : MISE À JOUR DES VALEURS
// ========================================
void update_lvgl_labels_safe(DisplayData* data) {
    static char buf[32];
    

    // === VOLTAGE FAST ===
    snprintf(buf, sizeof(buf), "%.1f V", data->fast_voltage);
    lv_label_set_text(label_voltage_fast, buf);
    lv_obj_invalidate(label_voltage_fast);
   
    
    // === VOLTAGE SLOW ===
    snprintf(buf, sizeof(buf), "%.1f V", data->slow_voltage);
    lv_label_set_text(label_voltage_slow, buf);
    lv_obj_invalidate(label_voltage_slow);
    
    // === SETPOINT ===
    snprintf(buf, sizeof(buf), "%.1f V", data->setpoint);
    lv_label_set_text(label_setpoint, buf);
    lv_obj_invalidate(label_setpoint);
    
    // === POSITION ===
    snprintf(buf, sizeof(buf), "%ld steps", data->position);
    lv_label_set_text(label_position, buf);
    lv_obj_invalidate(label_position);
    
    // === THC STATE ===
    if (data->thc_active) {
        lv_label_set_text(label_thc_state, LV_SYMBOL_PLAY " ACTIF");
        lv_obj_set_style_text_color(label_thc_state, lv_color_hex(0x00FF00), 0);
    } else {
        lv_label_set_text(label_thc_state, LV_SYMBOL_STOP " INACTIF");
        lv_obj_set_style_text_color(label_thc_state, lv_color_hex(0xFF0000), 0);
    }
    lv_obj_invalidate(label_thc_state);
    
    // === ENABLE STATE ===
    if (data->enable_active) {
        lv_label_set_text(label_enable_state, LV_SYMBOL_OK " ACTIF");
        lv_obj_set_style_text_color(label_enable_state, lv_color_hex(0x00FF00), 0);
    } else {
        lv_label_set_text(label_enable_state, LV_SYMBOL_CLOSE " INACTIF");
        lv_obj_set_style_text_color(label_enable_state, lv_color_hex(0xFF0000), 0);
    }
    lv_obj_invalidate(label_enable_state);
    
    // === ANTI-DIVE STATE ===
    if (data->anti_dive_active) {
        lv_label_set_text(label_antidive_state, LV_SYMBOL_WARNING " ACTIF");
        lv_obj_set_style_text_color(label_antidive_state, lv_color_hex(0xFFAA00), 0);
    } else {
        lv_label_set_text(label_antidive_state, LV_SYMBOL_OK " INACTIF");
        lv_obj_set_style_text_color(label_antidive_state, lv_color_hex(0x00FF00), 0);
    }
    lv_obj_invalidate(label_antidive_state);
    
    // === ARC VOLTAGE STATE ===
    if (data->arc_ok) {
        lv_label_set_text(label_arc_state, LV_SYMBOL_OK " OK");
        lv_obj_set_style_text_color(label_arc_state, lv_color_hex(0x00FF00), 0);
    } else {
        lv_label_set_text(label_arc_state, LV_SYMBOL_CLOSE " NOK");
        lv_obj_set_style_text_color(label_arc_state, lv_color_hex(0xFF0000), 0);
    }
    lv_obj_invalidate(label_arc_state);


}

void update_all_screen_values() {
    char buf[32];
    
    // Écran Monitoring
    if (label_setpoint != NULL) {
        snprintf(buf, sizeof(buf), "%.1f V", Setpoint);
        lv_label_set_text(label_setpoint, buf);
    }
    
    // Écran Setpoint
    if (label_setpoint_val != NULL) {
        snprintf(buf, sizeof(buf), "%.1f", Setpoint);
        lv_label_set_text(label_setpoint_val, buf);
    }
    
    // Écran Correction
    if (label_correction_val != NULL) {
        snprintf(buf, sizeof(buf), "%.2f", voltage_correction_factor);
        lv_label_set_text(label_correction_val, buf);
    }
    
    // Écran Kp
    if (label_kp_val != NULL) {
        snprintf(buf, sizeof(buf), "%.1f", Kp);
        lv_label_set_text(label_kp_val, buf);
    }
    
    // Écran Ki
    if (label_ki_val != NULL) {
        snprintf(buf, sizeof(buf), "%.1f", Ki);
        lv_label_set_text(label_ki_val, buf);
    }
    
    // Écran Steps
    if (label_steps_val != NULL) {
        snprintf(buf, sizeof(buf), "%.0f", STEPS_PER_MM_Z);
        lv_label_set_text(label_steps_val, buf);
    }
}


void taskUI(void *pvParameters) {
    Serial.println("📱 Task UI démarrée sur Core 1");
    
    unsigned long lastDebug = 0;
    unsigned long lastLabelUpdate = 0;
    int loopCount = 0;
    
    // Copie locale des données
    DisplayData local_data = {0};
    
    for (;;) {
        lv_timer_handler();
        loopCount++;
        
        // === COPIER LES DONNÉES AVEC MUTEX ===
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_data = display_data;  // Copie rapide
            xSemaphoreGive(dataMutex);
        }
        
        // === MISE À JOUR DES LABELS (avec données locales) ===
        if (millis() - lastLabelUpdate >= 200) {
            update_lvgl_labels_safe(&local_data);
            lastLabelUpdate = millis();
        }
        
        if (millis() - lastDebug > 5000) {
            Serial.printf("📱 Task UI: %d appels | Fast: %.1fV | Slow: %.1fV\n", 
                         loopCount, local_data.fast_voltage, local_data.slow_voltage);
            loopCount = 0;
            lastDebug = millis();
        }
        
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


void setup() {
  i2cMutex = xSemaphoreCreateMutex();
  spiMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

      if (!i2cMutex || !spiMutex || !dataMutex) {
        Serial.println("❌ ERREUR création mutex!");
        while(1);
    }

  // --- 1. Initialisation EEPROM (Spécifique ESP32) ---
  // On réserve 512 octets de mémoire flash pour émuler l'EEPROM
  Serial.begin(115200);
  delay(1000);
//   
// diagnose_lvgl_touch();
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialise EEPROM");

    delay(1000);
    ESP.restart();
  }
  // Initialize EEPROM with default values if not already initialized
  initializeEEPROM();

  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);

  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_DIR_PIN, LOW);

  pinMode(ENABLE_PIN, INPUT_PULLUP);
  // Sur ESP32, INPUT simple suffit souvent, mais INPUT_PULLUP peut stabiliser
  pinMode(CNC_Z_STEP_IN, INPUT);
  pinMode(CNC_Z_DIR_IN, INPUT);

  // --- 3. I2C ---
  Wire.begin(ADS_I2C_SDA_PIN, ADS_I2C_SCL_PIN); // Démarre I2C sur SDA(17) et SCL(22)
  Wire.setClock(400000);  
    // Initialisation I²C de l'ADS1115
  Serial.println("Initialisation de l'ADS1115...");
  if (!ads.begin(ADS1115_ADDR_GND)) {
    Serial.println("ERREUR: ADS1115 non détecté!");
  } else {
    Serial.println("ADS1115 OK");
    ads.setGain(GAIN_ONE); // ±4.096V
  }

   //AJOUT TFT : Initialisation de l'écran
    tft.init();
    tft.setRotation(1);     // Rotation 1 (paysage) pour un ESP32/TFT typique
    tft.fillScreen(TFT_BLACK);
    tft.writecommand(0x01); // Software Reset
    delay(120);
    
    tft.writecommand(0x11); // Sleep Out
    delay(120);
    
    tft.writecommand(0x3A); // Pixel Format
    tft.writedata(0x55);    // 16-bit
    
    tft.writecommand(0x36); // MADCTL
    tft.writedata(0x28);    // MV + BGR (CONFIG 4 VALIDÉE)
    
    tft.writecommand(0x20); // Inversion OFF
    
    tft.writecommand(0x29); // Display ON
    delay(50);
    // ✅ INIT XPT2046 APRÈS TFT
    if (!ts.begin()) {
        Serial.println("❌ XPT2046 échec!");
    }
    ts.setRotation(1);

    // Initialisation LVGL (APRÈS TFT)
    lvgl_setup();

    update_all_screen_values();

  for (int i = 0; i < speed_filter_size; i++) {
    speed_readings[i] = 0.0;
  }

  myPID.SetOutputLimits(-100, 100); 
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetSampleTime(1); 
  

  // Validation des valeurs (inchangé)
  if (isnan(Setpoint) || Setpoint < 80 || Setpoint > 200) Setpoint = DEFAULT_SETPOINT;
  if (isnan(voltage_correction_factor) || voltage_correction_factor < 0.5 || voltage_correction_factor > 2.0) voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;
  if (isnan(STEPS_PER_MM_Z) || STEPS_PER_MM_Z < 200 || STEPS_PER_MM_Z > 2000) STEPS_PER_MM_Z = DEFAULT_STEP_PER_MM;
  //if (isnan(Kp) || Kp < 0.0 || Kp > 10) Kp = DEFAULT_KP;
  if (isnan(Ki) || Ki < 0.0 || Ki > 10) Ki = DEFAULT_KI;
  if (isnan(Kd) || Kd < 0.0 || Kd > 0.1) Kd = DEFAULT_KD;
  myPID.SetTunings(Kp, Ki, Kd);
  
  //threshold_speed = cut_speed * threshold_ratio;

  // --- 4. Résolution ADC ---
  // L'ESP32 est 12 bits max (0-4095). 
  // Si vous aviez des calculs basés sur 14 bits (16383), il faudra les diviser par 4.
  analogReadResolution(12);

  // New: Initialize position history
  for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
    position_history[i].time = 0;
    position_history[i].position = 0;
  }
BaseType_t taskCreated = xTaskCreatePinnedToCore(
    taskUI,
    "TaskUI",
    8192,
    NULL,
    2,
    &taskUIHandle,
    1
);
xTaskCreatePinnedToCore(
    taskLvglTick,
    "LVGL_Tick",
    2048,
    NULL,
    3,      // priorité > UI
    NULL,
    1
);

    if (taskCreated == pdPASS) {
        Serial.println("✅ Task UI créée avec succès");
    } else {
        Serial.println("❌ ERREUR: Task UI non créée!");
    }
}

void simulateSineWave() {
    if (!simulation_mode) return;
    
    unsigned long elapsed = millis() - simulation_start_time;
    float time_sec = elapsed / 1000.0;
    
    // Calcul sinusoïde : Input = Setpoint + A*sin(2πft) + offset
    float angle = 2.0 * PI * simulation_frequency * time_sec;
    Input = Setpoint + simulation_amplitude * sin(angle) + simulation_offset;
    
    // Marque input_ready pour que le PID compute
    input_ready = true;
    
    // Simule aussi fast_voltage pour l'affichage
    fast_voltage = Input;
    slow_voltage = Input;
    arc_voltage_ok = true;  // Toujours arc OK en simulation
}


void initializeEEPROM() {
    uint8_t initializedFlag = 0;

    EEPROM.get(EEPROM_INITIALIZED_FLAG, initializedFlag);

    if (initializedFlag != 0xAA) {
        // Initialize EEPROM with default values
        Serial.println("Initialisation EEPROM en cours..."); 
        EEPROM.put(EEPROM_SETPOINT_ADDR, DEFAULT_SETPOINT);
        EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
        EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, DEFAULT_STEP_PER_MM);
        EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP); // Écrit la valeur non nulle (p. ex. 100.0)
        EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
        EEPROM.put(EEPROM_KD_ADDR, DEFAULT_KD);

        // Set the initialized flag
        initializedFlag = 0xAA;
        EEPROM.put(EEPROM_INITIALIZED_FLAG, initializedFlag);
        
        // Sauvegarder les changements (CRITIQUE sur ESP32)
        EEPROM.commit();
        Serial.println("EEPROM initialized with default values");
    }

    // --- SECTION DE CHARGEMENT ---
    
    // Load parameters from EEPROM
    float temp;
    EEPROM.get(EEPROM_SETPOINT_ADDR, temp);
    Setpoint = temp;
    Serial.print("Loaded Setpoint: "); Serial.println(Setpoint, 2);

    EEPROM.get(EEPROM_CORRECTION_FACTOR_ADDR, temp);
    voltage_correction_factor = temp;
    Serial.print("Loaded voltage_correction_factor: "); Serial.println(voltage_correction_factor, 2);

    EEPROM.get(EEPROM_KP_ADDR, temp);
    Kp = temp;
    Serial.print("Loaded Kp: "); Serial.println(Kp, 2);

    EEPROM.get(EEPROM_KI_ADDR, temp);
    Ki = temp;
    Serial.print("Loaded Ki: "); Serial.println(Ki, 4);

    EEPROM.get(EEPROM_KD_ADDR, temp);
    Kd = temp;
    Serial.print("Loaded Kd: "); Serial.println(Kd, 4);

    EEPROM.get(EEPROM_STEPS_MM_Z_ADDR, STEPS_PER_MM_Z);
    Serial.print("Loaded steps par mm: "); Serial.println(STEPS_PER_MM_Z, 4);
}

 void loop() {
    // ========================================
    // VARIABLES STATIQUES (déclarées UNE SEULE FOIS)
    // ========================================
    static unsigned long lastTouchCheck = 0;
    static bool was_touched = false;
    static int last_x = 0, last_y = 0;

    static unsigned long lastDataUpdate = 0;
    if (millis() - lastDataUpdate >= 100) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            display_data.fast_voltage = fast_voltage;
            display_data.slow_voltage = slow_voltage;
            display_data.setpoint = Setpoint;
            display_data.position = stepper.currentPosition();
            display_data.thc_active = thc_active;
            display_data.enable_active = (digitalRead(ENABLE_PIN) == LOW);
            display_data.anti_dive_active = anti_dive_active;
            display_data.arc_ok = arc_voltage_ok;
            
            xSemaphoreGive(dataMutex);
        }
        lastDataUpdate = millis();
    }
    
    // Variables pour calibration (à activer/désactiver)
    static bool CALIBRATION_MODE = false;  // ✅ Mettre à false après calibration
    static int touch_count = 0;
    static int corners[4][2] = {{0,0}, {0,0}, {0,0}, {0,0}};
    
    unsigned long loopStartTime = micros();
    unsigned long currentTime = millis();
    
   
    if (!CALIBRATION_MODE) {
        
        // Exécution moteur
        if (use_accelstepper_run) {
            stepper.run(); 
        }
        
        // Enregistrement position
        if (currentTime - last_position_record_time >= POSITION_HISTORY_INTERVAL) {
            position_history[position_history_index].time = currentTime;
            position_history[position_history_index].position = stepper.currentPosition();
            position_history_index = (position_history_index + 1) % POSITION_HISTORY_SIZE;
            last_position_record_time = currentTime;
        }
        
        // Commandes serial
        if (Serial.available() > 0) {
            String command = Serial.readStringUntil('\n');
            if (command == "RESET_EEPROM") {
                EEPROM.put(EEPROM_SETPOINT_ADDR, DEFAULT_SETPOINT);
                EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
                EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP);
                EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
                EEPROM.put(EEPROM_KD_ADDR, DEFAULT_KD);
                EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, DEFAULT_STEP_PER_MM);
                byte flag = 0xAA;
                EEPROM.put(EEPROM_INITIALIZED_FLAG, flag);
                EEPROM.commit();
                
                float temp;
                EEPROM.get(EEPROM_SETPOINT_ADDR, temp);
                Setpoint = temp;
                EEPROM.get(EEPROM_CORRECTION_FACTOR_ADDR, temp);
                voltage_correction_factor = temp;
                EEPROM.get(EEPROM_KP_ADDR, temp);
                Kp = temp;
                EEPROM.get(EEPROM_KI_ADDR, temp);
                Ki = temp;
                EEPROM.get(EEPROM_KD_ADDR, temp);
                Kd = temp;
                myPID.SetTunings(Kp, Ki, Kd);
                EEPROM.get(EEPROM_STEPS_MM_Z_ADDR, STEPS_PER_MM_Z);
                Serial.println("EEPROM reset via serial command");
            }
        }
        
        // Lecture ADC (50ms)
        static unsigned long lastAdcTask = 0;
        if (millis() - lastAdcTask >= 50) {
            readAndFilterVoltage();
            lastAdcTask = millis();
        }

        // Contrôle THC (5ms)
        static unsigned long lastControlTask = 0;
        if (micros() - lastControlTask >= 5000) {
            managePlasmaAndTHC();
            lastControlTask = micros();
        }

        // Logs périodiques
        if (currentTime - lastLogTime >= LOG_INTERVAL) {
            bool enable_pin_low = (digitalRead(ENABLE_PIN));
            bool arc_ok = (fast_voltage > arc_threshold);
            
            Serial.print("ENABLE PIN: ");
            Serial.print(enable_pin_low ? "HIGH" : "LOW");
            Serial.print(" | Arc detected: ");
            Serial.print(arc_ok ? "Yes" : "No");
            Serial.print(" | THC OFF: ");
            Serial.print(thc_off ? "No" : "Yes");
            Serial.print(" | THC active: ");
            Serial.print(thc_active ? "ACTIVE " : "INACTIVE ");
            Serial.print(" | Fast voltage: ");
            Serial.print(fast_voltage);
            Serial.print(" V | Slow voltage: ");
            Serial.print(slow_voltage);
            Serial.print(" V | Anti-dive: ");
            Serial.print(anti_dive_active ? "Active" : "Inactive");
            Serial.print(" | Kp: ");
            Serial.println(Kp);
            
            if (!thc_active) {
                Serial.print("Reason THC inactive: ");
                if (thc_active) Serial.println("ENABLE_PIN LOW");
                else if (enable_pin_low) Serial.println("ENABLE_PIN HIGH");
                else if (!arc_ok) Serial.println("Arc not detected (low voltage)");
                else if (anti_dive_active) Serial.println("Anti-dive active");
                else if (thc_off) Serial.println("THC off");
                else Serial.println("Unknown condition");
            }
            
            lastLogTime = currentTime;
        }
        

        // Timing loop
        unsigned long loopEndTime = micros();
        unsigned long loopExecutionTime = loopEndTime - loopStartTime;
        loopExecutionTimeSum += loopExecutionTime;
        loopCount++;

        if (currentTime - lastLoopLogTime >= LOOP_LOG_INTERVAL) {
            if (loopCount > 0) {
                unsigned long averageLoopTime = loopExecutionTimeSum / loopCount;
                float loopFrequency = 1000000.0 / averageLoopTime;
                Serial.print("Average execution time: ");
                Serial.print(averageLoopTime);
                Serial.print(" us | Frequency: ");
                Serial.print(loopFrequency, 1);
                Serial.println(" Hz");
            }
            loopExecutionTimeSum = 0;
            loopCount = 0;
            lastLoopLogTime = currentTime;
        }
    }
}


void navigateScreen(int direction) {
    if(direction ==0){
      currentScreen=0;}
      else{currentScreen=currentScreen+direction;}
    if (currentScreen >= NB_SCREENS) {
        currentScreen = 0;
    }
    if (currentScreen < 0) {
        currentScreen = NB_SCREENS;
    }
    
}


void readAndFilterVoltage() {

      // ✅ AJOUT : Mode simulation bypass lecture ADC
    if (simulation_mode) {
        simulateSineWave();  // Ou autre fonction de simulation
        return;  // Skip lecture ADC réelle
    }
    // === WARM-UP ADC (1s) ===
    static unsigned long start_time = 0;
    static bool warmed_up = false;
    if (start_time == 0) start_time = millis();
    if (millis() - start_time >= 1000) warmed_up = true;

    // === LECTURE ADS1115  ===
    if (millis() - lastAdcRead >= 50) {
        // Protection I²C
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            adc_raw_value = ads.readADC_SingleEnded(ADS_PLASMA_CHANNEL);
            xSemaphoreGive(i2cMutex);
            lastAdcRead = millis();
        } else {
            Serial.println("I2C timeout!");
            return; // Skip ce cycle si timeout
        }
    }
    


    // --- MISE À L'ÉCHELLE CRITIQUE ADS1115 ---
    // 1. Tension mesurée après diviseur (V_divided)
    //    V_divided = (raw_value * ADS_GAIN_V) / ADS_RESOLUTION_MAX
    float V_divided = ((float)adc_raw_value * ADS_GAIN_V) / ADS_RESOLUTION_MAX;

    // 2. Tension plasma réelle (V_plasma)
    //    V_plasma = V_divided / PLASMA_VOLTAGE_DIVIDER_RATIO
    float raw = V_divided / PLASMA_VOLTAGE_DIVIDER_RATIO;
    
    // === FAST : Oversample 10x + Low-pass (PID input) ===
    oversample_sum += raw;
    oversample_count++;
    if (oversample_count >= OVERSAMPLE_TARGET) {
        float avg_raw = oversample_sum / OVERSAMPLE_TARGET;
        const float INPUT_ALPHA = 0.7f;
        uncorrected_fast = INPUT_ALPHA * avg_raw + (1.0f - INPUT_ALPHA) * last_pid_input;
        last_pid_input = uncorrected_fast;
        fast_voltage = uncorrected_fast * voltage_correction_factor;
        
        noInterrupts();  // Protection ESP32
        Input = fast_voltage;
        input_ready = true;  // Signal que Input est frais
        interrupts();
        // Reset cycle
        oversample_sum = 0.0f;
        oversample_count = 0;
    }

    // === SLOW : Moyenne 200 + Low-pass (anti-dive ref) ===
    const int N_SLOW = 200;
    static float slow_samples[N_SLOW];
    static int slow_idx = 0;
    static float slow_sum = 0.0f;
    static bool slow_init = false;
    static float slow_lp = 0.0f;
    const float ALPHA_SLOW = 0.8f;

    if (!slow_init) {
        for (int i = 0; i < N_SLOW; i++) slow_samples[i] = raw;
        slow_sum = raw * N_SLOW;
        slow_init = true;
    }
    slow_sum -= slow_samples[slow_idx];
    slow_samples[slow_idx] = raw;
    slow_sum += raw;
    slow_idx = (slow_idx + 1) % N_SLOW;

    float slow_raw_avg = slow_sum / N_SLOW;
    uncorrected_slow = slow_raw_avg;
    slow_voltage = ALPHA_SLOW * (slow_raw_avg * voltage_correction_factor) + 
                  (1.0f - ALPHA_SLOW) * slow_lp;
    slow_lp = slow_voltage;

    // === ANTI-DIVE : SÉCURISÉ → UNIQUEMENT si THC ACTIVE ! ===
    static float voltage_at_activation = 0.0f;
    const float DROP_THRESHOLD = 5.0f;
    const float RETURN_THRESHOLD = 3.0f;
    const unsigned long MAX_ANTI_DIVE_DURATION = 1000;
    static bool last_anti_dive_state = false;

    // Activation désactivation de l'Anti_Div
    if (warmed_up && 
        abs(fast_voltage - slow_voltage)> DROP_THRESHOLD && 
        !anti_dive_active && thc_active) {
        
        anti_dive_active = true;
        just_anti_dive_activated = true;
        anti_dive_start_time = millis();
        voltage_at_activation = slow_voltage;
        
        Serial.print("**Anti-dive ON (THC ACTIVE)** | Cut: ");
        Serial.print(fast_voltage, 1);
        Serial.print("V | Slow: ");
        Serial.print(slow_voltage, 1);
        Serial.print("V | Saved: ");
        Serial.println(voltage_at_activation, 1);
    } else if (anti_dive_active&&abs(Setpoint-slow_voltage) < RETURN_THRESHOLD) {
          anti_dive_active=false;
        }
  }
void managePlasmaAndTHC() {

  // ===== LECTURE DES ENTREES =====
  bool enable_active = (digitalRead(ENABLE_PIN) == LOW);      // THC ENABLE
  bool thc_off       = (digitalRead(THC_OFF_PIN) == HIGH);
  unsigned long currentTime = millis();

  static bool last_thc_active = false;

  if (thc_active && !last_thc_active) {
      z_reference = stepper.currentPosition();
  }
  last_thc_active = thc_active;

    // Détection arc plasma
  arc_voltage_ok = (fast_voltage > arc_threshold);
  
  // Determine THC state
    thc_active = enable_active &&
               arc_voltage_ok &&
               !anti_dive_active &&
               !thc_off;
  
    // --- 5. LOGIQUE DE COMMANDE DU MOTEUR Z ---

   if (anti_dive_active) {
        // --- MODE 1 : ANTI-DIVE (PRIORITÉ MAXIMALE - Mouvement de POSITION) ---

         if (pid_running) {
              myPID.SetMode(MANUAL);
              Output = 0.0;
              pid_running = false;
          }

          use_accelstepper_run = false; // Mode AccelStepper (moveTo)     
        
    } else if (thc_active) {
        if (!pid_running) {
            myPID.SetMode(AUTOMATIC);
            pid_running = true;
        }
        
        myPID.Compute();
        z_target = Output * STEPS_PER_MM_Z;
        // Configuration moteur
        stepper.setMaxSpeed(5000);
        stepper.setAcceleration(20000);
        
        if (labs(z_target - last_z_target) > 10) {
            stepper.moveTo(z_target);
            last_z_target = z_target;
          }
        use_accelstepper_run = true; // Mode AccelStepper (moveTo)
        
    } else { 
              
        // --- MODE 3 : PASSTHROUGH (CNC PREND LE CONTRÔLE - Contrôle DIRECT) ---
        // 1. Réinitialisation des variables de régulation
        if (pid_running) {
            myPID.SetMode(MANUAL);
            Output = 0.0;
            pid_running = false;
            Serial.println("PID stopped - Passthrough mode ");
        // 2. Désactiver AccelStepper pour éviter les conflits
        stepper.stop();      
        z_target = stepper.currentPosition();
        stepper.moveTo(z_target);
        use_accelstepper_run = false;
        z_target = 0;
        }
        
        
        // 2. Copie directe vers les broches de sortie (Passthrough)
        int stepState = digitalRead(CNC_Z_STEP_IN); 
        int dirState = digitalRead(CNC_Z_DIR_IN); 
        digitalWrite(STEPPER_STEP_PIN, stepState); 
        digitalWrite(STEPPER_DIR_PIN, dirState);
        

    }
} 

// void updateTFT() {
//     // La logique d'effacement de l'écran ou de la zone de mise à jour va ici.
  
// if(currentScreen+1!=1){// Les bouton + et - ne sont pas disponible sur la page principale
//     // 1. Bouton AUGMENTER (UP - Ajustement Setpoint)
//     tft.fillRect(UP_BUTTON_X_MIN, UP_BUTTON_Y_MIN, 
//                  UP_BUTTON_X_MAX - UP_BUTTON_X_MIN, 
//                  UP_BUTTON_Y_MAX - UP_BUTTON_Y_MIN, 
//                  TFT_DARKGREEN); 
    
//     tft.setTextSize(3);
//     tft.setTextColor(TFT_WHITE);
//     // Dessin du symbole 'plus' ou d'une flèche vers le haut
//     tft.setCursor(UP_BUTTON_X_MIN + 60, UP_BUTTON_Y_MIN + 20);
//     tft.println("+"); // Ou un caractère flèche '▲'
    
//     // 2. Bouton DIMINUER (DOWN - Ajustement Setpoint)
//     tft.fillRect(DOWN_BUTTON_X_MIN, DOWN_BUTTON_Y_MIN, 
//                  DOWN_BUTTON_X_MAX - DOWN_BUTTON_X_MIN, 
//                  DOWN_BUTTON_Y_MAX - DOWN_BUTTON_Y_MIN, 
//                  TFT_DARKRED); 
                 
//     tft.setTextColor(TFT_WHITE);
//     // Dessin du symbole 'moins'
//     tft.setCursor(DOWN_BUTTON_X_MIN + 60, DOWN_BUTTON_Y_MIN + 20);
//     tft.println("-"); // Ou un caractère flèche '▼'
//     tft.setTextSize(1);
//     }
    
//         //---Bouton page suivante
//     int MENU_LARGUEUR=MENU_BUTTON_X_MAX-MENU_BUTTON_X_MIN;
//     int MENU_HAUTEUR=MENU_BUTTON_Y_MAX-MENU_BUTTON_Y_MIN;           
//     tft.fillRect(MENU_BUTTON_X_MIN, MENU_BUTTON_Y_MIN,MENU_LARGUEUR ,MENU_HAUTEUR, TFT_DARKGREY);
//     tft.setTextFont(2);
//     tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
//     tft.drawCentreString("Suivant", 40, 290, 2);

//             //---Bouton page précédent
//     int PRECED_LARGUEUR=PRECED_BUTTON_X_MAX-PRECED_BUTTON_X_MIN;
//     int PRECED_HAUTEUR=PRECED_BUTTON_Y_MAX-PRECED_BUTTON_Y_MIN;           
//     tft.fillRect(PRECED_BUTTON_X_MIN, PRECED_BUTTON_Y_MIN,PRECED_LARGUEUR ,PRECED_HAUTEUR, TFT_DARKGREY);
//     tft.setTextFont(2);
//     tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
//     tft.drawCentreString("Precedent", 80, 290, 2);

//            //---Bouton HOME
//     int HOME_LARGUEUR=HOME_BUTTON_X_MAX-HOME_BUTTON_X_MIN;
//     int HOME_HAUTEUR=HOME_BUTTON_Y_MAX-HOME_BUTTON_Y_MIN;           
//     tft.fillRect(HOME_BUTTON_X_MIN, HOME_BUTTON_Y_MIN,HOME_LARGUEUR ,HOME_HAUTEUR, TFT_DARKGREY);
//     tft.setTextFont(2);
//     tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
//     tft.drawCentreString("Home", 230 , 290 , 2);
    
// }

// void updateDisplay() {
//     static int last_currentScreen = -1;
//     if (currentScreen != last_currentScreen) {
//       EEPROM.commit();
//         tft.fillScreen(TFT_BLACK);
//         last_currentScreen = currentScreen;
//     tft.setTextFont(2);
//     // --- HEADER (largeur 480) ---
//     tft.fillRect(0, 0, 480, 28, TFT_DARKGREY);
//     tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
//     tft.setCursor(6, 6);
//     tft.printf("THC | Screen %d/%d", currentScreen + 1, NB_SCREENS);
//     updateTFT();
//     }
//     tft.setTextColor(TFT_WHITE, TFT_BLACK);
//     tft.setTextFont(2);
//     tft.setTextSize(1);
//     tft.setTextColor(TFT_WHITE, TFT_BLACK);
//     switch (currentScreen) {

//         case 0: // ---- SCREEN 1 ----
            
//             tft.drawString("STATUS : MONITORING", 10, 40, 2);

//             tft.drawString("Voltage:", 10, 70, 2);
//             tft.setTextColor(TFT_CYAN, TFT_BLACK);
//             tft.drawFloat(fast_voltage, 1, 220, 70, 2);
//             tft.drawFloat(slow_voltage, 1, 250, 70, 2);

//             tft.setTextColor(TFT_WHITE, TFT_BLACK);
//             tft.drawString("Setpoint:", 10, 90, 2);
//             tft.setTextColor(TFT_GREEN, TFT_BLACK);
//             tft.drawFloat(Setpoint, 1, 220, 90, 2);

//             tft.setTextColor(TFT_WHITE, TFT_BLACK);
//             tft.drawString("position :", 10, 110, 2);
//             tft.setTextColor(TFT_GREEN, TFT_BLACK);
//             tft.drawFloat(stepper.currentPosition(), 1, 220, 110, 2);

//             tft.setTextColor(TFT_WHITE, TFT_BLACK);
//             tft.drawString("THC State:", 10, 130, 2);
//             tft.setTextColor(thc_active ? TFT_GREEN : TFT_RED, TFT_BLACK);
//             tft.drawString(thc_active ? "ACTIF  ":"INACTIF", 220, 130, 2);

//             tft.setTextColor(TFT_WHITE, TFT_BLACK);
//             tft.drawString("Enable:", 10, 150, 2);
//             tft.setTextColor(digitalRead(ENABLE_PIN) ? TFT_RED : TFT_GREEN, TFT_BLACK);
//             tft.drawString(digitalRead(ENABLE_PIN)? "INACTIF":"ACTIF    ", 220, 150, 2);

//             tft.setTextColor(TFT_WHITE, TFT_BLACK);
//             tft.drawString("ANTI DIV:", 10, 170, 2);
//             tft.setTextColor(digitalRead(anti_dive_active) ? TFT_GREEN : TFT_RED, TFT_BLACK);
//             tft.drawString(digitalRead(anti_dive_active) ? "INACTIF":"ACTIF  ", 220, 170, 2);

//             tft.setTextColor(TFT_WHITE, TFT_BLACK);
//             tft.drawString("Tension:", 10, 190, 2);
//             tft.setTextColor(arc_voltage_ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
//             tft.drawString(arc_voltage_ok ? "OK ":"NOK", 220, 190, 2);
//             break;

//         case 1: // ---- SCREEN 2 ----
               
//             tft.drawString("ADJ. Consigne", 10, 40, 4);
//             tft.drawString("Consigne (V):", 10, 100, 4);
//             tft.setTextColor(TFT_CYAN, TFT_BLACK);
//             tft.drawFloat(Setpoint, 1, 260, 100, 4);
//             break;

//         case 2: // ---- SCREEN 2 ----
//             tft.drawString("ADJ. CORRECTION FACTOR", 10, 40, 4);
//             tft.drawString("Current Factor:", 10, 100, 4);
//             tft.setTextColor(TFT_YELLOW, TFT_BLACK);
//             tft.drawFloat(temp_voltage_correction_factor, 2, 260, 100, 4);

//             tft.setTextColor(TFT_RED, TFT_BLACK);
//             tft.drawString("Saves on exit!", 10, 170, 2);
//             break;

//         case 3:
//             tft.drawString("ADJ. PID Kp", 10, 40, 4);
//             tft.drawString("Kp:", 10, 100, 4);
//             tft.setTextColor(TFT_RED, TFT_BLACK);
//             tft.drawFloat(Kp, 1, 260, 100, 4);
//             break;

//         case 4:
//             tft.drawString("ADJ. PID Ki", 10, 40, 4);
//             tft.drawString("Ki:", 10, 100, 4);
//             tft.setTextColor(TFT_RED, TFT_BLACK);
//             tft.drawFloat(Ki, 1, 260, 100, 4);
//             break;        
            
//         case 5:
//             tft.drawString("Adj Steps par mm Z", 10, 40, 4);
//             tft.drawString("Steps/mm:", 10, 100, 4);
//             tft.setTextColor(TFT_RED, TFT_BLACK);
//             tft.drawFloat(STEPS_PER_MM_Z, 1, 260, 100, 4);
//             break;


//     }
// }

// void handleTouchInput(uint16_t x, uint16_t y) {
//     // Bouton UP
//     if (x >= UP_BUTTON_X_MIN && x <= UP_BUTTON_X_MAX && 
//         y >= UP_BUTTON_Y_MIN && y <= UP_BUTTON_Y_MAX) {
//         adjustCurrentSetting(1);
//         flashButton(UP_BUTTON_X_MIN, UP_BUTTON_Y_MIN, 
//                    UP_BUTTON_X_MAX, UP_BUTTON_Y_MAX, TFT_GREEN);
//         return;
//     }

//     // Bouton DOWN
//     if (x >= DOWN_BUTTON_X_MIN && x <= DOWN_BUTTON_X_MAX && 
//         y >= DOWN_BUTTON_Y_MIN && y <= DOWN_BUTTON_Y_MAX) {
//         adjustCurrentSetting(-1);
//         flashButton(DOWN_BUTTON_X_MIN, DOWN_BUTTON_Y_MIN, 
//                    DOWN_BUTTON_X_MAX, DOWN_BUTTON_Y_MAX, TFT_RED);
//         return;
//     }

//     // Bouton MENU (header)
//     if (x >= MENU_BUTTON_X_MIN && x <= MENU_BUTTON_X_MAX && 
//         y >= MENU_BUTTON_Y_MIN && y <= MENU_BUTTON_Y_MAX) {
//         navigateScreen(1);
//         flashButton(MENU_BUTTON_X_MIN, MENU_BUTTON_Y_MIN, 
//                    MENU_BUTTON_X_MAX, MENU_BUTTON_Y_MAX, TFT_BLUE);
//         return;
//     }

//         // Bouton Précédent
//     if (x >= PRECED_BUTTON_X_MIN && x <= PRECED_BUTTON_X_MAX && 
//         y >= PRECED_BUTTON_Y_MIN && y <= PRECED_BUTTON_Y_MAX) {
//         navigateScreen(-1);
//         flashButton(PRECED_BUTTON_X_MIN, PRECED_BUTTON_Y_MIN, 
//                    PRECED_BUTTON_X_MAX, PRECED_BUTTON_Y_MAX, TFT_BLUE);

//         return;
//     }
//                 // Bouton Home
//     if (x >= HOME_BUTTON_X_MIN && x <= HOME_BUTTON_X_MAX && 
//         y >= HOME_BUTTON_Y_MIN && y <= HOME_BUTTON_Y_MAX) {
//         navigateScreen(0);
//         flashButton(HOME_BUTTON_X_MIN, HOME_BUTTON_Y_MIN, 
//                    HOME_BUTTON_X_MAX, HOME_BUTTON_Y_MAX, TFT_BLUE);

//         return;
//     }
// }