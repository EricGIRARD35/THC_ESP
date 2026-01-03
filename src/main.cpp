
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
#include "ui_screens.h"
#include "screen_graph.h"



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

DisplayData display_data = {0}; 

// Pleine échelle de l'ADS1115 (FSR - Full Scale Range) en volts.
// Nous allons configurer l'ADC pour utiliser un gain de 1 (GAIN_1), ce qui donne ±4.096V.
// ASSUREZ-VOUS que la tension maximale après le diviseur est inférieure à 4.096V.
const float ADS_GAIN_V = 4.096; 

// Résolution de l'ADS1115 (2^15 pour les lectures à simple extrémité signées, d'où 32767)
const float ADS_RESOLUTION_MAX = 32767.0; 

// Rapport de votre diviseur de tension (Exemple : si 40:1, alors 1/40 = 0.025)
// C'est le coefficient K tel que V_plasma = V_mesurée / K
float PLASMA_VOLTAGE_DIVIDER_RATIO = 76.4; // Vérifiez votre diviseur réel !

// Variables pour la lecture tactile non-bloquante
unsigned long lastTouchTime = 0;
const unsigned long touchInterval = 50; // Lire le tactile toutes les 50 ms

// --- Prototypes des fonctions d'interface tactile ---
void handleTouchInput(uint16_t x, uint16_t y);
// void adjustCurrentSetting(int direction);
void flashButton(int x1, int y1, int x2, int y2, uint16_t color);
void  lv_scr_load();



// Parametres par défaut
const float DEFAULT_SETPOINT = 110.0; //Attention valeur de DEFAULT stocké sur 4 octets mais utilisé en double pour le PID
static float slow_lp = 0.0f;
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
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); // reverse pour que l'action du PID soit inverse de l'erreur
long z_target = 0;
 long last_z_target = 0;
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
bool anti_dive_active = false;         // Anti-dive state
unsigned long anti_dive_start_time = 0;// Anti-dive start time
bool enable_was_active = false;
volatile bool enable_active_g = false;

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
 unsigned long last_eeprom_write = 0;
const unsigned long EEPROM_WRITE_INTERVAL = 1000;

// Temporary variable for voltage correction factor adjustment
// float temp_voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;

// === OVERSAMPLING NON-BLOQUANT pour PID ultra-stable ===
#define OVERSAMPLE_TARGET 10  // 10 samples ~10ms @1kHz
 float oversample_sum = 0.0;
 uint8_t oversample_count = 0;
 float last_pid_input = 0.0;  // Dernière moyenne pour low-pass
const float INPUT_ALPHA = 0.7;      // Low-pass fort sur moyenne

// Function declarations
void initializeEEPROM();
void calculateSpeed();
void readAndFilterVoltage();
void managePlasmaAndTHC();
void countStepX();
void countStepY();
// void updateDisplay();

// === ANTI-DIVE : SÉCURISÉ → UNIQUEMENT si THC ACTIVE ! ===
float voltage_at_activation = 0.0f;
float DROP_THRESHOLD = 5.0f;
float RETURN_THRESHOLD = 3.0f;
const unsigned long MAX_ANTI_DIVE_DURATION = 1000;
bool last_anti_dive_state = false;

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


// Utilisation de IRAM_ATTR pour une exécution en nanosecondes
void IRAM_ATTR handleCNCStep() {
  // On ne laisse passer les pas de la CNC QUE si le THC n'est pas en train de piloter
  if (!enable_active_g) {
    digitalWrite(STEPPER_STEP_PIN, digitalRead(CNC_Z_STEP_IN));
  }
}

void IRAM_ATTR handleCNCDir() {
  if (!enable_active_g) {
    digitalWrite(STEPPER_DIR_PIN, digitalRead(CNC_Z_DIR_IN));
  }
}


void taskLvglTick(void *pvParameters) {
    for (;;) {
        lv_tick_inc(1);                 // ⬅️ 1 ms
        vTaskDelay(pdMS_TO_TICKS(1));   // ⬅️ tick à 1 kHz
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
  attachInterrupt(digitalPinToInterrupt(CNC_Z_STEP_IN), handleCNCStep, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CNC_Z_DIR_IN), handleCNCDir, CHANGE);

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
    //tft.invertDisplay(true);
    tft.setRotation(1);     // Rotation 1 (paysage) pour un ESP32/TFT typique
    tft.fillScreen(TFT_BLACK);
  
    // ✅ INIT XPT2046 APRÈS TFT
    if (!ts.begin()) {
        Serial.println("❌ XPT2046 échec!");
    }
    ts.setRotation(1);

    // Initialisation LVGL (APRÈS TFT)
    lvgl_setup();

  for (int i = 0; i < speed_filter_size; i++) {
    speed_readings[i] = 0.0;
  }

  myPID.SetOutputLimits(-100, 100); 
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetSampleTime(1); 
  

  // Validation des valeurs (inchangé)
  if (isnan(Setpoint) || Setpoint < 80 || Setpoint > 200) Setpoint = DEFAULT_SETPOINT;
  if (isnan(STEPS_PER_MM_Z) || STEPS_PER_MM_Z < 100 || STEPS_PER_MM_Z > 1000) STEPS_PER_MM_Z = DEFAULT_STEP_PER_MM;
  if (isnan(Kp) || Kp < 0.0 || Kp > 10) Kp = DEFAULT_KP;
  if (isnan(Ki) || Ki < 0.0 || Ki > 10) Ki = DEFAULT_KI;
  if (isnan(Kd) || Kd < 0.0 || Kd > 0.1) Kd = DEFAULT_KD;
  myPID.SetTunings(Kp, Ki, Kd);
  
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
        //EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
        EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, DEFAULT_STEP_PER_MM);
        EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP); // Écrit la valeur non nulle (p. ex. 100.0)
        EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
        EEPROM.put(EEPROM_DIVISEUR_VOLTAGE_ADDR, PLASMA_VOLTAGE_DIVIDER_RATIO);

// Stockage en uint16_t (2 octets) pour les seuils
        uint16_t drop_val = (uint16_t)(DROP_THRESHOLD * 10);
        uint16_t ret_val  = (uint16_t)(RETURN_THRESHOLD * 10);
        
        EEPROM.put(EEPROM_DROP_THRESHOLD_ADDR, drop_val); // uint16 (2 oct)
        EEPROM.put(EEPROM_RETURN_THRESHOLD_ADDR, ret_val); // uint16 (2 oct)      

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

    EEPROM.get(EEPROM_KP_ADDR, temp);
    Kp = temp;
    Serial.print("Loaded Kp: "); Serial.println(Kp, 2);

    EEPROM.get(EEPROM_KI_ADDR, temp);
    Ki = temp;
    Serial.print("Loaded Ki: "); Serial.println(Ki, 4);

    EEPROM.get(EEPROM_DIVISEUR_VOLTAGE_ADDR, temp);
    PLASMA_VOLTAGE_DIVIDER_RATIO = temp;
    Serial.print("Loaded PLASMA_VOLTAGE_DIVIDER_RATIO: "); Serial.println(PLASMA_VOLTAGE_DIVIDER_RATIO, 4);

    EEPROM.get(EEPROM_STEPS_MM_Z_ADDR, STEPS_PER_MM_Z);
    Serial.print("Loaded steps par mm: "); Serial.println(STEPS_PER_MM_Z, 4);

    // Charger les seuils de drop/return
    uint16_t drop_val = 0;              
    uint16_t ret_val = 0;
    EEPROM.get(EEPROM_DROP_THRESHOLD_ADDR, drop_val);
    EEPROM.get(EEPROM_RETURN_THRESHOLD_ADDR, ret_val);
    DROP_THRESHOLD = (float)drop_val / 10.0;
    RETURN_THRESHOLD = (float)ret_val / 10.0;
    Serial.print("Loaded DROP_THRESHOLD: "); Serial.println(DROP_THRESHOLD, 1);
    Serial.print("Loaded RETURN_THRESHOLD: "); Serial.println(RETURN_THRESHOLD, 1);
}

 void loop() {
    // ========================================
    // VARIABLES STATIQUES (déclarées UNE SEULE FOIS)
    // ========================================
     unsigned long lastTouchCheck = 0;
     bool was_touched = false;
     int last_x = 0, last_y = 0;

     unsigned long lastDataUpdate = 0;
    if (millis() - lastDataUpdate >= 100) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            display_data.fast_voltage = fast_voltage;
            display_data.slow_voltage = slow_voltage;
            display_data.setpoint = Setpoint;
            display_data.output_pid = Output;
            display_data.position = stepper.currentPosition();
            display_data.thc_active = thc_active;
            display_data.enable_active = (digitalRead(ENABLE_PIN) == LOW);
            display_data.anti_dive_active = anti_dive_active;
            display_data.arc_ok = arc_voltage_ok;
            
            xSemaphoreGive(dataMutex);
        }
        lastDataUpdate = millis();
    }

// ✅ AJOUTEZ : Mise à jour graphique (200ms)
    static unsigned long lastGraphUpdate = 0;
    if (currentScreen == 1 && millis() - lastGraphUpdate >= 200) {  // Screen 1 = graphique
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            update_graph_data(
                display_data.fast_voltage,
                display_data.setpoint,
                display_data.output_pid
            );
            xSemaphoreGive(dataMutex);
        }
        lastGraphUpdate = millis();
    }
    
    // ✅ AJOUTEZ : Tâche graphique (échelle auto)
    if (currentScreen == 1) {
        graph_update_task();
    }

     int touch_count = 0;
     int corners[4][2] = {{0,0}, {0,0}, {0,0}, {0,0}};
    
    unsigned long loopStartTime = micros();
    unsigned long currentTime = millis();
    

        
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
                //EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
                EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP);
                EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
                EEPROM.put(EEPROM_DIVISEUR_VOLTAGE_ADDR, PLASMA_VOLTAGE_DIVIDER_RATIO);
                EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, DEFAULT_STEP_PER_MM);
                byte flag = 0xAA;
                EEPROM.put(EEPROM_INITIALIZED_FLAG, flag);
                EEPROM.commit();
                
                float temp;
                EEPROM.get(EEPROM_SETPOINT_ADDR, temp);
                Setpoint = temp;
                EEPROM.get(EEPROM_KP_ADDR, temp);
                Kp = temp;
                EEPROM.get(EEPROM_KI_ADDR, temp);
                Ki = temp;
                EEPROM.get(EEPROM_DIVISEUR_VOLTAGE_ADDR, temp);
                PLASMA_VOLTAGE_DIVIDER_RATIO = temp;
                myPID.SetTunings(Kp, Ki, Kd);
                EEPROM.get(EEPROM_STEPS_MM_Z_ADDR, STEPS_PER_MM_Z);
                Serial.println("EEPROM reset via serial command");
            }
        }
        
        // Lecture ADC (50ms)
         unsigned long lastAdcTask = 0;
        if (millis() - lastAdcTask >= 50) {
            readAndFilterVoltage();
            lastAdcTask = millis();
        }

        // Contrôle THC (5ms)
         unsigned long lastControlTask = 0;
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
    //    V_plasma = V_divided * PLASMA_VOLTAGE_DIVIDER_RATIO
    float raw = V_divided * PLASMA_VOLTAGE_DIVIDER_RATIO;
    
    // === FAST : Echantillonnage 10x + Low-pass (PID input) ===
    oversample_sum += raw;
    oversample_count++;
    if (oversample_count >= OVERSAMPLE_TARGET) {
        float avg_raw = oversample_sum / OVERSAMPLE_TARGET;
        const float INPUT_ALPHA = 0.7f;
        fast_voltage = INPUT_ALPHA * avg_raw + (1.0f - INPUT_ALPHA) * last_pid_input;
        last_pid_input = fast_voltage;
            
        // === SLOW : Moyenne 200 + Low-pass (référence anti-dive) ===
        const int N_SLOW = 200;
        static float slow_samples[N_SLOW];
        static int slow_idx = 0;
        static float slow_sum = 0.0f;
        static bool slow_init = false;
        
        const float ALPHA_SLOW = 0.0005f;
        //Initialisation de fast voltage au front montant de Enable        
bool enable_now = (digitalRead(ENABLE_PIN) == LOW);

if (enable_now && !enable_was_active) {
    // Front montant d'Enable : l'arc est déjà stabilisé côté G-code, on capture la référence ici
    for (int i = 0; i < N_SLOW; i++) slow_samples[i] = fast_voltage;
    slow_sum = fast_voltage * N_SLOW;
    slow_lp = fast_voltage;
    slow_idx = 0;
    slow_init = true;
}
enable_was_active = enable_now;
if (!enable_now && anti_dive_active) {
    anti_dive_active = false;   // Sécurité : Enable retombé, on relâche l'axe
}
        slow_sum -= slow_samples[slow_idx]; // supresssion de la plus ancienne valeur dans la somme
        slow_samples[slow_idx] = avg_raw; // ajout de la nouvelle valeur dans le tableau
        slow_sum += avg_raw; // Ajout de la nouvelle valeur dans la somme
        slow_idx = (slow_idx + 1) % N_SLOW; // décalage de l'index circulaire

        float slow_raw_avg = slow_sum / N_SLOW; // calcul de la moyenne glissante
        // Application d'un filtre low-pass sur la moyenne
        slow_voltage = ALPHA_SLOW * slow_raw_avg  + 
                    (1.0f - ALPHA_SLOW) * slow_lp;
        slow_lp = slow_voltage;

        noInterrupts();  // Protection ESP32
        Input = fast_voltage;
        input_ready = true;  // Signal que Input est frais
        interrupts();
        // Reset cycle
        oversample_sum = 0.0f;
        oversample_count = 0;
    }

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
    } else if (anti_dive_active&&abs(fast_voltage-slow_voltage) < RETURN_THRESHOLD) {
          anti_dive_active=false;
        }
  }
void managePlasmaAndTHC() {

  // ===== LECTURE DES ENTREES =====
  bool enable_active = (digitalRead(ENABLE_PIN) == LOW);      // THC ENABLE
  enable_active_g = enable_active; 
    // Resynchronisation immédiate et systématique de la direction dès qu'Enable
  // est inactif, qu'on soit encore en anti-dive ou déjà en passthrough pur.
  // Évite qu'un pas de retrait CNC parte avec une direction périmée héritée du PID.
  if (!enable_active) {
      digitalWrite(STEPPER_DIR_PIN, digitalRead(CNC_Z_DIR_IN));
  }

  bool thc_off       = (digitalRead(THC_OFF_PIN) == HIGH);
  unsigned long currentTime = millis();

    // Détection arc plasma
  arc_voltage_ok = (fast_voltage > arc_threshold);
  
  // Determine THC state
    thc_active = enable_active &&
               arc_voltage_ok;// &&
               //!anti_dive_active &&
               //!thc_off
  
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

        }
        stepper.stop();      
        z_target = stepper.currentPosition();
        stepper.moveTo(z_target);
        use_accelstepper_run = false;
        z_target = 0;
        // Resynchronisation forcée de la direction : ne pas attendre un futur
        // changement d'état de CNC_Z_DIR_IN, qui pourrait ne jamais arriver
        // si la direction n'a pas besoin de changer pour le prochain mouvement CNC.
        digitalWrite(STEPPER_DIR_PIN, digitalRead(CNC_Z_DIR_IN));
    }
}