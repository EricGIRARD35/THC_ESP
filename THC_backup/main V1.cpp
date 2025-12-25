#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <limits.h>
#include <TFT_eSPI.h>
#include <Adafruit_ADS1X15.h>


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

Adafruit_ADS1115 ads;
// Si le pin ADDR est connecté à GND -> 0x48
#define ADS1115_ADDR_GND 0x48 
// Canal de l'ADS1115 utilisé pour lire le voltage plasma (ex: AIN0)
#define ADS_PLASMA_CHANNEL 1

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
void adjustCurrentSetting(int direction);
void flashButton(int x1, int y1, int x2, int y2, uint16_t color);
void navigateScreen();

// // Variable statique pour limiter la fréquence de lecture tactile
// static unsigned long lastTouchTime = 0;
// const unsigned long touchInterval = 20; // 20 ms intervalle = 50 Hz

#define UP_BUTTON_X_MIN 350
#define UP_BUTTON_X_MAX 480
#define UP_BUTTON_Y_MIN 90
#define UP_BUTTON_Y_MAX 160

#define DOWN_BUTTON_X_MIN 350
#define DOWN_BUTTON_X_MAX 480
#define DOWN_BUTTON_Y_MIN 190
#define DOWN_BUTTON_Y_MAX 250

#define MENU_BUTTON_X_MIN 330
#define MENU_BUTTON_X_MAX 480
#define MENU_BUTTON_Y_MIN 280
#define MENU_BUTTON_Y_MAX 350

#define PRECED_BUTTON_X_MIN 0
#define PRECED_BUTTON_X_MAX 150
#define PRECED_BUTTON_Y_MIN 280
#define PRECED_BUTTON_Y_MAX 350

#define HOME_BUTTON_X_MIN 165
#define HOME_BUTTON_X_MAX 315
#define HOME_BUTTON_Y_MIN 280
#define HOME_BUTTON_Y_MAX 350

// --- Constantes de Zone Tactile --
#define TFT_DARKRED 0x4000 // Une valeur hexadécimale pour un rouge sombre

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
const unsigned long LOG_INTERVAL = 1000;
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
const unsigned long ANTI_DIVE_DURATION_MIN = 50; // Min duration (ms) at high speed
const unsigned long ANTI_DIVE_DURATION_MAX = 300; // Max duration (ms) at low speed

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
void updateDisplay();

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

void setup() {

  // --- 1. Initialisation EEPROM (Spécifique ESP32) ---
  // On réserve 512 octets de mémoire flash pour émuler l'EEPROM
  Serial.begin(115200);
  
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialise EEPROM");
    delay(1000);
    ESP.restart();
  }

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

    // Initialisation I²C de l'ADS1115
  Serial.println("Initialisation de l'ADS1115 (ADC Externe)...");
  ads.begin();

  // ➡️ AJOUT TFT : Initialisation de l'écran
    tft.init();
    tft.setRotation(1);     // Rotation 1 (paysage) pour un ESP32/TFT typique
    tft.fillScreen(TFT_BLACK);
    uint16_t calData[5] = { 303, 3529, 281, 3450, 7 };
    tft.setTouch(calData);

  for (int i = 0; i < speed_filter_size; i++) {
    speed_readings[i] = 0.0;
  }

  myPID.SetOutputLimits(-100, 100); 
  myPID.SetMode(AUTOMATIC);
  Ki = DEFAULT_KI;
  Kd = DEFAULT_KD;
  Kp = DEFAULT_KP;
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetSampleTime(1); 
  // Initialize EEPROM with default values if not already initialized
  initializeEEPROM();

  // Validation des valeurs (inchangé)
  if (isnan(Setpoint) || Setpoint < 80 || Setpoint > 200) Setpoint = DEFAULT_SETPOINT;
  if (isnan(voltage_correction_factor) || voltage_correction_factor < 0.5 || voltage_correction_factor > 2.0) voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;
  if (isnan(STEPS_PER_MM_Z) || STEPS_PER_MM_Z < 200 || STEPS_PER_MM_Z > 2000) STEPS_PER_MM_Z = DEFAULT_STEP_PER_MM;
  if (isnan(Kp) || Kp < 0.0 || Kp > 10) Kp = DEFAULT_KP;
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
  loopStartTime = micros();
  unsigned long currentTime = millis();

  // Exécution Conditionnelle du Moteur (CRITIQUE)
  if (use_accelstepper_run) {
  stepper.run(); 
  }

  //  Enregistrement de la position
  if (currentTime - last_position_record_time >= POSITION_HISTORY_INTERVAL) {
    position_history[position_history_index].time = currentTime;
    position_history[position_history_index].position = stepper.currentPosition();
    position_history_index = (position_history_index + 1) % POSITION_HISTORY_SIZE;
    last_position_record_time = currentTime;
  }

  if (millis() - lastTouchTime >= touchInterval) { // Lecture et gestion du Touch
    uint16_t touchX, touchY;
    
    // Vérifie si l'écran est touché et récupère les coordonnées
    // La méthode exacte dépend de votre librairie TFT, ici exemple avec TFT_eSPI:
    if (tft.getTouch(&touchX, &touchY)) {
        
        // La fonction qui gère l'action basée sur les coordonnées
        handleTouchInput(touchX, touchY);
        lastTouchTime = millis();
        Serial.print("ecran touché");
        Serial.print(touchX,1);
        Serial.print(touchY,1);
    }
    
  }

  // Check serial commands for EEPROM reset
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "RESET_EEPROM") {
      EEPROM.put(EEPROM_SETPOINT_ADDR, DEFAULT_SETPOINT);
      EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
      EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP);
      EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
      EEPROM.put(EEPROM_KD_ADDR, DEFAULT_KD);
      EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, DEFAULT_STEP_PER_MM);
      initializedFlag = 0xAA;
      EEPROM.put(EEPROM_INITIALIZED_FLAG, initializedFlag);

      EEPROM.commit(); // <--- AJOUT CRITIQUE ESP32 : Valider le reset
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

  bool enable_pin_low = (digitalRead(ENABLE_PIN));
  bool arc_voltage_ok = (fast_voltage > arc_threshold);

  if (currentTime - lastLogTime >= LOG_INTERVAL) {
      Serial.print("ENABLE PIN: ");
      Serial.print(enable_pin_low ? "HIGH" : "LOW");
      Serial.print(" | Arc detected: ");
      Serial.print(arc_voltage_ok ? "Yes" : "No");
      Serial.print(" | THC OFF: ");
      Serial.print(thc_off ? "No" : "Yes");
      Serial.print(" | THC active: ");
      Serial.print(thc_active ? "ACTIVE " : "INACTIVE ");
      Serial.print(" | Fast voltage: ");
      Serial.print(fast_voltage);
      Serial.print(" V | Slow voltage: ");
      Serial.print(slow_voltage);
      Serial.print(" V | Anti-dive: ");
      Serial.println(anti_dive_active ? "Active" : "Inactive");
 
      // Explicit log if THC inactive
      if (!thc_active) {
        Serial.print("Reason THC inactive: ");
        if (thc_active) Serial.println("ENABLE_PIN LOW");
        else if (enable_pin_low) Serial.println("ENABLE_PIN HIGH");
        else if (!arc_voltage_ok) Serial.println("Arc not detected (low voltage)");
        else if (anti_dive_active) Serial.println("Anti-dive active");
        else if (thc_off) Serial.println("THC off");
        else Serial.println("Unknown condition");
      }
    
    lastLogTime = currentTime;
  }
  
  if (currentTime - last_display_time >= display_interval) {
    updateDisplay();
    last_display_time = currentTime;
  }

  static unsigned long lastPidTime = 0;
  unsigned long currentMicros = micros();
  if (currentMicros - lastPidTime >= 1000) { // 1kHz
    readAndFilterVoltage();
    managePlasmaAndTHC();
    lastPidTime = currentMicros;
  }
// Exécution Conditionnelle du Moteur (CRITIQUE)

  loopEndTime = micros();
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

void adjustCurrentSetting(int direction) {
    float increment = 0.0;
    
    switch(currentScreen) {
        case 0: // Monitoring - read only
            Serial.println("Monitoring screen - no adjustment");
            break;
            
        case 1: // Setpoint
            increment = 1.0;
            Setpoint += direction * increment;
            Setpoint = constrain(Setpoint, 80.0, 200.0);
            EEPROM.put(EEPROM_SETPOINT_ADDR, (float)Setpoint);
            
            break;
            
        case 2: // Voltage correction factor
            increment = 0.01;
            temp_voltage_correction_factor += direction * increment;
            temp_voltage_correction_factor = constrain(temp_voltage_correction_factor, 0.5, 2.0);
            break;
          
        case 3: // Kp
            increment = 0.5;
            Kp += direction * increment;
            Kp = constrain(Kp, 0.0, 10.0);
            myPID.SetTunings(Kp, Ki, Kd);
            EEPROM.put(EEPROM_KP_ADDR,(float)Kp);
            
            break;
            
        case 4: // Ki
            increment = 0.1;
            Ki += direction * increment;
            Ki = constrain(Ki, 0.0, 10.0);
            myPID.SetTunings(Kp, Ki, Kd);
            EEPROM.put(EEPROM_KI_ADDR, (float)Ki);
            
            break;

            case 5: // Steps pas mm
            increment = 1;
            STEPS_PER_MM_Z += direction * increment;
            STEPS_PER_MM_Z = constrain(STEPS_PER_MM_Z, 200.0, 2000.0);
            EEPROM.put(EEPROM_STEPS_MM_Z_ADDR, (float)STEPS_PER_MM_Z);
            break;
    }
    updateDisplay();
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
    updateDisplay();
}
void flashButton(int x1, int y1, int x2, int y2, uint16_t color) {
    // tft.fillRect(x1, y1, x2 - x1, y2 - y1, color);
    // delay(50); 
    // // Idéalement, redessiner le bouton d'origine ici ou forcer un updateTFT()
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

    // === 1x LECTURE ADS1115 (16 bits) ===
    // Remplace analogRead(PLASMA_VOLTAGE)
    int16_t adc_raw_value = ads.readADC_SingleEnded(ADS_PLASMA_CHANNEL); 

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
        fast_voltage > slow_voltage + DROP_THRESHOLD && 
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
    } else if (anti_dive_active&&(fast_voltage-slow_voltage) < RETURN_THRESHOLD) {
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

void updateTFT() {
    // La logique d'effacement de l'écran ou de la zone de mise à jour va ici.
  
if(currentScreen+1!=1){// Les bouton + et - ne sont pas disponible sur la page principale
    // 1. Bouton AUGMENTER (UP - Ajustement Setpoint)
    tft.fillRect(UP_BUTTON_X_MIN, UP_BUTTON_Y_MIN, 
                 UP_BUTTON_X_MAX - UP_BUTTON_X_MIN, 
                 UP_BUTTON_Y_MAX - UP_BUTTON_Y_MIN, 
                 TFT_DARKGREEN); 
    
    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE);
    // Dessin du symbole 'plus' ou d'une flèche vers le haut
    tft.setCursor(UP_BUTTON_X_MIN + 60, UP_BUTTON_Y_MIN + 20);
    tft.println("+"); // Ou un caractère flèche '▲'
    
    // 2. Bouton DIMINUER (DOWN - Ajustement Setpoint)
    tft.fillRect(DOWN_BUTTON_X_MIN, DOWN_BUTTON_Y_MIN, 
                 DOWN_BUTTON_X_MAX - DOWN_BUTTON_X_MIN, 
                 DOWN_BUTTON_Y_MAX - DOWN_BUTTON_Y_MIN, 
                 TFT_DARKRED); 
                 
    tft.setTextColor(TFT_WHITE);
    // Dessin du symbole 'moins'
    tft.setCursor(DOWN_BUTTON_X_MIN + 60, DOWN_BUTTON_Y_MIN + 20);
    tft.println("-"); // Ou un caractère flèche '▼'
    tft.setTextSize(1);
    }
    
        //---Bouton page suivante
    int MENU_LARGUEUR=MENU_BUTTON_X_MAX-MENU_BUTTON_X_MIN;
    int MENU_HAUTEUR=MENU_BUTTON_Y_MAX-MENU_BUTTON_Y_MIN;           
    tft.fillRect(MENU_BUTTON_X_MIN, MENU_BUTTON_Y_MIN,MENU_LARGUEUR ,MENU_HAUTEUR, TFT_DARKGREY);
    tft.setTextFont(2);
    tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    tft.drawCentreString("Suivant", 40, 290, 2);

            //---Bouton page précédent
    int PRECED_LARGUEUR=PRECED_BUTTON_X_MAX-PRECED_BUTTON_X_MIN;
    int PRECED_HAUTEUR=PRECED_BUTTON_Y_MAX-PRECED_BUTTON_Y_MIN;           
    tft.fillRect(PRECED_BUTTON_X_MIN, PRECED_BUTTON_Y_MIN,PRECED_LARGUEUR ,PRECED_HAUTEUR, TFT_DARKGREY);
    tft.setTextFont(2);
    tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    tft.drawCentreString("Precedent", 80, 290, 2);

           //---Bouton HOME
    int HOME_LARGUEUR=HOME_BUTTON_X_MAX-HOME_BUTTON_X_MIN;
    int HOME_HAUTEUR=HOME_BUTTON_Y_MAX-HOME_BUTTON_Y_MIN;           
    tft.fillRect(HOME_BUTTON_X_MIN, HOME_BUTTON_Y_MIN,HOME_LARGUEUR ,HOME_HAUTEUR, TFT_DARKGREY);
    tft.setTextFont(2);
    tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    tft.drawCentreString("Home", 230 , 290 , 2);
    
}

void updateDisplay() {
    static int last_currentScreen = -1;
    if (currentScreen != last_currentScreen) {
      EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        last_currentScreen = currentScreen;
    tft.setTextFont(2);
    // --- HEADER (largeur 480) ---
    tft.fillRect(0, 0, 480, 28, TFT_DARKGREY);
    tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    tft.setCursor(6, 6);
    tft.printf("THC | Screen %d/%d", currentScreen + 1, NB_SCREENS);
    updateTFT();
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    switch (currentScreen) {

        case 0: // ---- SCREEN 1 ----
            
            tft.drawString("STATUS : MONITORING", 10, 40, 2);

            tft.drawString("Voltage:", 10, 70, 2);
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.drawFloat(fast_voltage, 1, 220, 70, 2);
            tft.drawFloat(slow_voltage, 1, 250, 70, 2);

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString("Setpoint:", 10, 90, 2);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.drawFloat(Setpoint, 1, 220, 90, 2);

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString("position :", 10, 110, 2);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.drawFloat(stepper.currentPosition(), 1, 220, 110, 2);

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString("THC State:", 10, 130, 2);
            tft.setTextColor(thc_active ? TFT_GREEN : TFT_RED, TFT_BLACK);
            tft.drawString(thc_active ? "ACTIF  ":"INACTIF", 220, 130, 2);

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString("Enable:", 10, 150, 2);
            tft.setTextColor(digitalRead(ENABLE_PIN) ? TFT_RED : TFT_GREEN, TFT_BLACK);
            tft.drawString(digitalRead(ENABLE_PIN)? "INACTIF":"ACTIF    ", 220, 150, 2);

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString("ANTI DIV:", 10, 170, 2);
            tft.setTextColor(digitalRead(anti_dive_active) ? TFT_GREEN : TFT_RED, TFT_BLACK);
            tft.drawString(digitalRead(anti_dive_active) ? "INACTIF":"ACTIF  ", 220, 170, 2);

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString("Tension:", 10, 190, 2);
            tft.setTextColor(arc_voltage_ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
            tft.drawString(arc_voltage_ok ? "OK ":"NOK", 220, 190, 2);
            break;

        case 1: // ---- SCREEN 2 ----
               
            tft.drawString("ADJ. Consigne", 10, 40, 4);
            tft.drawString("Consigne (V):", 10, 100, 4);
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.drawFloat(Setpoint, 1, 260, 100, 4);
            break;

        case 2: // ---- SCREEN 2 ----
            tft.drawString("ADJ. CORRECTION FACTOR", 10, 40, 4);
            tft.drawString("Current Factor:", 10, 100, 4);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.drawFloat(temp_voltage_correction_factor, 2, 260, 100, 4);

            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawString("Saves on exit!", 10, 170, 2);
            break;

        case 3:
            tft.drawString("ADJ. PID Kp", 10, 40, 4);
            tft.drawString("Kp:", 10, 100, 4);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawFloat(Kp, 1, 260, 100, 4);
            break;

        case 4:
            tft.drawString("ADJ. PID Ki", 10, 40, 4);
            tft.drawString("Ki:", 10, 100, 4);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawFloat(Ki, 1, 260, 100, 4);
            break;        
            
        case 5:
            tft.drawString("Adj Steps par mm Z", 10, 40, 4);
            tft.drawString("Steps/mm:", 10, 100, 4);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawFloat(STEPS_PER_MM_Z, 1, 260, 100, 4);
            break;


    }
}

void handleTouchInput(uint16_t x, uint16_t y) {
    // Bouton UP
    if (x >= UP_BUTTON_X_MIN && x <= UP_BUTTON_X_MAX && 
        y >= UP_BUTTON_Y_MIN && y <= UP_BUTTON_Y_MAX) {
        adjustCurrentSetting(1);
        flashButton(UP_BUTTON_X_MIN, UP_BUTTON_Y_MIN, 
                   UP_BUTTON_X_MAX, UP_BUTTON_Y_MAX, TFT_GREEN);
        return;
    }

    // Bouton DOWN
    if (x >= DOWN_BUTTON_X_MIN && x <= DOWN_BUTTON_X_MAX && 
        y >= DOWN_BUTTON_Y_MIN && y <= DOWN_BUTTON_Y_MAX) {
        adjustCurrentSetting(-1);
        flashButton(DOWN_BUTTON_X_MIN, DOWN_BUTTON_Y_MIN, 
                   DOWN_BUTTON_X_MAX, DOWN_BUTTON_Y_MAX, TFT_RED);
        return;
    }

    // Bouton MENU (header)
    if (x >= MENU_BUTTON_X_MIN && x <= MENU_BUTTON_X_MAX && 
        y >= MENU_BUTTON_Y_MIN && y <= MENU_BUTTON_Y_MAX) {
        navigateScreen(1);
        flashButton(MENU_BUTTON_X_MIN, MENU_BUTTON_Y_MIN, 
                   MENU_BUTTON_X_MAX, MENU_BUTTON_Y_MAX, TFT_BLUE);
        return;
    }

        // Bouton Précédent
    if (x >= PRECED_BUTTON_X_MIN && x <= PRECED_BUTTON_X_MAX && 
        y >= PRECED_BUTTON_Y_MIN && y <= PRECED_BUTTON_Y_MAX) {
        navigateScreen(-1);
        flashButton(PRECED_BUTTON_X_MIN, PRECED_BUTTON_Y_MIN, 
                   PRECED_BUTTON_X_MAX, PRECED_BUTTON_Y_MAX, TFT_BLUE);

        return;
    }
                // Bouton Home
    if (x >= HOME_BUTTON_X_MIN && x <= HOME_BUTTON_X_MAX && 
        y >= HOME_BUTTON_Y_MIN && y <= HOME_BUTTON_Y_MAX) {
        navigateScreen(0);
        flashButton(HOME_BUTTON_X_MIN, HOME_BUTTON_Y_MIN, 
                   HOME_BUTTON_X_MAX, HOME_BUTTON_Y_MAX, TFT_BLUE);

        return;
    }
}