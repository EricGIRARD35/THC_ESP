#ifndef UI_SCREENS_H
#define UI_SCREENS_H

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ========================================
// VARIABLES HARDWARE (extern - définies dans main.cpp)
// ========================================

extern TFT_eSPI tft;
extern XPT2046_Touchscreen ts;
extern SemaphoreHandle_t spiMutex;
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t dataMutex;

// EEPROM addresses for parameters (Rien ne change ici)
#define EEPROM_SETPOINT_ADDR 0
#define EEPROM_CORRECTION_FACTOR_ADDR 4
#define EEPROM_STEPS_MM_Z_ADDR 8
#define EEPROM_KP_ADDR 16
#define EEPROM_KI_ADDR 20
#define EEPROM_KD_ADDR 24
#define EEPROM_INITIALIZED_FLAG 28

// ========================================
// CONSTANTES D'ÉCRAN
// ========================================

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

// ========================================
// VARIABLES GLOBALES LVGL (extern)
// ========================================

// ========================================
// VARIABLES GLOBALES LVGL (extern)
// ========================================

// Écrans
extern lv_obj_t *screen_monitoring;
extern lv_obj_t *screen_setpoint;
extern lv_obj_t *screen_correction;
extern lv_obj_t *screen_kp;
extern lv_obj_t *screen_ki;
extern lv_obj_t *screen_steps;
extern lv_obj_t *screen_settings;

// Labels dynamiques - Monitoring
extern lv_obj_t *label_voltage_fast;
extern lv_obj_t *label_voltage_slow;
extern lv_obj_t *label_setpoint;
extern lv_obj_t *label_position;
extern lv_obj_t *label_thc_state;
extern lv_obj_t *label_enable_state;
extern lv_obj_t *label_antidive_state;
extern lv_obj_t *label_arc_state;

// Labels dynamiques - Ajustement
extern lv_obj_t *label_setpoint_val;
extern lv_obj_t *label_correction_val;
extern lv_obj_t *label_kp_val;
extern lv_obj_t *label_ki_val;
extern lv_obj_t *label_steps_val;

// Settings multi-paramètres
extern lv_obj_t *label_param_name;
extern lv_obj_t *label_param_value;
extern int selected_param;
extern const char* param_names[5];

// Variable currentScreen
extern int currentScreen;

// ========================================
// PROTOTYPES DE FONCTIONS
// ========================================
void lvgl_setup();
// Création des écrans
void create_screen_monitoring();
void create_screen_setpoint();
void create_screen_correction();
void create_screen_kp();
void create_screen_ki();
void create_screen_steps();
void create_screen_settings();

// Callbacks
void btn_nav_event_handler(lv_event_t *e);
void btn_adjust_event_handler(lv_event_t *e);
void btn_change_param_event(lv_event_t *e);
void btn_adjust_multi_event(lv_event_t *e);

// Fonctions utilitaires
void update_all_screen_values();
void update_param_value_display();

// Structure de données pour mise à jour
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

void update_lvgl_labels_safe(DisplayData* data);

#endif // UI_SCREENS_H