#include "ui_screens.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>

// ========================================
// DÉCLARATIONS EXTERNES (variables de main.cpp)
// ========================================

// Variables THC (définies dans main.cpp)
extern double Setpoint;
extern double voltage_correction_factor;
extern double Kp, Ki, Kd;
extern float STEPS_PER_MM_Z;
extern PID myPID;

// ========================================
// DÉFINITIONS DES VARIABLES GLOBALES LVGL
// ========================================

// Écrans
lv_obj_t *screen_monitoring = NULL;
lv_obj_t *screen_setpoint = NULL;
lv_obj_t *screen_correction = NULL;
lv_obj_t *screen_kp = NULL;
lv_obj_t *screen_ki = NULL;
lv_obj_t *screen_steps = NULL;
lv_obj_t *screen_settings = NULL;

// Labels dynamiques - Monitoring
lv_obj_t *label_voltage_fast = NULL;
lv_obj_t *label_voltage_slow = NULL;
lv_obj_t *label_setpoint = NULL;
lv_obj_t *label_position = NULL;
lv_obj_t *label_thc_state = NULL;
lv_obj_t *label_enable_state = NULL;
lv_obj_t *label_antidive_state = NULL;
lv_obj_t *label_arc_state = NULL;

// Labels dynamiques - Ajustement
lv_obj_t *label_setpoint_val = NULL;
lv_obj_t *label_correction_val = NULL;
lv_obj_t *label_kp_val = NULL;
lv_obj_t *label_ki_val = NULL;
lv_obj_t *label_steps_val = NULL;

// Settings
lv_obj_t *label_param_name = NULL;
lv_obj_t *label_param_value = NULL;
int selected_param = 0;

const char* param_names[] = {
    "Setpoint (V)",
    "Correction",
    "Kp",
    "Ki",
    "Steps/mm"
};

int currentScreen = 0;
#define LVGL_BUFFER_SIZE (SCREEN_WIDTH * 40)

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LVGL_BUFFER_SIZE];
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

// ========================================
// IMPLÉMENTATION DES FONCTIONS
// ========================================

void create_screen_monitoring() {
    Serial.println("🏗️ Création screen_monitoring...");
    screen_monitoring = lv_obj_create(NULL);
    
    if (screen_monitoring == NULL) {
        Serial.println("❌ ERREUR: lv_obj_create() failed!");
        return;
    }

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
    
    Serial.printf("✅ screen_monitoring créé: %p\n", screen_monitoring);
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