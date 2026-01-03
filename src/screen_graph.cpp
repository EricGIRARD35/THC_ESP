#include "screen_graph.h"
#include <Arduino.h>

// Variables externes (définies dans main.cpp)
extern float Setpoint;
extern int currentScreen;
extern void navigate_to_screen(int screen_index);
extern void btn_nav_event_handler(lv_event_t *e);

// Variables locales (privées à ce fichier)
lv_obj_t *screen_graph = NULL;
static lv_obj_t *chart = NULL;
static lv_chart_series_t *ser_input = NULL;
static lv_chart_series_t *ser_setpoint = NULL;
static lv_chart_series_t *ser_output = NULL;
static lv_obj_t *label_title = NULL;

static lv_obj_t *lbl_in = NULL;
static lv_obj_t *lbl_sp = NULL;
static lv_obj_t *lbl_out = NULL;

#define CHART_POINTS 100

// ========================================
// FONCTION : Créer l'écran graphique
// ========================================

void create_screen_graph() {
    // Création écran
    screen_graph = lv_obj_create(NULL);
    
    // ========================================
    // HEADER (Titre + bouton retour)
    // ========================================
    
    // Container header
    lv_obj_t *header = lv_obj_create(screen_graph);
    lv_obj_set_size(header, 480, 50);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x2C3E50), 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    
    // Titre
    label_title = lv_label_create(header);
    lv_label_set_text(label_title, "GRAPHIQUE TEMPS REEL");
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(label_title, LV_ALIGN_LEFT_MID, 10, 0);
    
    // Bouton retour (← HOME)
    lv_obj_t *btn_back = lv_btn_create(header);
    lv_obj_set_size(btn_back, 70, 50);
    lv_obj_align(btn_back, LV_ALIGN_RIGHT_MID, -5, 0);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0x34495E), 0);
    lv_obj_add_event_cb(btn_back, btn_nav_event_handler, LV_EVENT_CLICKED, (void*)0);
    
    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, LV_SYMBOL_HOME);
    lv_obj_center(lbl_back);
        
    // ========================================
    // INFOS VALEURS ACTUELLES (en haut)
    // ========================================
    
    lv_obj_t *info_container = lv_obj_create(screen_graph);
    lv_obj_set_size(info_container, 460, 40);
    lv_obj_align(info_container, LV_ALIGN_TOP_MID, 0, 55);
    lv_obj_set_style_bg_opa(info_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(info_container, 0, 0);
    lv_obj_set_flex_flow(info_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(info_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // Input
    lbl_in = lv_label_create(info_container);
    lv_label_set_text(lbl_in, "In: --.-V");
    lv_obj_set_style_text_color(lbl_in, lv_color_hex(0x0000FF), 0); 
    
    // Setpoint
    lbl_sp = lv_label_create(info_container);
    lv_label_set_text(lbl_sp, "SP: --.-V");
    lv_obj_set_style_text_color(lbl_sp, lv_color_hex(0x00FF00), 0);
    
    // Output
    lbl_out = lv_label_create(info_container);
    lv_label_set_text(lbl_out, "Out: --.--mm");
    lv_obj_set_style_text_color(lbl_out, lv_color_hex(0xFF0000), 0);
    
    // ========================================
    // GRAPHIQUE LVGL
    // ========================================
    
    chart = lv_chart_create(screen_graph);
    lv_obj_set_size(chart, 460, 180);
    lv_obj_align(chart, LV_ALIGN_CENTER, 0, 10);
    
    // Configuration chart
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, CHART_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 80, 160);  // 80-160V
    lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, -500, 500); // -5 à 5mm
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_SECONDARY_Y, 5, 2, 5, 2, true, 40);
    lv_chart_set_div_line_count(chart, 5, 10);  // Grille
    
    // Style
    lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);  // Pas de points
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);  // Lignes épaisses
    
    // ========================================
    // 3 SÉRIES (courbes)
    // ========================================
    
    ser_input = lv_chart_add_series(chart, lv_color_hex(0x0000FF), LV_CHART_AXIS_PRIMARY_Y);      // Bleu
    ser_setpoint = lv_chart_add_series(chart, lv_color_hex(0x00FF00), LV_CHART_AXIS_PRIMARY_Y);   // Vert
    ser_output = lv_chart_add_series(chart, lv_color_hex(0xFF0000), LV_CHART_AXIS_SECONDARY_Y);     // Rouge
    
    // Initialise à 120V (milieu)
    for (int i = 0; i < CHART_POINTS; i++) {
        ser_input->y_points[i] = 120;
        ser_setpoint->y_points[i] = 120;
    }
    
    lv_chart_refresh(chart);
    
    // ========================================
    // LÉGENDE
    // ========================================
    
    lv_obj_t *legend = lv_obj_create(screen_graph);
    lv_obj_set_size(legend, 460, 30);
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_opa(legend, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(legend, 0, 0);
    lv_obj_set_flex_flow(legend, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(legend, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // Input legend
    lv_obj_t *leg_in = lv_label_create(legend);
    lv_label_set_text(leg_in, "━ Input");
    lv_obj_set_style_text_color(leg_in, lv_color_hex(0x0000FF), 0);
    
    // Setpoint legend
    lv_obj_t *leg_sp = lv_label_create(legend);
    lv_label_set_text(leg_sp, "━ Setpoint");
    lv_obj_set_style_text_color(leg_sp, lv_color_hex(0x00FF00), 0);
    
    // Output legend
    lv_obj_t *leg_out = lv_label_create(legend);
    lv_label_set_text(leg_out, "━ Output");
    lv_obj_set_style_text_color(leg_out, lv_color_hex(0xFF0000), 0);
    
    Serial.println("✅ Écran graphique créé");
}

// ========================================
// FONCTION : Mise à jour des données
// ========================================

void update_graph_data(float input, float setpoint, float output) {
    if (chart == NULL) return;
    
    // Ajoute les points (shift automatique vers la gauche)
    lv_chart_set_next_value(chart, ser_input, (int32_t)input);
    lv_chart_set_next_value(chart, ser_setpoint, (int32_t)setpoint);
    lv_chart_set_next_value(chart, ser_output, (int32_t)(output*100.0f)); // Conversion mm en centièmes pour plus de précision
    
        // Mise a jour des labels
    char buf[32];
    
    if(lbl_in) {
        snprintf(buf, sizeof(buf), "In: %.1fV", input);
        lv_label_set_text(lbl_in, buf);
    }
    
    if(lbl_sp) {
        snprintf(buf, sizeof(buf), "SP: %.1fV", setpoint);
        lv_label_set_text(lbl_sp, buf);
    }
    
    if(lbl_out) {
        snprintf(buf, sizeof(buf), "Out: %.2fmm", output);
        lv_label_set_text(lbl_out, buf);
    }
    // Force redessinage
    lv_chart_refresh(chart);
}

// ========================================
// FONCTION : Mise à jour échelle auto
// ========================================

static void update_chart_scale() {
    if (chart == NULL || ser_input == NULL || ser_output == NULL) return;
    
    // --- ÉCHELLE PRIMAIRE (Tension : Input & Setpoint) ---
    int32_t min_v = 9999, max_v = -9999;
    
    // --- ÉCHELLE SECONDAIRE (Mouvement : Output) ---
    int32_t min_z = 9999, max_z = -9999;
    
    for (int i = 0; i < CHART_POINTS; i++) {
        // Scan pour la Tension (Axe Gauche)
        if (ser_input->y_points[i] < min_v) min_v = ser_input->y_points[i];
        if (ser_input->y_points[i] > max_v) max_v = ser_input->y_points[i];
        if (ser_setpoint->y_points[i] < min_v) min_v = ser_setpoint->y_points[i];
        if (ser_setpoint->y_points[i] > max_v) max_v = ser_setpoint->y_points[i];

        // Scan pour l'Output/Position (Axe Droit)
int16_t val_z = (int16_t)ser_output->y_points[i]; 
    if (val_z < min_z) min_z = val_z;
    if (val_z > max_z) max_z = val_z;
    }

    // --- Calcul et application Axe Primaire (Gauche) ---
    int32_t range_v = max_v - min_v;
    if (range_v < 10) range_v = 10;
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, min_v - (range_v/10), max_v + (range_v/10));

    // --- Calcul et application Axe Secondaire (Droit) ---
    int32_t range_z = max_z - min_z;
    if (range_z < 20) range_z = 20; // Seuil mini pour ne pas trop zoomer sur le bruit
    lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, min_z - (range_z/10), max_z + (range_z/10));
}

// ========================================
// FONCTION : Tâche de mise à jour (appeler dans loop)
// ========================================

void graph_update_task() {
    static unsigned long last_scale_update = 0;
    
    // Mise à jour échelle toutes les 2 secondes
    if (millis() - last_scale_update >= 2000) {
        update_chart_scale();
        last_scale_update = millis();
    }
}