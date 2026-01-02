#ifndef SCREEN_GRAPH_H
#define SCREEN_GRAPH_H

#include <lvgl.h>

// Déclarations publiques
extern lv_obj_t *screen_graph;

// Fonctions publiques
void create_screen_graph();
void update_graph_data(float input, float setpoint, float output);
void graph_update_task();  // À appeler dans loop()

#endif