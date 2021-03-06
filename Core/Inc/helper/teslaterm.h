
#include <stdint.h>
#include "cli_basic.h"

#define TT_GAUGE 1
#define TT_GAUGE_CONF 2
#define TT_CHART 3
#define TT_CHART_DRAW 4
#define TT_CHART_CONFIG 5
#define TT_CHART_CLEAR 6
#define TT_CHART_LINE 7
#define TT_CHART_TEXT 8
#define TT_CHART_TEXT_CENTER 9
#define TT_STATUS 10
#define TT_CONFIG_GET 11
#define TT_EVENT 12


#define TT_UNIT_NONE 0
#define TT_UNIT_V 1
#define TT_UNIT_A 2
#define TT_UNIT_W 3
#define TT_UNIT_Hz 4
#define TT_UNIT_C 5

#define TT_COLOR_WHITE 0
#define TT_COLOR_RED 1
#define TT_COLOR_BLUE 2
#define TT_COLOR_GREEN 3
#define TT_COLOR_GRAY 8



void send_gauge(uint8_t gauge, int16_t val, port_str *ptr);

void send_chart(uint8_t chart, int16_t val, port_str *ptr);
void send_chart_draw(port_str *ptr);
void send_chart_config(uint8_t chart, int16_t min, int16_t max, int16_t offset, uint8_t unit,char * text, port_str *ptr);
void send_gauge_config(uint8_t gauge, int16_t min, int16_t max, char * text, port_str *ptr);
void send_config(char* param, const char* help_text, port_str *ptr);

