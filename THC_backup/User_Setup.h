#define ILI9488_DRIVER

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC   21
#define TFT_RST   -1

// Broche de Chip Select du contrôleur Tactile
#define TOUCH_CS 15 
// Broche d'Interruption du Tactile (T_IRQ)
//#define T_IRQ 13

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SPI_FREQUENCY 16000000
#define SPI_READ_FREQUENCY 20000000
#define SPI_TOUCH_FREQUENCY 2500000

// Ajustement couleurs (nécessaire sur ILI9488)
#define TFT_RGB_ORDER TFT_BGR


