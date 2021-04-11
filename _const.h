//#define DEBUG 1
//#define DEBUG_PLOTTER 1
//#define DEBUG_FIZZLEFADE 0
//#define OUTPUT_LINE_COUNT 0
//#define OUTPUT_DISPLAY_MESSAGE 1

#define DEFAULT_DRAW_COLOR 1
#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64

#define FONT u8g2_font_courB10_tf
#define FONT_WIDTH 11
#define FONT_HEIGTH 16
#define UPPER_MARGIN 8
#define LOWER_MARGIN 8

#define SHAKE_DELAY_INTERVAL 350 // Post-message delay in ms until display is reset

#define FIZZLEFADE_BUFFER_THRESHOLD 200 // LFSR iterations from fizzlefade to wait until 
                                        // writing the page buffer into display

#define MAX_MESSAGE_LENGTH 24

/*
  -----------------DEBUG FEATURE START
*/
#ifdef  DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#ifdef  DEBUG_PLOTTER
#define DEBUG_PRINTP(x) Serial.print(x)
#define DEBUG_PRINTLNP(x) Serial.println(x)
#else
#define DEBUG_PRINTP(x) DEBUG_PRINT(x)
#define DEBUG_PRINTLNP(x) DEBUG_PRINTLN(x)
#endif
/*
  -----------------DEBUG END
*/