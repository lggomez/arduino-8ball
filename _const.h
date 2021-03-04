#define DEFAULT_DRAW_COLOR 1
#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64

#define FONT u8g2_font_courB10_tf
#define FONT_WIDTH 11
#define FONT_HEIGTH 16
#define UPPER_MARGIN 8
#define LOWER_MARGIN 8

#define REFRESH_INTERVAL 250

#define FIZZLEFADE_BUFSIZE 136

#define MAX_MESSAGE_LENGTH 24

/*
  -----------------DEBUG FLAGS AND FEATURE START
*/
#ifdef  DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define OUTPUT_ACCEL_VECTORS 0
#define OUTPUT_DISPLAY_MESSAGE 1
/*
  -----------------DEBUG FLAGS AND FEATURE END
*/
