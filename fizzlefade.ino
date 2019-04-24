#include <math.h>
#include <string.h>
#include "_const.h"

#define OUTPUT_PRINT_FIZZLEFADE 1
#define OUTPUT_LINE_COUNT 0

void drawFill(void) {
  u8g2.drawBox(0, 0, SH1106_WIDTH, SH1106_HEIGHT);
}

void drawFillPaged(void) {
  u8g2.firstPage();
  do {
    drawFill();
  } while (u8g2.nextPage());
}

/*
    Modified fizzlefade implementation for arduino
    See http://fabiensanglard.net/fizzlefade/index.php for details on the original

    Changes include:
      - Fade to text
      - Send buffer to lcd every FIZZLEFADE_BUFSIZE iterations instead of
        doing it for each pixel to improve performance
      - Support for fill or clear mode
*/
boolean fizzlefade_message(char* charMessage, fizzlefade_mode mode)
{
  u8g2.firstPage();
  u8g2_uint_t batch_count;

  do {
    uint32_t rndval = 1;
    u8g2_uint_t x, y;
    batch_count;
    byte lineCount = getLineCount(charMessage);
#if OUTPUT_PRINT_MESSAGE
    DEBUG_PRINT(F("Draw string: "));
    DEBUG_PRINT(charMessage);
    DEBUG_PRINTLN(F(""));
#endif
    do
    {
      y =  rndval & 0x000FF;        /* Y = low 8 bits */
      x = (rndval & 0x1FF00) >> 8;  /* X = High 9 bits */
      unsigned lsb = rndval & 1;    /* Get the output bit. */
      rndval >>= 1;                 /* Shift register */
      if (lsb) {                    /* If the output is 0, the xor can be skipped. */
        rndval ^= 0x00012000;
      }
      if (x <= SH1106_WIDTH && y <= SH1106_HEIGHT) {
        batch_count++;
        fizzle_message(x , y, charMessage, lineCount, mode, batch_count);
      }
    } while (rndval != 1);
  } while (u8g2.nextPage());

#if OUTPUT_PRINT_FIZZLEFADE
  DEBUG_PRINT(F("Fizzlefade done. iterations: "));
  DEBUG_PRINT(batch_count);
  DEBUG_PRINTLN(F(""));
#endif
  return 0;
}

void fizzle_message(u8g2_uint_t x, u8g2_uint_t y, char* charMessage, byte lineCount, fizzlefade_mode mode, u8g2_uint_t batch_count)
{
  if (mode == clear) {
    u8g2.setDrawColor(0);
  }
  u8g2.drawPixel(x, y);
  if (batch_count % (FIZZLEFADE_BUFSIZE + 1) == FIZZLEFADE_BUFSIZE) {
    printMessage(charMessage, lineCount, mode);
    u8g2.sendBuffer();
  }
  u8g2.setDrawColor(DEFAULT_DRAW_COLOR);
}

void printMessage(char* charMessage, byte lineCount, fizzlefade_mode mode) {
  u8g2.setDrawColor((mode == clear) ? 1 : 0);
  char* buf;
  for (byte i = 1; i <= lineCount; i++) {
    buf = subStr(charMessage, "\n", i);
    byte xOffset = (strlen(buf) % 2) * floor(FONT_WIDTH / 2);
    byte x = (SH1106_WIDTH / 2) - (FONT_WIDTH * floor(strlen(buf) / 2)) - xOffset;
    byte yOffset = floor(1 - (0.5 * (lineCount - 1))) * FONT_HEIGTH;
    byte y = UPPER_MARGIN + (FONT_HEIGTH * i) + yOffset;
    u8g2.drawStr(x, y, buf);
  }
}

byte getLineCount(const char* charMessage) {
  byte lineCount = 1;
  for (; *charMessage; charMessage++)
    lineCount += *charMessage == '\n';
#if OUTPUT_LINE_COUNT
  DEBUG_PRINT(F("Message line count: "));
  DEBUG_PRINT(lineCount);
  DEBUG_PRINTLN(F(""));
#endif
  return lineCount;
}

// Function to return a substring defined by a delimiter at an index
char* subStr(char* str, char *delim, byte index) {
  char *act, *sub, *ptr;
  static char copy[MAX_MESSAGE_LENGTH];
  byte i;

  // Since strtok consumes the first arg, make a copy
  strcpy(copy, str);

  for (i = 1, act = copy; i <= index; i++, act = NULL) {
    sub = strtok_r(act, delim, &ptr);
    if (sub == NULL) break;
  }
  return sub;
}
