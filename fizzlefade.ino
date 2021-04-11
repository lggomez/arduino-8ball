#include <math.h>
#include <string.h>
#include "_const.h"

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
    Modified fizzlefade implementation for Arduino integrated with u8g2 display
    See http://fabiensanglard.net/fizzlefade/index.php for details on the original

    Changes include:
      - Fade to a given text message
      - Send buffer to display every FIZZLEFADE_BUFFER_THRESHOLD iterations instead of
        doing it for each pixel to improve performance
      - Support for fill or clear mode
*/
void fizzlefade_message(char* charMessage, fizzlefade_mode mode)
{
  u8g2.firstPage();
  u8g2_uint_t fizzlefade_iterations;
  byte lineCount = getLineCount(charMessage);

  if (mode == clear) {
    // Fill the display at start if mode is clear
    drawFill();
  }

  do {
    uint32_t rndval = 1;
    u8g2_uint_t x, y;
    fizzlefade_iterations;

#if OUTPUT_PRINT_MESSAGE
    DEBUG_PRINT(F("Draw string: "));
    DEBUG_PRINT(charMessage);
    DEBUG_PRINTLN(F(""));
#endif
    // Start the fizzlefade LFSR loop
    do
    {
      y =  rndval & 0x000FF;        /* Y = low 8 bits */
      x = (rndval & 0x1FF00) >> 8;  /* X = High 9 bits */
      unsigned lsb = rndval & 1;    /* Get the output bit. */
      rndval >>= 1;                 /* Shift register */
      if (lsb) {                    /* If the output is 0, the xor can be skipped. */
        rndval ^= 0x00012000;
      }

      // If pixel is within bounds, count it as a batch iteration and start message print
      if (x <= SH1106_WIDTH && y <= SH1106_HEIGHT) {
        fizzlefade_iterations++;
        fizzle_message(x, y, charMessage, lineCount, mode, fizzlefade_iterations);
      }
    } while (rndval != 1);
  } while (u8g2.nextPage());

#if DEBUG_FIZZLEFADE
  DEBUG_PRINT(F("Fizzlefade done. iterations: "));
  DEBUG_PRINT(fizzlefade_iterations);
  DEBUG_PRINTLN(F(""));
#endif
}

// fizzle_message draws the random pixel provided and prints the message into the display buffer
// if the iterations match FIZZLEFADE_BUFFER_THRESHOLD, the string will be writter and the
// buffer will be written to the display
void fizzle_message(u8g2_uint_t x, u8g2_uint_t y, char* charMessage, byte lineCount, fizzlefade_mode mode, u8g2_uint_t fizzlefade_iterations)
{
  if (mode == clear) {
    u8g2.setDrawColor(0);
  }
  u8g2.drawPixel(x, y);
  if (fizzlefade_iterations % (FIZZLEFADE_BUFFER_THRESHOLD + 1) == FIZZLEFADE_BUFFER_THRESHOLD) {
    printMessage(charMessage, lineCount, mode);
    u8g2.sendBuffer();
  }
  u8g2.setDrawColor(DEFAULT_DRAW_COLOR);
}

// printMessage prints a string message into the display page buffer, centering it
// TODO: the centering is still a bit off to the left in certain cases
void printMessage(char* charMessage, byte lineCount, fizzlefade_mode mode) {
  u8g2.setDrawColor((mode == clear) ? 1 : 0);
  char* buf;
  for (byte i = 1; i <= lineCount; i++) {
    buf = subStr(charMessage, "\n", i);
    byte x_offset = (strlen(buf) % 2) * floor(FONT_WIDTH / 2);
    byte x = (SH1106_WIDTH / 2) - (FONT_WIDTH * floor(strlen(buf) / 2)) - x_offset;
    byte y_offset = floor(1 - (0.5 * (lineCount - 1))) * FONT_HEIGTH;
    byte y = UPPER_MARGIN + (FONT_HEIGTH * i) + y_offset;
    u8g2.drawStr(x, y, buf);
  }
}

// getLineCount retuns the line count of a string given by the newline delimiter \n
byte getLineCount(const char* charMessage) {
  byte lineCount = 1;
  for (; *charMessage; charMessage++)
    lineCount += *charMessage == '\n';
#if DEBUG_MESSAGE_LINECOUNT
  DEBUG_PRINT(F("Message line count: "));
  DEBUG_PRINT(lineCount);
  DEBUG_PRINTLN(F(""));
#endif
  return lineCount;
}

// subStr returns a substring defined by a delimiter at an index
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