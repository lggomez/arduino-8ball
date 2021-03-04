#include <WString.h>
#include "_const.h"

String getMessage(long index) {
  switch (index) {
    case 0:
      return F("It is\ncertain\0");
      break;
    case 1:
      return F("It is\ndecidedly\nso\0");
      break;
    case 2:
      return F("Without\na doubt\0");
      break;
    case 3:
      return F("Yes\ndefinitely\0");
      break;
    case 4:
      return F("You may\nrely\non it\0");
      break;
    case 5:
      return F("As\nI see it,\nyes\0");
      break;
    case 6:
      return F("Most\nlikely\0");
      break;
    case 7:
      return F("Outlook\ngood\0");
      break;
    case 8:
      return F("Yes\0");
      break;
    case 9:
      return F("Signs\npoint to\nyes\0");
      break;
    case 10:
      return F("Reply\nhazy\ntry again\0");
      break;
    case 11:
      return F("Ask\nagain\nlater\0");
      break;
    case 12:
      return F("Better\nnot tell\nyou now\0");
      break;
    case 13:
      return F("Cannot\npredict\nnow\0");
      break;
    default:
      return F("UNDEFINED\0");
      break;
  }
}
