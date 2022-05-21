#include "BluetoothSerial.h"

#include "config.h"

#define BUFFER_SIZE 8192
#define COLOR_ORANGE 0xFBE0
#define COLOR_GREY 0x39C4

const char* bt = "u-blox NEO-M8N";

TTGOClass *ttgo;
TFT_eSPI *tft ;
AXP20X_Class *power;
bool irq = false;
byte xcolon = 0;

TinyGPSPlus gps;

HardwareSerial *hwSerial = nullptr;
BluetoothSerial SerialBT;

uint8_t bufferReceive[BUFFER_SIZE];
uint16_t i1 = 0;

uint8_t bufferSend[BUFFER_SIZE];
uint16_t i2 = 0;

void pressed()
{
  ttgo->setBrightness(170);
}

void released()
{
  ttgo->setBrightness(85);
}

void setup(void)
{
  Serial.begin(115200);

  ttgo = TTGOClass::getWatch();
  ttgo->begin();
  ttgo->openBL();
  ttgo->setBrightness(85);

  tft = ttgo->tft;
  power = ttgo->power;

  tft->fillScreen(TFT_BLACK);
  tft->setTextFont(2);

  pinMode(AXP202_INT, INPUT_PULLUP);
  attachInterrupt(AXP202_INT, [] {
    irq = true;
  }, FALLING);

  power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ, true);
  power->clearIRQ();

  // GPS
  tft->println("Initializing u-blox NEO-M8N...");

  ttgo->enableLDO3();

  if (hwSerial == nullptr)
  {
    hwSerial = new HardwareSerial(1);
  }
  hwSerial->begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);

  tft->setTextColor(TFT_GREEN);
  tft->println("Done.");

  // BT
  tft->setTextColor(TFT_WHITE);
  tft->println("Initializing bluetooth: ");
  tft->setTextColor(TFT_YELLOW);
  tft->println(bt);

  SerialBT.begin(bt);

  tft->setTextColor(TFT_GREEN);
  tft->println("Done.");

  tft->setTextColor(TFT_WHITE);

  // User button
  ttgo->button->setPressedHandler(pressed);
  ttgo->button->setReleasedHandler(released);
}

void loop(void)
{
  if (irq)
  {
    irq = false;

    power->readIRQ();

    if (power->isPEKShortPressIRQ() )
    {
      if (ttgo->bl->isOn())
      {
        ttgo->closeBL();
        ttgo->displaySleep();
      }
      else
      {
        ttgo->displayWakeup();
        ttgo->openBL();
      }
    }

    power->clearIRQ();
  }

  ttgo->button->loop();

  if (SerialBT.available())
  {
    while (SerialBT.available())
    {
      bufferReceive[i1] = (uint8_t)SerialBT.read();
      if (i1 < BUFFER_SIZE - 1)
      {
        i1++;
      }
    }

    hwSerial->write(bufferReceive, i1);
    i1 = 0;
  }

  if (hwSerial->available())
  {
    while (hwSerial->available())
    {
      bufferSend[i2] = (char)hwSerial->read();
      gps.encode(bufferSend[i2]);
      if (i2 < BUFFER_SIZE - 1)
      {
        i2++;
      }
    }

    SerialBT.write(bufferSend, i2);

    tft->setTextSize(1);
    tft->setTextColor(COLOR_ORANGE, TFT_BLACK);

    uint8_t hh = gps.time.hour();
    uint8_t mm = gps.time.minute();
    uint8_t ss = gps.time.second();
    uint8_t dday = gps.date.day();
    uint8_t mmonth = gps.date.month();
    uint16_t yyear = gps.date.year();

    byte xpos = 40;
    byte ypos = 90;

    if (hh < 10)
    {
      xpos += tft->drawChar('0', xpos, ypos, 7);
    }

    xpos += tft->drawNumber(hh, xpos, ypos, 7);
    xcolon = xpos + 3;
    xpos += tft->drawChar(':', xcolon, ypos, 7);

    if (mm < 10)
    {
      xpos += tft->drawChar('0', xpos, ypos, 7);
    }

    tft->drawNumber(mm, xpos, ypos, 7);

    if (ss % 2)
    {
      tft->setTextColor(COLOR_GREY, TFT_BLACK);
      xpos += tft->drawChar(':', xcolon, ypos, 7);
      tft->setTextColor(COLOR_ORANGE, TFT_BLACK);
    }
    else
    {
      tft->drawChar(':', xcolon, ypos, 7);
    }

    tft->setTextSize(2);
    tft->setCursor(36, 150);

    if (dday < 10)
    {
      tft->print("0");
    }

    tft->print(dday);
    tft->print(".");

    if (mmonth < 10)
    {
      tft->print("0");
    }

    tft->print(mmonth);
    tft->print(".");
    tft->print(yyear);

    tft->setTextSize(1);

    if (gps.location.isValid())
    {
      tft->setCursor(10, 200);
      tft->print(gps.location.lat(), 8);
      tft->print(F(","));
      tft->print(gps.location.lng(), 8);
    }

    if (gps.satellites.isValid())
    {
      tft->setTextColor(TFT_WHITE, TFT_BLACK);
      tft->print(F(" ["));
      tft->print(gps.satellites.value());
      tft->print(F("]"));
    }

    if (gps.altitude.isValid())
    {
      tft->setTextColor(TFT_GREEN, TFT_BLACK);
      tft->setCursor(10, 220);
      tft->print(gps.altitude.meters(), 3);
      tft->print(F("m"));
    }

    if (gps.speed.isValid())
    {
      tft->setTextColor(TFT_YELLOW, TFT_BLACK);
      tft->setCursor(80, 220);
      tft->print(gps.speed.kmph(), 1);
      tft->print(F("km/h"));
    }

    if(gps.course.isValid())
    {
      tft->setTextColor(TFT_CYAN, TFT_BLACK);
      tft->setCursor(160, 220);
      tft->print(gps.course.deg(), 1);
      tft->print(F("d"));
    }

    i2 = 0;
  }

  delay(1000);
}
