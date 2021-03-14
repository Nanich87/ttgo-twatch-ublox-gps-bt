#include "BluetoothSerial.h"

#include "config.h"

#define BUFFER_SIZE 8192

const char* bt = "u-blox NEO-M8N";

TTGOClass *ttgo;
TFT_eSPI *tft ;
AXP20X_Class *power;
bool irq = false;

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
      if (i2 < BUFFER_SIZE - 1)
      {
        i2++;
      }
    }

    SerialBT.write(bufferSend, i2);
    i2 = 0;
  }
}
