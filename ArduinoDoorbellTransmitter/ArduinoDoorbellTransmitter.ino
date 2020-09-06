//#define DEBUG true    // Uncomment to get some basic Serial output
//#define DEBUG verbose // Uncomment to get more advanced Serial output

#if DEBUG == verbose
  #include <cc1101_debug_service.h>
#endif

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

typedef union { // makes converting the voltage int into 2 bytes easier
  int asInt;
  byte asBytes[2];
} voltage;

// CONSTANTS (Change to your liking)
const int WAKE_BUTTON_PIN = 2; // interrupt pin to wake the Arduino out of deep sleep
#ifdef ESP32
  const int GDO0 = 2; // for esp32! GDO0 on GPIO pin 2.
#elif ESP8266
  const int GDO0 = 5; // for esp8266! GDO0 on pin 5 = D1.
#else
  const int GDO0 = 6; // for Arduino! GDO0 on pin 6.
#endif
const byte DOORBELL_ID[8] = {127, 33, 45, 91, 27, 60, 8, 16};

int DELAY_BETWEEN_RETRIES = 500; // milliseconds
int MAX_RETRIES = 3;

// CONSTANTS (Do not touch)
const byte TRANSMITTER_ID = 1;
const byte RECEIVER_ID = 2;
byte currentRetryCount = 0;

// Other variables that change during runtime (Do not touch)
static int interruptPin;

byte adcsraSave; // to save the ADCSRA value
voltage batteryVoltage;
static unsigned long lastTxTime = 0;
bool shouldGoToSleep = true;

#if defined(DEBUG)
  static unsigned long lastWakeupTime = 0;
#endif
  
void setup() {
  
  #if defined(DEBUG)
    Serial.begin(9600);
    Serial.println("Doorbell Transmitter");
  #endif

  #if defined(DEBUG)
    Serial.println("Setting up power saving settings...");
  #endif
  for (int pin = 0; pin < 20; pin++) { // all pins to output - power saving
    pinMode(pin,OUTPUT);
    digitalWrite(pin,LOW);
  }
  wdt_disable(); // disable WDT for power saving
  EIFR = 3; // clear external interrupt flag register
  //ADCSRA = 0; // disable ADC for power saving

  #if defined(DEBUG)
    Serial.println("Setting Up Interrupt Pin...");
  #endif
  pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);
  //pinMode(WAKE_BUTTON_PIN, INPUT);
  interruptPin = digitalPinToInterrupt(WAKE_BUTTON_PIN);

  #if defined(DEBUG)
    Serial.println("Initializing CC1101 Library...");
  #endif
  ELECHOUSE_cc1101.setGDO(GDO0, 0); // set lib internal gdo pins (GDO0,GDO2). GDO2 not used for this example.
  ELECHOUSE_cc1101.Init(); // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setCCMode(1); // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0); // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(433.92); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setSyncMode(2); // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  // ELECHOUSE_cc1101.setPA(10); // set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12)   Default is max!
  ELECHOUSE_cc1101.setCrc(1); // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.

  #if defined(DEBUG)
    Serial.println("Doorbell Transmitter Is Ready!");
  #endif
  delay(100);
}

void loop() {
  #if DEBUG == verbose
    cc1101_debug.debug();
  #endif
  
  if (shouldGoToSleep) {
    enterSleep();
    shouldGoToSleep = false;
    currentRetryCount = 0;
    batteryVoltage.asInt = getVoltage();
    #if DEBUG == verbose
      Serial.print("Battery voltage: ");
      Serial.print(batteryVoltage.asInt);
      Serial.println("mV");
    #endif
  }
  
  if (!shouldGoToSleep) {
      unsigned long now = millis();
      if (currentRetryCount == 0 || now - lastTxTime > DELAY_BETWEEN_RETRIES) {
        sendDoorbellSignal();
        lastTxTime = now;
        
        #if defined(DEBUG)
          Serial.println("Waiting For A Response...");
        #endif
      }
      
      awaitDoorbellReceiverResponse();
      if (currentRetryCount > MAX_RETRIES) {
        #if defined(DEBUG)
          Serial.println("Failed To Get A Response From The Doorbell Reveicer!");
        #endif
        shouldGoToSleep = true;
      }
  }
}

void sendDoorbellSignal() {
  byte packet[12];

  for (int i = 0; i < 8; i++) {
    packet[i] = DOORBELL_ID[i];
  }
  packet[8] = TRANSMITTER_ID;
  packet[9] = currentRetryCount;
  packet[10] = batteryVoltage.asBytes[0];
  packet[11] = batteryVoltage.asBytes[1];

  currentRetryCount++;

  #if defined(DEBUG)
    Serial.println("Sending Doorbell Signal...");
  #endif
  
  ELECHOUSE_cc1101.SendData(packet, 12);
  
  #if defined(DEBUG)
    Serial.println("Sent Doorbell Signal!");
  #endif
  
  #if DEBUG == verbose
    Serial.print("Transmit data ");
    for (int i = 0; i < 12; i++) {
      Serial.print(packet[i], DEC);
      Serial.print(",");
    }
    Serial.println();
  #endif

  ELECHOUSE_cc1101.SetRx();
}

void awaitDoorbellReceiverResponse() {
  if (ELECHOUSE_cc1101.CheckRxFifo(50)) {
    if (ELECHOUSE_cc1101.CheckCRC()) {

      byte buffer[61] = {0};
      int len = ELECHOUSE_cc1101.ReceiveData(buffer);
      buffer[len] = '\0';

      for (int i = 0; i < 8; i++) {
        if (buffer[i] != DOORBELL_ID[i]) {
          return;
        }
      }
      if (buffer[8] != RECEIVER_ID) {
        return;
      }

      #if DEBUG == verbose
        Serial.print("Received data ");
        for (int i = 0; i < 9; i++) {
          Serial.print(buffer[i]);
          Serial.print(",");
        }
        Serial.println();
      #endif

      #if defined(DEBUG)
        Serial.println("Successfully Received A Response!");
      #endif
      
      shouldGoToSleep = true;
    }
  }
}

// Read the voltage of the battery the Arduino is currently running on (in millivolts)
int getVoltage(void) {
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // For mega boards
    const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #else // For 168/328 boards
    const long InternalReferenceVoltage = 1056L;
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #endif
  //delay(50); // Let mux settle a little to get a more stable A/D conversion
  ADCSRA |= _BV( ADSC ); // Start a conversion 
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) ); // Wait for it to complete
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // Scale the value; calculates for straight line value
  return results*10; // convert from centivolts to millivolts
}

void onWakeUp() {
  // detachInterrupt causes the Arduino wake up twice. no clue why
  // detachInterrupt(interruptPin); //eliminates switch bouncing issues
  sleep_disable();
}

void enterSleep() {
  #if defined(DEBUG)
    Serial.println("Entering Sleep Mode...");
  #endif

  // Save ADCSRA value
  adcsraSave = ADCSRA;
  ADCSRA = 0; // disable ADC

  // Put CC1101 module to sleep
  ELECHOUSE_cc1101.SpiStrobe(0x36); // Exit RX / TX, turn off frequency synthesizer and exit
  ELECHOUSE_cc1101.SpiStrobe(0x39); // Enter power down mode when CSn goes high.

  #if defined(DEBUG)
    Serial.print("I was awake for ");
    Serial.print(millis()-lastWakeupTime);
    Serial.println("ms!");
    delay(100);
  #endif
  Serial.println();

  set_sleep_mode (SLEEP_MODE_PWR_DOWN); // Deep sleep
  sleep_enable();
  attachInterrupt(interruptPin, onWakeUp, FALLING);
  sleep_bod_disable(); // disable brownout detector during sleep
  sleep_cpu(); // now go to sleep

  #if defined(DEBUG)
    lastWakeupTime = millis();
  #endif

  ADCSRA = adcsraSave;

  #if defined(DEBUG)
    Serial.println("Woke Up From Sleep!");
    Serial.println("Doorbell Button Has Been Pressed!");
  #endif
}
