#define DEBUG true    // Uncomment to get some basic Serial output
//#define DEBUG verbose // Uncomment to get more advanced Serial output

#if DEBUG == verbose
  #include <cc1101_debug_service.h>
#endif

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <avr/sleep.h>
#include <avr/power.h>

typedef union { // makes converting the voltage int into 2 bytes easier
  int asInt;
  byte asBytes[2];
} voltage;

const int WAKE_BUTTON_PIN = 2; // interrupt pin to wake the Arduino out of deep sleep

#ifdef ESP32
  const int GDO0 = 2; // for esp32! GDO0 on GPIO pin 2.
#elif ESP8266
  const int GDO0 = 5; // for esp8266! GDO0 on pin 5 = D1.
#else
  const int GDO0 = 6; // for Arduino! GDO0 on pin 6.
#endif

static int interruptPin = 0;

const byte DOORBELL_ID[8] = {127, 33, 45, 91, 27, 60, 8, 16};
const byte TRANSMITTER_ID = 1;
const byte RECEIVER_ID = 2;
byte currentRetryCount = 0;

voltage batteryVoltage;
bool goToSleep = true;

int delayBetweenRetries = 1000; // milliseconds
int maxRetries = 10;
static unsigned long lastTxTime;

#if defined(DEBUG)
  static unsigned long lastWakeupTime;
#endif
  
void setup() {
  #if defined(DEBUG)
    Serial.begin(9600);
    Serial.println("Doorbell Transmitter");
  #endif

  #if defined(DEBUG)
    Serial.println("Setting Up Interrupt Pin...");
  #endif
  pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);
  //pinMode(WAKE_BUTTON_PIN, INPUT);
  interruptPin = digitalPinToInterrupt(WAKE_BUTTON_PIN);

  #if defined(DEBUG)
    Serial.println("Initializing CC1101 Library...");
  #endif
  ELECHOUSE_cc1101.Init(); // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setGDO(GDO0, 0); // set lib internal gdo pins (GDO0,GDO2). GDO2 not used for this example.
  ELECHOUSE_cc1101.setCCMode(1); // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0); // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(433.92); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setSyncMode(2); // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  // ELECHOUSE_cc1101.setPA(10); // set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12)   Default is max!
  ELECHOUSE_cc1101.setCrc(1); // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.

  #if defined(DEBUG)
    Serial.println("Doorbell Transmitter Is Ready!");
  #endif
}

void loop() {
  #if DEBUG == verbose
    cc1101_debug.debug();
  #endif
  
  if (goToSleep) {
      enterSleep();
  }
  
  //if (digitalRead(WAKE_BUTTON_PIN) == LOW) {
  //  currentRetryCount = 0;
  //}
  
  if (!goToSleep) {
      batteryVoltage.asInt = getVoltage();
      transmit_data();
      receive_data();
      if (currentRetryCount > maxRetries) {
        #if defined(DEBUG)
          Serial.println("Failed To Get A Response From The Doorbell Reveicer!");
        #endif
        goToSleep = true;
      }
  }

  //#if defined(DEBUG)
  //  Serial.println("Test I am wake"); // Test for Sleepmode work!
  //#endif
}
//////////////////////////////////////////////////////////
void transmit_data() {
  unsigned long now = millis();
  if (currentRetryCount == 0 || now - lastTxTime > delayBetweenRetries) {
    byte Packet[12];

    for (int i = 0; i < 8; i++) {
      Packet[i] = DOORBELL_ID[i];
    }
    Packet[8] = TRANSMITTER_ID;
    Packet[9] = currentRetryCount;
    Packet[10] = batteryVoltage.asBytes[0];
    Packet[11] = batteryVoltage.asBytes[1];

    currentRetryCount++;

    #if defined(DEBUG)
      Serial.println("Sending Doorbell Signal...");
    #endif
    
    ELECHOUSE_cc1101.SendData(Packet, 12);
    
    #if DEBUG == verbose
      Serial.print("Transmit data ");
      for (int i = 0; i < 12; i++) {
        Serial.print(Packet[i], DEC);
        Serial.print(",");
      }
      Serial.println();
    #endif
    
    ELECHOUSE_cc1101.SetRx();
    
    #if defined(DEBUG)
      Serial.println("Waiting For A Response...");
    #endif
    
    lastTxTime = now;
  }
}
//////////////////////////////////////////////////////////
void receive_data() {
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
      
      goToSleep = true;
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
  delay(50); // Let mux settle a little to get a more stable A/D conversion
  ADCSRA |= _BV( ADSC ); // Start a conversion 
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) ); // Wait for it to complete
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // Scale the value; calculates for straight line value
  return results*10; // convert from centivolt to volt
}

void onWakeUp() {
  currentRetryCount = 0;
  goToSleep = false;
  //detachInterrupt(interruptPin);// arduino freezes. but works fine without a detach!
}

void enterSleep() {
  #if defined(DEBUG)
    Serial.println("Entering Sleep Mode...");
  #endif

  ELECHOUSE_cc1101.SpiStrobe(0x36);//Exit RX / TX, turn off frequency synthesizer and exit
  ELECHOUSE_cc1101.SpiStrobe(0x39);//Enter power down mode when CSn goes high.
  pinMode(GDO0,OUTPUT);

  attachInterrupt(interruptPin, onWakeUp, CHANGE);

  #if defined(DEBUG)
    Serial.print("I was awake for ");
    Serial.print(millis()-lastWakeupTime);
    Serial.println("ms");
  #endif
  Serial.println();
  
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  sleep_mode();

  sleep_disable();

  #if defined(DEBUG)
    lastWakeupTime = millis();
  #endif
  
  pinMode(GDO0,INPUT);

  #if defined(DEBUG)
    Serial.println("Woke Up From Sleep!");
  #endif

  #if defined(DEBUG)
    Serial.println("Doorbell Button Has Been Pressed!");
  #endif
}
