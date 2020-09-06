//#define DEBUG true    // Uncomment to get some basic Serial output
//#define DEBUG verbose // Uncomment to get more advanced Serial output

#if DEBUG == verbose
  #include <cc1101_debug_service.h>
#endif

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <DFPlayerMini_Fast.h>
#include <AceButton.h>
using namespace ace_button;

typedef union { // makes converting the voltage bytes back into an int easier
  int asInt;
  byte asBytes[2];
} voltage;

// CONSTANTS (Change to your liking)
const int DOORBELL_RINGTONE_CHANGE_BUTTON_PIN = 9;
const int DF_PLAYER_RX_PIN = 8;
const int DF_PLAYER_TX_PIN = 9;
#ifdef ESP32
  const int GDO0 = 2; // for esp32! GDO0 on GPIO pin 2.
#elif ESP8266
  const int GDO0 = 5; // for esp8266! GDO0 on pin 5 = D1.
#else
  const int GDO0 = 6; // for Arduino! GDO0 on pin 6.
#endif
const byte DOORBELL_ID[8] = {127, 33, 45, 91, 27, 60, 8, 16};
const int MIN_TIME_BETWEEN_RINGS = 1000; // milliseconds

// CONSTANTS (Do not touch)
const byte TRANSMITTER_ID = 1;
const byte RECEIVER_ID = 2;
const int RINGTONE_SETTING_ADDRESS = 0x0;

// Other variables that change during runtime (Do not touch)
static unsigned long lastPlayTime = 0;
byte lastTransmitterRetryCount = 255;
bool shouldRingDoorbell = false;
bool shouldTransmitResponse = false;
voltage batteryVoltage;
byte currentDoorbellRingtone;
int volume = 30; // 0-30 (30 = 100%)
int totalTrackCount;

// Instantiate some classes
SoftwareSerial mp3PlayerSerial(DF_PLAYER_TX_PIN, DF_PLAYER_RX_PIN);
DFPlayerMini_Fast mp3Player;

AceButton changeRingtoneButton(DOORBELL_RINGTONE_CHANGE_BUTTON_PIN);
void handleChangeRingtoneButtonEvent(AceButton*, uint8_t, uint8_t);

void setup() {
  #if defined(DEBUG)
    Serial.begin(9600);
    Serial.println("Doorbell Receiver");
  #endif
  
  #if defined(DEBUG)
    Serial.println("Setting Up Internal LED...");
  #endif
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  #if defined(DEBUG)
    Serial.println("Setting up button...");
  #endif
  pinMode(DOORBELL_RINGTONE_CHANGE_BUTTON_PIN, INPUT_PULLUP);

  ButtonConfig* changeRingtoneButtonConfig = changeRingtoneButton.getButtonConfig();
  changeRingtoneButtonConfig->setEventHandler(handleChangeRingtoneButtonEvent);
  changeRingtoneButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  

  #if defined(DEBUG)
    Serial.println("Loading settings...");
  #endif
  currentDoorbellRingtone = EEPROM.read(RINGTONE_SETTING_ADDRESS);
  if (currentDoorbellRingtone < 1 || currentDoorbellRingtone > 254)
      currentDoorbellRingtone = 1;

  #if defined(DEBUG)
    Serial.println("Initializing CC1101 Library...");
  #endif
  ELECHOUSE_cc1101.Init(); // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setGDO(GDO0, 0); // set lib internal gdo pins (GDO0,GDO2). GDO2 not used for this example.
  ELECHOUSE_cc1101.setCCMode(1); // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0); // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(433.92); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setSyncMode(2);  // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  // ELECHOUSE_cc1101.setPA(10);  // set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12)   Default is max!
  ELECHOUSE_cc1101.setCrc(1); // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
  ELECHOUSE_cc1101.SetRx();

  #if defined(DEBUG)
    Serial.println("Initializing Audio Library...");
  #endif
  mp3PlayerSerial.begin(9600);
  mp3Player.begin(mp3PlayerSerial);
  totalTrackCount = mp3Player.numSdTracks();
  #if defined(DEBUG)
    Serial.print("Setting volume to ");
    Serial.print((float)volume/30.0*100.0);
    Serial.println("%...");
  #endif
  mp3Player.volume(volume);
  delay(20);

  #if defined(DEBUG)
    Serial.println("Doorbell Receiver Is Ready");
    Serial.println();
  #endif
}

void loop() {
  #if DEBUG == verbose
    cc1101_debug.debug();
  #endif
  
  unsigned long now = millis();
  
  awaitDoorbellTransmitterSignal();
  
  if (shouldTransmitResponse) {
    sendResponse();
  }
    
  if (shouldRingDoorbell) {
    if (now > lastPlayTime+MIN_TIME_BETWEEN_RINGS) {
        ringDoorbell();
        lastPlayTime = now;
    }
    shouldRingDoorbell = false;
  }

  changeRingtoneButton.check();
}

void handleChangeRingtoneButtonEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {
  currentDoorbellRingtone++;
  if (currentDoorbellRingtone >= totalTrackCount)
    currentDoorbellRingtone = 0;
  #if defined(DEBUG)
    Serial.print("Ringtone changed to file #");
    Serial.println(currentDoorbellRingtone);
    Serial.println("Writing ringtone setting to EEPROM...");
  #endif
  EEPROM.update(RINGTONE_SETTING_ADDRESS, currentDoorbellRingtone);
}

void ringDoorbell() {
  #if defined(DEBUG)
    Serial.println("Doorbell Signal Received!");
    float cellVoltage = millivoltToVolt(batteryVoltage.asInt/2);
    float capacity = convertAlkalineVoltageToCapacity(cellVoltage);
    float usableCapacity = calculateUsableCapacity(capacity);
    Serial.print(usableCapacity);
    Serial.println("% Usable Battery Left");
    #if DEBUG == verbose
      Serial.print(cellVoltage);
      Serial.println("V Per Cell Left");
      Serial.print(capacity);
      Serial.println("% Theorecical Battery Left");
    #endif
  #endif
  digitalWrite(LED_BUILTIN, HIGH);
  #if defined(DEBUG)
    Serial.print("Playing MP3 file #");
    Serial.println(currentDoorbellRingtone);
  #endif
  mp3Player.play(currentDoorbellRingtone);
  digitalWrite(LED_BUILTIN, LOW);
}

void sendResponse() {
  byte packet[9];

  for (int i = 0; i < 8; i++) {
    packet[i] = DOORBELL_ID[i];
  }
  packet[8] = RECEIVER_ID;

  ELECHOUSE_cc1101.SendData(packet, 9);
  
  #if DEBUG == verbose
    Serial.print("Transmit data ");
    for (int i = 0; i < 9; i++) {
      Serial.print(packet[i], DEC);
      Serial.print(",");
    }
    Serial.println();
  #endif

  #if defined(DEBUG)
    Serial.println();
  #endif
  
  ELECHOUSE_cc1101.SetRx();
  shouldTransmitResponse = false;
  lastTransmitterRetryCount = 255;
}

void awaitDoorbellTransmitterSignal() {
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
      if (buffer[8] != TRANSMITTER_ID) {
        return;
      }

      #if DEBUG == verbose
        Serial.print("Data Received: ");
        for (int i = 0; i < 8; i++) {
          Serial.print(buffer[i]);
          Serial.print(",");
        }
        Serial.println();
      #endif
  
      batteryVoltage.asBytes[0] = buffer[10];
      batteryVoltage.asBytes[1] = buffer[11];
      
      if (buffer[9] <= lastTransmitterRetryCount) {
        lastTransmitterRetryCount = buffer[9];
        shouldRingDoorbell = true;
      }
      shouldTransmitResponse = true;
    }
  }
}

float millivoltToVolt(int millivolt) {
    return (float)millivolt/1000;
}

// For alkaline-based 1.5V AA cells
float convertAlkalineVoltageToCapacity(float v) {
  if (v >= 1.55)
    return 100.0; //static value
  else if (v <= 0)
    return 0.0; //static value
  else if (v > 1.4)
    return 60.60606*v + 6.060606; //linear regression
  else if (v < 1.1)
    return 8.3022*v; //linear regression
  else
    return 9412 - 23449*v + 19240*v*v - 5176*v*v*v; // cubic regression
}

// Since the Arduino requires at least 2.4V to run, we pretend the battery capacity is 0% when it goes below 2.4V (2 * 1.2V)
// This happens at about 27%
const float USABLE_CAPACITY_LOWER_LIMIT = 27;
float calculateUsableCapacity(float realCapacity) {
  float usableCapacity = (realCapacity-USABLE_CAPACITY_LOWER_LIMIT) / (100-USABLE_CAPACITY_LOWER_LIMIT) * 100;
  if (usableCapacity < USABLE_CAPACITY_LOWER_LIMIT)
      return 0.0;
  else
      return usableCapacity;
}
