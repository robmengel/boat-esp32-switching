#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <Wire.h>

// ================= CONFIGURATION =================
const int NUM_CHANNELS = 8; 
const unsigned long STATUS_TRANSMIT_INTERVAL = 1000; // Send N2K status every 1 second
unsigned long lastStatusTime = 0;

// ================= RELAY MAPPING (BANK 0 - OUTPUTS) =================
// Total: 5 used, 3 spare
#define RELAY_CABIN_LIGHT   0 // Relay 1 (Tied to DI1 momentary button)
#define RELAY_ANCHOR_LIGHTS 1 // Relay 2 (Auto-enabled when Nav Lights turn ON)
#define RELAY_NAV_LIGHTS    2 // Relay 3
#define RELAY_DASH_LIGHTS   3 // Relay 4
#define RELAY_WIPER         4 // Relay 5 (Single speed + flyback diode)
#define RELAY_SPARE_6       5 // Relay 6 (Available)
#define RELAY_SPARE_7       6 // Relay 7 (Available)
#define RELAY_SPARE_8       7 // Relay 8 (Available)

// ================= DIGITAL INPUT MAPPING (BANK 1 - SENSORS) =================
#define DI_CABIN_LIGHT_PIN  39 // DI1: Momentary switch input for cabin light
#define DI_HIGH_WATER_PIN   38 // DI2: Bilge high-water float switch sensor
#define DI_BILGE_RUN_PIN    37 // DI3: Bilge pump active/running power monitor
#define DI_PORT_TILT_PIN    36 // DI4: Port Volvo 250 drive tilt-up warning switch
#define DI_STBD_TILT_PIN    35 // DI5: Starboard Volvo 250 drive tilt-up warning switch

// Waveshare Board Hardware Pin Definitions
#define I2C_SDA 42
#define I2C_SCL 41
#define TCA9554_ADDR 0x20

// TCA9554 Register Addresses
#define REG_INPUT 0x00
#define REG_OUTPUT 0x01
#define REG_POLARITY 0x02
#define REG_CONFIG 0x03

Preferences preferences;
bool relayStates[NUM_CHANNELS];
uint8_t relayBitmask = 0x00;

// Sensor & Switch State Tracking
bool lastCabinButtonState = HIGH;
unsigned long lastCabinDebounceTime = 0;

bool highWaterAlarmActive = false;
bool bilgeRunningActive = false;
bool portTiltActive = false;
bool stbdTiltActive = false;

int lastHighWaterReading = HIGH;
int lastBilgeRunReading = HIGH;
int lastPortTiltReading = HIGH;
int lastStbdTiltReading = HIGH;

unsigned long lastHWDebounceTime = 0;
unsigned long lastBilgeRunDebounceTime = 0;
unsigned long lastPortTiltDebounceTime = 0;
unsigned long lastStbdTiltDebounceTime = 0;

const unsigned long DEBOUNCE_DELAY = 50;

// Forward declarations
void applyRelayToHardware(int channel, bool state);
void initRelayHardware();
void saveRelayStateToFlash(int channel, bool state);

// ================= NMEA 2000 CALLBACK =================
void HandleBinaryStatusControl(const tN2kMsg &N2kMsg) {
  unsigned char BankInstance;
  tN2kBinaryStatus BankStates;

  if (ParseN2kBinaryStatus(N2kMsg, BankInstance, BankStates)) {
    // Only process control commands for Bank 0 (Relay Outputs)
    if (BankInstance == 0) {
      for (int i = 0; i < NUM_CHANNELS; i++) {
        tN2kOnOff requestedState = N2kGetBinaryStatus(BankStates, i + 1);
        
        if (requestedState != N2kOnOff_Unavailable) {
          bool newState = (requestedState == N2kOnOff_On);
          
          if (relayStates[i] != newState) {
            relayStates[i] = newState;
            applyRelayToHardware(i, newState);
            saveRelayStateToFlash(i, newState);
          }
        }
      }

      // --- NAV & ANCHOR INTERLOCK LOGIC ---
      // If Nav Lights were just turned ON, automatically force Anchor Lights ON
      if (relayStates[RELAY_NAV_LIGHTS] == true && relayStates[RELAY_ANCHOR_LIGHTS] == false) {
        relayStates[RELAY_ANCHOR_LIGHTS] = true;
        applyRelayToHardware(RELAY_ANCHOR_LIGHTS, true);
        saveRelayStateToFlash(RELAY_ANCHOR_LIGHTS, true);
      }
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Initialize Relay Hardware & I2C
  initRelayHardware();

  // Configure Digital Input pins as inputs with internal pull-up
  pinMode(DI_CABIN_LIGHT_PIN, INPUT_PULLUP);
  pinMode(DI_HIGH_WATER_PIN, INPUT_PULLUP);
  pinMode(DI_BILGE_RUN_PIN, INPUT_PULLUP);
  pinMode(DI_PORT_TILT_PIN, INPUT_PULLUP);
  pinMode(DI_STBD_TILT_PIN, INPUT_PULLUP);

  // 2. Load Persistent States from Flash for Relays
  preferences.begin("relays", false);
  for (int i = 0; i < NUM_CHANNELS; i++) {
    String key = "r" + String(i);
    relayStates[i] = preferences.getBool(key.c_str(), false); // Default to OFF
    applyRelayToHardware(i, relayStates[i]);
  }
  preferences.end();

  // 3. Configure NMEA 2000 Stack
  NMEA2000.SetProductInformation("12935", 100, "ESP32-Relay", "1.0.0", "1.0.0");
  NMEA2000.SetDeviceInformation(123456, 130, 25, 2046);

  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleBinaryStatusControl);

  const unsigned long TransmitMessages[] = {127501L, 0};
  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.Open();
}

// ================= LOOP =================
void loop() {
  // Process incoming NMEA 2000 CAN frames
  NMEA2000.Parse();

  unsigned long currentMillis = millis();

  // --- 1. POLL DI1: Cabin Light Momentary Switch ---
  int cabinReading = digitalRead(DI_CABIN_LIGHT_PIN);
  if (cabinReading != lastCabinButtonState) {
    lastCabinDebounceTime = currentMillis;
  }
  if ((currentMillis - lastCabinDebounceTime) > DEBOUNCE_DELAY) {
    if (cabinReading == LOW && lastCabinButtonState == HIGH) {
      relayStates[RELAY_CABIN_LIGHT] = !relayStates[RELAY_CABIN_LIGHT];
      applyRelayToHardware(RELAY_CABIN_LIGHT, relayStates[RELAY_CABIN_LIGHT]);
      saveRelayStateToFlash(RELAY_CABIN_LIGHT, relayStates[RELAY_CABIN_LIGHT]);
    }
  }
  lastCabinButtonState = cabinReading;


  // --- 2. POLL DI2: Bilge High-Water Alarm ---
  int hwReading = digitalRead(DI_HIGH_WATER_PIN);
  if (hwReading != lastHighWaterReading) {
    lastHWDebounceTime = currentMillis;
  }
  if ((currentMillis - lastHWDebounceTime) > DEBOUNCE_DELAY) {
    highWaterAlarmActive = (hwReading == LOW);
  }
  lastHighWaterReading = hwReading;


  // --- 3. POLL DI3: Bilge Pump Running Status ---
  int bilgeRunReading = digitalRead(DI_BILGE_RUN_PIN);
  if (bilgeRunReading != lastBilgeRunReading) {
    lastBilgeRunDebounceTime = currentMillis;
  }
  if ((currentMillis - lastBilgeRunDebounceTime) > DEBOUNCE_DELAY) {
    bilgeRunningActive = (bilgeRunReading == LOW);
  }
  lastBilgeRunReading = bilgeRunReading;


  // --- 4. POLL DI4: Port Drive Tilt Warning ---
  int portTiltReading = digitalRead(DI_PORT_TILT_PIN);
  if (portTiltReading != lastPortTiltReading) {
    lastPortTiltDebounceTime = currentMillis;
  }
  if ((currentMillis - lastPortTiltDebounceTime) > DEBOUNCE_DELAY) {
    portTiltActive = (portTiltReading == LOW);
  }
  lastPortTiltReading = portTiltReading;


  // --- 5. POLL DI5: Starboard Drive Tilt Warning ---
  int stbdTiltReading = digitalRead(DI_STBD_TILT_PIN);
  if (stbdTiltReading != lastStbdTiltReading) {
    lastStbdTiltDebounceTime = currentMillis;
  }
  if ((currentMillis - lastStbdTiltDebounceTime) > DEBOUNCE_DELAY) {
    stbdTiltActive = (stbdTiltReading == LOW);
  }
  lastStbdTiltReading = stbdTiltReading;


  // --- 6. PERIODIC STATUS TRANSMISSION (PGN 127501) ---
  if (currentMillis - lastStatusTime >= STATUS_TRANSMIT_INTERVAL) {
    lastStatusTime = currentMillis;

    tN2kMsg N2kMsg;

    // --- Bank 0: Relay Outputs ---
    tN2kBinaryStatus relayBank;
    N2kResetBinaryStatus(relayBank);
    for (int i = 0; i < NUM_CHANNELS; i++) {
      N2kSetBinaryStatus(relayBank, relayStates[i] ? N2kOnOff_On : N2kOnOff_Off, i + 1);
    }
    SetN2kBinaryStatus(N2kMsg, 0, relayBank); // Instance 0
    NMEA2000.SendMsg(N2kMsg);

    // --- Bank 1: Sensor & Alarm Status ---
    tN2kBinaryStatus sensorBank;
    N2kResetBinaryStatus(sensorBank);
    N2kSetBinaryStatus(sensorBank, highWaterAlarmActive ? N2kOnOff_On : N2kOnOff_Off, 1);
    N2kSetBinaryStatus(sensorBank, bilgeRunningActive ? N2kOnOff_On : N2kOnOff_Off, 2);
    N2kSetBinaryStatus(sensorBank, portTiltActive ? N2kOnOff_On : N2kOnOff_Off, 3);
    N2kSetBinaryStatus(sensorBank, stbdTiltActive ? N2kOnOff_On : N2kOnOff_Off, 4);
    
    SetN2kBinaryStatus(N2kMsg, 1, sensorBank); // Instance 1
    NMEA2000.SendMsg(N2kMsg);
  }
}

// ================= HELPER FUNCTIONS =================
void saveRelayStateToFlash(int channel, bool state) {
  preferences.begin("relays", false);
  preferences.putBool(String("r" + String(channel)).c_str(), state);
  preferences.end();
}

void initRelayHardware() {
  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(0x00); 
  Wire.endTransmission();

  relayBitmask = 0x00;
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(REG_OUTPUT);
  Wire.write(relayBitmask);
  Wire.endTransmission();
}

void applyRelayToHardware(int channel, bool state) {
  if (channel < 0 || channel >= NUM_CHANNELS) return;

  if (state) {
    relayBitmask |= (1 << channel);  
  } else {
    relayBitmask &= ~(1 << channel); 
  }

  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(REG_OUTPUT);
  Wire.write(relayBitmask);
  Wire.endTransmission();

  Serial.printf("Relay %d set to: %s (Bitmask: 0x%02X)\n", channel + 1, state ? "ON" : "OFF", relayBitmask);
}