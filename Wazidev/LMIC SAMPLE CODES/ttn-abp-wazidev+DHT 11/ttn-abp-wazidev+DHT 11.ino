#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <LowPower.h>
#include "DHT.h"
#define DHTPIN 6    // Digital pin connected to the DHT sensor
// Enable debug prints to serial monitor
#define MY_DEBUG
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);
// const int sensor_pin = A0;
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
// DEVEUI und AppEUI LSB
//static const PROGMEM u1_t NWKSKEY[16] = { 0xED, 0xB4, 0x31, 0x5B, 0x4F, 0xEB, 0x49, 0x7B, 0x22, 0x28, 0x86, 0x0A, 0x6B, 0x72, 0xCC, 0x30}; //dpm-1
//static const PROGMEM u1_t NWKSKEY[16] = { 0x28, 0x0D, 0xE0, 0x72, 0xA3, 0x7D, 0xB9, 0x51, 0xB3, 0xB9, 0xED, 0xCD, 0xB5, 0x08, 0xFC, 0x97}; // dpm-2
//static const PROGMEM u1_t NWKSKEY[16] = { 0xE8, 0x10, 0x8C, 0x86, 0x04, 0x9F, 0x6A, 0x5A, 0x13, 0x1C, 0x68, 0x12, 0x99, 0x9B, 0x84, 0xC3}; //dpm-3
//static const PROGMEM u1_t NWKSKEY[16] = { 0x2D, 0x0C, 0x5F, 0x72, 0xB6, 0x70, 0xEB, 0x27, 0xA7, 0x65, 0x7C, 0xF2, 0x9F, 0xB9, 0xB9, 0x5C}; // dpm-4
static const PROGMEM u1_t NWKSKEY[16] = { 0xD9, 0x61, 0xC2, 0x00, 0x85, 0x84, 0xE9, 0xA5, 0x46, 0x1C, 0x10, 0x84, 0x41, 0x81, 0x43, 0xA8 }; // dpm-5

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
// MSB

//static const u1_t PROGMEM APPSKEY[16] = { 0x69, 0x11, 0xC6, 0x29, 0x0F, 0x58, 0x4D, 0x10, 0x54, 0x09, 0x59, 0x3C, 0x98, 0x04, 0x1E, 0x87 }; //dpm -1
//static const u1_t PROGMEM APPSKEY[16] = { 0x2C, 0x8D, 0x71, 0xE4, 0xE0, 0xE7, 0xB8, 0x1A, 0xFD, 0x33, 0x9F, 0x71, 0x78, 0x95, 0xAE, 0xF7 }; //dpm - 2
//static const u1_t PROGMEM APPSKEY[16] = { 0x40, 0xE9, 0x07, 0x16, 0x02, 0xCC, 0x07, 0x24, 0xE8, 0x32, 0x7C, 0x90, 0x04, 0x4B, 0xF2, 0x0D }; //dpm - 3
//static const u1_t PROGMEM APPSKEY[16] = { 0xB3, 0xB3, 0xB5, 0xE1, 0xEB, 0x98, 0x77, 0xDC, 0x4F, 0xD7, 0x2B, 0xE8, 0x85, 0x82, 0xA1, 0xF9 }; //dpm - 4
static const u1_t PROGMEM APPSKEY[16] = { 0x8D, 0xEB, 0xDA, 0x7A, 0xE6, 0x20, 0x02, 0xD5, 0xC1, 0x70, 0x0E, 0x4B, 0x5D, 0xC2, 0xF9, 0x7C }; //dpm - 5

// LoRaWAN end-device address (DevAddr)
//static const u4_t DEVADDR = 0x260B46ED; //dpm- 1
//static const u4_t DEVADDR = 0x260B370E; //dpm-2
//static const u4_t DEVADDR = 0x260BCC64; //dpm- 3
//static const u4_t DEVADDR = 0x260B8122; //dpm-4
static const u4_t DEVADDR = 0x260BAC5E; //dpm-5

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL_MINUTES = 1;
const unsigned TX_INTERVAL = 60 * TX_INTERVAL_MINUTES;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {2, 3, LMIC_UNUSED_PIN}, //DIO0, DIO1 and DIO2 connected
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      do_send(&sendjob);
      for (int i = 0; i < int(TX_INTERVAL / 8); i++) {
        // Use library from https://github.com/rocketscream/Low-Power
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      }
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Wait a few seconds between measurements.
  delay(2000);
  int sensorValue = analogRead(A7);
  uint16_t v = ((sensorValue* 4.14) / 1023.0)*100;
  uint16_t h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  uint16_t t = dht.readTemperature();

 

  byte buffer[6];
  buffer[0] = t >> 8;
  buffer[1] = t;
  buffer[2] = h >> 8;
  buffer[3] = h;
  buffer[4] = v >> 8;
  buffer[5] = v;
 

  LMIC_setTxData2(1, buffer, sizeof(buffer), 0);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C  "));
  Serial.print(F(" Battery Voltage: "));
  Serial.print(v);
  Serial.println(" v");
 //Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
void setup() {
  Serial.begin(38400);
  Serial.println(F("Starting"));
  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  //    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  //    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7B),  BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_FSK, DR_FSK),  BAND_MILLI);      // g2-

  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
  dht.begin();
}
void loop() {
  os_runloop_once();
}