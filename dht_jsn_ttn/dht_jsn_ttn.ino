#include <TheThingsNetwork.h>
#include <SoftwareSerial.h>

//initialize the LoRa modeule - RN2483 on SoftwareSerial
SoftwareSerial loraSerial = SoftwareSerial(8, 7);
#define debugSerial Serial

// Set your AppEUI and AppKey
const char *appEui = "70B3D57ED0041604";
const char *appKey = "D69B29A152820001FDF3C905100E2299";

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868


#include "DHT.h"

#define DHTPIN 9     // what pin we're connected to
#define trigPin 2
#define echoPin 3
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
uint16_t duration;
uint16_t  distance;
// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);

  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);

  Serial.println("DHTxx test!");
  // Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  dht.begin();
}

void loop() {
  debugSerial.println("-- LOOP");
  // Wait a few seconds between measurements.
  delay(2000);

  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);

  delayMicroseconds(5);
  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance:
  distance = (duration * 0.034 / 2)*100;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  uint16_t h = dht.readHumidity()*100;
  // Read temperature as Celsius
  uint16_t t = dht.readTemperature()*100;
  // Read temperature as Fahrenheit
  uint16_t f = dht.readTemperature(true)*100;

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  uint16_t hi = dht.computeHeatIndex(f, h)*100;

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hi);
  Serial.print(" *F \t");
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");

  byte payload[10];
  payload[0] = highByte(distance);
  payload[1] = lowByte(distance);
  payload[2] = highByte(h);
  payload[3] = lowByte(h);
  payload[4] = highByte(t);
  payload[5] = lowByte(t);
  payload[6] = highByte(f);
  payload[7] = lowByte(f);
  payload[8] = highByte(hi);
  payload[9] = lowByte(hi);



  // Prints the distance on the Serial Monitor
  debugSerial.print("Distance: ");
  debugSerial.println(distance);
  debugSerial.print("Humidity:: ");
  debugSerial.println(h);
  debugSerial.print("Temperature:: ");
  debugSerial.println(t);
  debugSerial.print("Temperature Fahre:: ");
  debugSerial.println(f);
  debugSerial.print("Heat Index:: ");
  debugSerial.println(hi);



  ttn.sendBytes(payload, sizeof(payload));
  delay(2000);
}
