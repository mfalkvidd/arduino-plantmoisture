/* Note: Domoticz does not (yet) support setting V_TEXT values through the UI
 * Use https://ip/json.htm?type=command&param=udevice&idx=25&svalue=50
 * to set sleep time to 50 minutes
 */
#include <SPI.h>
#include <MySensor.h>

#define re_send(message,retries) {byte t = 0; while(!gw.send(message, true) && t<retries){Serial.println(millis());gw.wait(pow(2,t)*100));t++;}}
#define re_request(id,type,retries) {byte t = 0; while(!gw.request(id,type,0, true) && t<retries){Serial.println(millis());gw.wait(pow(2,t)*100);t++;}}
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define N_ELEMENTS(array) (sizeof(array)/sizeof((array)[0]))

#define CHILD_ID_MOISTURE 0
#define CHILD_ID_BATTERY 1
#define CHILD_ID_SLEEP_TIME 2
#define RADIO_DELAY 500
#define BUF_SIZE 25 // max payload for MySensors(NRF24l01)
#define THRESHOLD 1.1 // Only make a new reading with reverse polarity if the change is larger than 10%.
#define DEFAULT_SLEEP_TIME_MINUTES 30 // Sleep time between reads
#define MIN_SLEEP_TIME_MINUTES 1 // Do not allow shorter sleep times than 1 minute
#define MAX_SLEEP_TIME_MINUTES 1440 // Do not allow longer sleep times than 24 hours
#define CONFIG_TIME 86400000 // Fetch config updates once per day
#define STABILIZATION_TIME 1000 // Let the sensor stabilize before reading
#define BATTERY_FULL 3143 // 2xAA usually give 3.143V when full
#define BATTERY_ZERO 2340 // 2.43V limit for 328p at 8MHz. 1.9V, limit for nrf24l01 without step-up. 2.8V limit for Atmega328 with default BOD settings.
const int SENSOR_ANALOG_PINS[] = {A0, A1}; // Sensor is connected to these two pins. Avoid A3 if using ATSHA204. A6 and A7 cannot be used because they don't have pullups.

MySensor gw;
MyMessage msg(CHILD_ID_MOISTURE, V_HUM);
MyMessage voltage_msg(CHILD_ID_BATTERY, V_VOLTAGE);
MyMessage sleeptimeMsg(CHILD_ID_SLEEP_TIME, V_TEXT);
long oldvoltage = 0;
byte direction = 0;
int oldMoistureLevel = -1;
unsigned long sleeptime_ms = 0;
unsigned long lastConfigCheckTime = 0;
bool gotMessage = false;

void setup()
{
  Serial.begin(BAUD_RATE);
  gw.begin(incomingMessage);

  gw.sendSketchInfo("Plant moisture w bat", "1.6");
  Serial.println("Present battery");
  gw.present(CHILD_ID_BATTERY, S_CUSTOM);
  gw.wait(RADIO_DELAY);
  Serial.println("Present moisture");
  gw.present(CHILD_ID_MOISTURE, S_HUM);
  gw.wait(RADIO_DELAY);
  for (int i = 0; i < N_ELEMENTS(SENSOR_ANALOG_PINS); i++) {
    pinMode(SENSOR_ANALOG_PINS[i], OUTPUT);
    digitalWrite(SENSOR_ANALOG_PINS[i], LOW);
  }
  Serial.println("Present sleep time");
  gw.present(CHILD_ID_SLEEP_TIME, S_INFO);
  gw.wait(RADIO_DELAY);
  fetchConfig();
}

void loop()
{
  int moistureLevel = readMoisture();

  // Send rolling average of 2 samples to get rid of the "ripple" produced by different resistance in the internal pull-up resistors
  // See http://forum.mysensors.org/topic/2147/office-plant-monitoring/55 for more information
  if (oldMoistureLevel == -1) { // First reading, save current value as old
    oldMoistureLevel = moistureLevel;
  }
  if (moistureLevel > (oldMoistureLevel * THRESHOLD) || moistureLevel < (oldMoistureLevel / THRESHOLD)) {
    // The change was large, so it was probably not caused by the difference in internal pull-ups.
    // Measure again, this time with reversed polarity.
    moistureLevel = readMoisture();
  }
  gw.send(msg.set((moistureLevel + oldMoistureLevel) / 2.0 / 10.23, 1));
  oldMoistureLevel = moistureLevel;
  long voltage = readVcc();
  if (oldvoltage != voltage) { // Only send battery information if voltage has changed, to conserve battery.
    gw.send(voltage_msg.set(voltage / 1000.0, 3)); // redVcc returns millivolts. Set wants volts and how many decimals (3 in our case)
    gw.sendBatteryLevel(round((voltage - BATTERY_ZERO) * 100.0 / (BATTERY_FULL - BATTERY_ZERO)));
    oldvoltage = voltage;
  }
  // Check if we should ask the controller for configuration information
  if (millis() + sleeptime_ms > lastConfigCheckTime + CONFIG_TIME) {
    fetchConfig();
  }
  gw.sleep(sleeptime_ms);
}

int readMoisture() {
  pinMode(SENSOR_ANALOG_PINS[direction], INPUT_PULLUP); // Power on the sensor
  analogRead(SENSOR_ANALOG_PINS[direction]);// Read once to let the ADC capacitor start charging
  gw.sleep(STABILIZATION_TIME);
  int moistureLevel = (1023 - analogRead(SENSOR_ANALOG_PINS[direction]));

  // Turn off the sensor to conserve battery and minimize corrosion
  pinMode(SENSOR_ANALOG_PINS[direction], OUTPUT);
  digitalWrite(SENSOR_ANALOG_PINS[direction], LOW);

  direction = (direction + 1) % 2; // Make direction alternate between 0 and 1 to reverse polarity which reduces corrosion
  return moistureLevel;
}

// This is called when a message is received
void incomingMessage(const MyMessage &message) {
  gotMessage = true;
  Serial.print("Incoming message, time is ");
  Serial.println(millis());
  Serial.println(message.sensor);
  Serial.println(message.type);
  Serial.println(message.getLong());
  if (message.sensor == CHILD_ID_SLEEP_TIME) {
    if (message.type == V_TEXT) {
      long newSleeptime_ms = message.getLong() * 60L * 1000;
      if (newSleeptime_ms >= MIN_SLEEP_TIME_MINUTES * 60L * 1000 && newSleeptime_ms <= MAX_SLEEP_TIME_MINUTES * 60L * 1000) {
        sleeptime_ms = newSleeptime_ms;
        Serial.print("Setting new sleeptime: ");
        Serial.println(sleeptime_ms);
      }
    }
  }
}

void fetchConfig() {
  // Request latest sleep time
  Serial.print("Ask for sleep time, time is ");
  Serial.println(millis());
  // Usually takes approximately 3 seconds to get a response from Domoticz
  // We want to go to sleep as soon as we've gotten an incoming message, but if the controller doesn't respond
  // we need a timeout.
  byte t = 0;
  re_request(CHILD_ID_SLEEP_TIME, V_TEXT, 5);
  while (!gotMessage && t < 200) { // sleep in intervals of 100ms until we get a response or we have tried for 20 seconds
    // incomingMessage() will set gotMessage to true when an incoming message is detected
    gw.wait(100);
    t++;
  }
  gotMessage = false;
  // If we didn't get any sleeptime, sleeptime will be 0.
  // Ask the controller to create the variable by sending the default value
  Serial.print("Done waiting for sleep time, time is ");
  Serial.println(millis());
  if (sleeptime_ms == 0) {
    sleeptime_ms = DEFAULT_SLEEP_TIME_MINUTES * 60L * 1000;
    Serial.println("Send default sleep time");
    gw.send(sleeptimeMsg.set(DEFAULT_SLEEP_TIME_MINUTES));
  }
  lastConfigCheckTime = millis();
}

long readVcc() {
  // From http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

