//OLED
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64  
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire);

//BUZZER
#define BUZZER_PIN 9

// 'hr', 32x32px
const unsigned char epd_bitmap_hr [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc3, 0xe0, 0x0f, 0xe7, 0xf0, 0x1f, 
	0xff, 0xf8, 0x1f, 0xff, 0xfc, 0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xfc, 0x3c, 0x7c, 0xfc, 0x00, 0x20, 
	0x7c, 0x01, 0x00, 0x78, 0x1f, 0x8c, 0xf8, 0x0f, 0xff, 0xf0, 0x07, 0xff, 0xe0, 0x03, 0xff, 0xc0, 
	0x01, 0xff, 0x80, 0x00, 0xff, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x18, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'hum', 32x32px
const unsigned char epd_bitmap_hum [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 
	0x00, 0x3f, 0xfc, 0x00, 0x00, 0x7c, 0x3e, 0x00, 0x00, 0xe0, 0x07, 0x00, 0x01, 0xc0, 0x03, 0xc0, 
	0x03, 0x87, 0xf1, 0xc0, 0x07, 0x23, 0xf8, 0xe0, 0x0e, 0x70, 0x1c, 0x70, 0x0c, 0x78, 0x0e, 0x30, 
	0x0c, 0xfc, 0x07, 0x30, 0x1c, 0xce, 0x03, 0x38, 0x18, 0xc7, 0x83, 0x18, 0x18, 0xc3, 0xc3, 0x98, 
	0x18, 0x03, 0xc0, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1c, 0x00, 0x00, 0x30, 0x0c, 0x00, 0x00, 0x78, 
	0x0c, 0x00, 0x00, 0xf8, 0x0e, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x01, 0xfc, 0x03, 0x80, 0x03, 0xfe, 
	0x01, 0xc0, 0x03, 0xfe, 0x01, 0xe0, 0x03, 0xff, 0x00, 0x7c, 0x27, 0xff, 0x00, 0x3f, 0xe7, 0xff, 
	0x00, 0x07, 0xe3, 0xff, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x78
};
// 'temp', 32x32px
const unsigned char epd_bitmap_temp [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x07, 0xe0, 0x00, 
	0x00, 0x0e, 0x70, 0x00, 0x00, 0x0c, 0x30, 0x00, 0x00, 0x0c, 0x30, 0x00, 0x00, 0x0c, 0x3c, 0x00, 
	0x00, 0x0c, 0x3e, 0x00, 0x00, 0x0c, 0x38, 0x00, 0x00, 0x0c, 0x38, 0x00, 0x00, 0x0d, 0xbe, 0x00, 
	0x00, 0x0d, 0xbc, 0x00, 0x00, 0x0d, 0xb0, 0x00, 0x00, 0x0d, 0xbc, 0x00, 0x00, 0x0d, 0xbc, 0x00, 
	0x00, 0x0d, 0xb0, 0x00, 0x00, 0x0d, 0xb0, 0x00, 0x00, 0x1d, 0xb8, 0x00, 0x00, 0x19, 0x98, 0x00, 
	0x00, 0x33, 0xcc, 0x00, 0x00, 0x37, 0xec, 0x00, 0x00, 0x67, 0xe6, 0x00, 0x00, 0x67, 0xe6, 0x00, 
	0x00, 0x67, 0xe6, 0x00, 0x00, 0x37, 0xcc, 0x00, 0x00, 0x39, 0x9c, 0x00, 0x00, 0x1c, 0x38, 0x00, 
	0x00, 0x0f, 0xf0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'spo2', 20x20px
const unsigned char epd_bitmap_spo2 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0xf0, 0x00, 0x01, 0xf8, 0x00, 0x01, 0xf8, 0x00, 0x03, 
	0xfc, 0x00, 0x03, 0xfc, 0x00, 0x07, 0xfe, 0x00, 0x07, 0xfe, 0x00, 0x0f, 0xff, 0x00, 0x0f, 0xff, 
	0x00, 0x1f, 0xff, 0x80, 0x1f, 0xff, 0x80, 0x1f, 0xff, 0x80, 0x1f, 0xff, 0x80, 0x0d, 0xff, 0x00, 
	0x0e, 0xff, 0x00, 0x07, 0xfe, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x60, 0x00
};
// 'dd', 32x32px
const unsigned char epd_bitmap_dd [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf1, 0xe0, 0x00, 0xf9, 0xf1, 0xf0, 0x00, 0x60, 0x60, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x0f, 0x8f, 0x9f, 0x00, 0x07, 0x8f, 0x9f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xf1, 0xe0, 0x00, 0xf9, 0xf1, 0xf0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0f, 0x8f, 0x9f, 0x00, 0x07, 0x8f, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'aq', 32x32px
const unsigned char epd_bitmap_aq [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x0f, 0x3c, 0x00, 
	0x00, 0x0e, 0x1c, 0x00, 0x00, 0x04, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x00, 
	0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x78, 0x00, 0xff, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xe0, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xf0, 
	0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x0e, 0x7f, 0xc0, 0x00, 0x06, 
	0xff, 0xf0, 0x00, 0x07, 0x7f, 0xf8, 0x00, 0x07, 0x00, 0x1c, 0x00, 0x07, 0x00, 0x0c, 0x18, 0x07, 
	0x00, 0x0e, 0x1c, 0x06, 0x00, 0x0e, 0x1e, 0x0e, 0x04, 0x0e, 0x0f, 0xfc, 0x0e, 0x1c, 0x07, 0xf8, 
	0x07, 0xfc, 0x01, 0xe0, 0x03, 0xf8, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'rr', 23x23px
const unsigned char epd_bitmap_rr [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x01, 
	0x01, 0x00, 0x03, 0x93, 0x80, 0x07, 0x93, 0xc0, 0x0f, 0xd7, 0xe0, 0x0f, 0xff, 0xe0, 0x1f, 0xff, 
	0xf0, 0x3f, 0xab, 0xf8, 0x3c, 0x00, 0x78, 0x7e, 0xc4, 0xfc, 0x7e, 0xc6, 0xfc, 0xfd, 0xc6, 0x7e, 
	0xfc, 0xc6, 0x7e, 0xff, 0xc6, 0xfe, 0xff, 0x83, 0xfe, 0xff, 0x01, 0xfe, 0xfe, 0x00, 0xfe, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00
};

//AIR QUALITY
const int airQuality = A0;

//DUST DENSITY
int measurePin = A3;
int ledPower = 3;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

//TEMPERATURE AND HUMIDITY
#include <dht.h>
dht DHT; //a variable type of dht
const int DHT11_PIN= 2;

//MAX30102
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100]; // Infrared LED sensor data
uint32_t redBuffer[100]; // Red LED sensor data

int32_t bufferLength; // Data length
int32_t spo2; // SpO2 value
int8_t validSPO2; // Indicator if SpO2 calculation is valid
int32_t heartRate; // Heart rate value
int8_t validHeartRate; // Indicator if heart rate calculation is valid

byte pulseLED = 11; // Must be on PWM pin
byte readLED = 13; // Blinks with each data read
int sampleIndex = 0; // Tracks the current position in the buffer

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    // Wait for Serial Monitor to open
  }
  // AIR QUALITY
  pinMode(airQuality, INPUT);

  //DUST DENSITY
  pinMode(ledPower, OUTPUT);

  //HEART RATE AND OXYGEN SATURATION
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  //OLED
  Wire.begin();
  display.begin(0x3C, true);  // 0x3C is the I2C address
  display.clearDisplay();
  display.display();
}

// TIME INTERVALS
//OLED
const unsigned long screenInterval = 1000; // 1 second per screen
unsigned long previousMillis = 0;
int screenNumber = 1; // for case switch

//RR
unsigned long lastBreathTime = 0;
unsigned long lastBPMUpdate = 0;
const unsigned long minBreathInterval = 1000;  // seconds between breaths
const unsigned long bpmUpdateInterval = 60000; // Update BPM every 60 seconds
int breathCount = 0;
int bpmToDisplay = -1; // -1 means "reading"
int soundSensorPin = A1;  // Replace with actual sensor pin
int thresholdr = 35;  // Adjust this threshold based on your sensor readings

//AIR QUALITY
int threshold = 250;
unsigned long lastAirQualityRead = 0;
const unsigned long airQualityInterval = 5000;  // Read every 5 seconds
int airq = -1;  // -1 means "reading"

// TEMPERATURE
unsigned long previousTempMillis = 0;
const long tempInterval = 2000;  // Read temperature and humidity every 2 seconds
int lastTemperature = -1;
int lastHumidity = -1;

// DUST DENSITY
unsigned long previousDustMillis = 0;
const long dustInterval = 2000; 

// HEART RATE AND OXY SAT
unsigned long previousHRMillis = 0;
const long heartRateInterval = 2000; // Update heart rate and SpO2 every 2 seconds

// ASTHMA PREDICT
unsigned long previousAsthmaMillis = 0;
const unsigned long asthmaCheckInterval = 5000; // Runs every 5 seconds

void loop() {
  String breathRate = detectBreathing();  // Call function and store result
  int intValBR = breathRate.toInt();
  String aq = detectAirQuality();
  int intValAQ = aq.toInt();
  detectTemperatureHumidity();
  int temp = getTemperature();
  int hum = getHumidity();
  detectDustDensity();  // Update dust density every 2 seconds
  float dust = getDustDensity();
  detectHeartRateAndOxySat();
  String hr = getHeartRate();
  int intHR = hr.toInt();
  String o2sat = getSpO2();
  int intSAT = o2sat.toInt();
  String ap = detectAsthmaRisk(intValAQ, dust, temp, hum, intHR, intSAT, intValBR);

  unsigned long currentMillis = millis();

  // OLED Sequence
  if (currentMillis - previousMillis >= screenInterval) {
    previousMillis = currentMillis; // Update the last execution time
    switch (screenNumber) {
      case 1:
        oledSc1(aq, dust);
        screenNumber = 2;
        break;
      case 2:
        oledSc2(temp, hum);
        screenNumber = 3;
        break;
      case 3:
        oledSc3(hr, o2sat);
        screenNumber = 4;
        break;
      case 4:
        oledSc4(breathRate);
        screenNumber = 5;
        break;
      case 5:
        oledSc5(ap);
        screenNumber = 1;
        break;
    }
  }
}


// OLED DISPLAY SEQUENCE
void oledSc1(String airQuality, float dustDensity) {
  display.clearDisplay();
  // AIR QUALITY
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 1);
  display.setTextSize(1);
  display.print("Air Quality: ");
  display.display();

  display.setCursor(10, 12);
  display.setTextSize(2);
  display.println(airQuality);
  display.display();

  display.drawBitmap(96, 0, (const uint8_t*)epd_bitmap_aq, 32, 32, SH110X_WHITE);
  display.display();

  //DUST DENSITY
  display.setCursor(10, 35);
  display.setTextSize(1);
  display.print("Dust Density: ");
  display.display();

  display.setCursor(10, 47);
  display.setTextSize(2);
  display.println(String(dustDensity));
  display.display();

  display.drawBitmap(96, 32, (const uint8_t*)epd_bitmap_dd, 32, 32, SH110X_WHITE);
  display.display();
}

void oledSc2(int temperature, int humidity){
  display.clearDisplay();
  // TEMPERATURE
  display.setCursor(10, 1);
  display.setTextSize(1);
  display.print("Temperature: ");
  display.display();

  display.setCursor(10, 12);
  display.setTextSize(2);
  display.println(String(temperature) + " " + char(247) + "C");
  display.display();

  display.drawBitmap(96, 0, (const uint8_t*)epd_bitmap_temp, 32, 32, SH110X_WHITE);
  display.display();

  //HUMIDITY
  display.setCursor(10, 35);
  display.setTextSize(1);
  display.print("Humidity: ");
  display.display();

  display.setCursor(10, 47);
  display.setTextSize(2);
  display.println(String(humidity) + " %");
  display.display();

  display.drawBitmap(96, 32, (const uint8_t*)epd_bitmap_hum, 32, 32, SH110X_WHITE);
  display.display();
}

void oledSc3(String hr, String spo2){
  display.clearDisplay();
  // HEART RATE
  display.setCursor(10, 1);
  display.setTextSize(1);
  display.print("Heart Rate: ");
  display.display();

  display.setCursor(10, 12);
  display.setTextSize(2);
  display.println(String(hr) + " bpm" );
  display.display();

  display.drawBitmap(105, 5, (const uint8_t*)epd_bitmap_hr, 24, 24, SH110X_WHITE);
  display.display();

  // OXYGEN SATURATION
  display.setCursor(10, 35);
  display.setTextSize(1);
  display.print("O2 Saturation: ");
  display.display();

  display.setCursor(10, 47);
  display.setTextSize(2);
  display.println(String(spo2) + " %");
  display.display();

  display.drawBitmap(105, 39, (const uint8_t*)epd_bitmap_spo2, 20, 20, SH110X_WHITE);
  display.display();
}

void oledSc4(String rr){
  display.clearDisplay();
  // HEART RATE
  display.setCursor(10, 1);
  display.setTextSize(1);
  display.print("Respiration Rate: ");
  display.display();

  display.setCursor(10, 12);
  display.setTextSize(2);
  display.print(rr + " bpm");  // Show stored BPM after 60s
  display.display();

  display.drawBitmap(101, 5, (const uint8_t*)epd_bitmap_rr, 23, 23, SH110X_WHITE);
  display.display();
}

void oledSc5(String ap){
  display.clearDisplay();
  // HEART RATE
  display.setCursor(10, 1);
  display.setTextSize(1);
  display.print("Asthma Prediction: ");
  display.display();

  display.setCursor(10, 12);
  display.setTextSize(1);
  display.print(ap);  // Show stored BPM after 60s
  display.display();
}

// SENSOR READINGS
String detectBreathing() {
    int soundLevel = analogRead(soundSensorPin);

    if (soundLevel > thresholdr) {  
        unsigned long currentTime = millis();
        if (currentTime - lastBreathTime > minBreathInterval) {
            breathCount++;
            lastBreathTime = currentTime;
            Serial.println(breathCount);
        }
    }

    // Update BPM every 60 seconds without blocking other code
    if (millis() - lastBPMUpdate >= bpmUpdateInterval) {
        bpmToDisplay = breathCount;  // Store BPM for display
        breathCount = 0;  // Reset count
        lastBPMUpdate = millis();  // Reset timer

        Serial.print("Breaths Per Minute (BPM): ");
        Serial.println(bpmToDisplay);
    }

    // Return BPM if available, otherwise return "reading"
    return (bpmToDisplay == -1) ? "..." : String(bpmToDisplay);
}

String detectAirQuality() {
    unsigned long currentMillis = millis();

    // Check if it's time to read the air quality sensor
    if (currentMillis - lastAirQualityRead >= airQualityInterval) {
        lastAirQualityRead = currentMillis;  // Update the last execution time
        airq = analogRead(airQuality);  // Read sensor value
    }
    return (airq == -1) ? "..." : String(airq) + " AQI";
}

void detectTemperatureHumidity() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousTempMillis >= tempInterval) {
        previousTempMillis = currentMillis;
        int chk = DHT.read11(DHT11_PIN);
        lastTemperature = DHT.temperature;  // Store last valid temperature
        lastHumidity = DHT.humidity;        // Store last valid humidity
    }
}

int getTemperature() {
    return lastTemperature;
}

int getHumidity() {
    return lastHumidity;
}

void detectDustDensity() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousDustMillis >= dustInterval) {
        previousDustMillis = currentMillis;

        digitalWrite(ledPower, LOW); // Turn on the LED
        delayMicroseconds(samplingTime);
        
        voMeasured = analogRead(measurePin); // Read the dust value
        delayMicroseconds(deltaTime);
        
        digitalWrite(ledPower, HIGH); // Turn off the LED
        delayMicroseconds(sleepTime);

        // Convert analog reading to voltage
        calcVoltage = voMeasured * (5.0 / 1024.0);
        dustDensity = 170 * calcVoltage - 0.1;  // Compute dust density
    }
}

float getDustDensity() {
    return dustDensity;
}

void detectHeartRateAndOxySat() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousHRMillis >= heartRateInterval) {
        previousHRMillis = currentMillis;

        long irValue = particleSensor.getIR();
        if (irValue < 50000) {
            // No finger detected
            Serial.println("No finger detected!");
            spo2 = -1;
            heartRate = -1;
            return;
        }

        bufferLength = 100; // Ensure buffer length is set

        // Read sensor data
        for (byte i = 0; i < bufferLength; i++) {
            while (!particleSensor.available())
                particleSensor.check();

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample();
        }

        // Calculate heart rate and SpO2
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

        // Shift buffer for new readings
        for (byte i = 25; i < bufferLength; i++) {
            redBuffer[i - 25] = redBuffer[i];
            irBuffer[i - 25] = irBuffer[i];
        }

        for (byte i = bufferLength - 25; i < bufferLength; i++) {
            while (!particleSensor.available())
                particleSensor.check();

            digitalWrite(readLED, !digitalRead(readLED));

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample();
        }

        // Display only valid results
        if (validHeartRate && heartRate > 0 && validSPO2 && spo2 > 0) {
            Serial.print("Heart Rate: ");
            Serial.print(heartRate);
            Serial.print(" bpm, SpO2: ");
            Serial.print(spo2);
            Serial.println(" %");
        } else {
            Serial.println("Invalid reading, please reposition finger.");
        }
    }
}

String getHeartRate() {
    return (validHeartRate && heartRate > 0) ? String(heartRate) : "...";
}

String getSpO2() {
    return (validSPO2 && spo2 > 0) ? String(spo2) : "...";
}

String detectAsthmaRisk(int air_quality, float dust_density, int temperature, int humidity, int heart_rate, int spo2, int breath_rate) {
    if (dust_density > 150) {
        return "High Risk: Dust Density";
        alert();
    } else if (air_quality > 126) {
        return "High Risk: Air Quality";
        alert();
    } else if (temperature < 23) {
        return "Moderate Risk: Low Temperature";
        alert();
    } else if (humidity < 30) {
        return "Moderate Risk: Low Humidity";
        alert();
    } else if (humidity > 70) {
        return "Moderate Risk: High Humidity";
        alert();
    } else if (heart_rate > 120 || heart_rate < 60) {
        return "Moderate Risk: Abnormal Heart Rate";
        alert();
    } else if (spo2 < 95) {
        return "High Risk: Low SpO2";
        alert();
    } else if (breath_rate > 25) {
        return "Moderate Risk: Abnormal Breathing Rate";
        alert();
    } else {
        return "No Risk Detected";
    }
}

void alert(){
  tone(BUZZER_PIN, 500); // Send 1KHz sound signal...
}

