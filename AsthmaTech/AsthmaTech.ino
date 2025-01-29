//SD CARD
#include <SPI.h>
#include <SD.h>
const int chipSelect = 4; // Pin for the SD card module (usually 4 or 10 for most modules)


//OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire);


//AIR QUALITY
const int airQuality = A0;
int threshold = 250;


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
dht DHT; //create a variable type of dht
const int DHT11_PIN= 2; //Humiture sensor attach to pin2


void setup() {
  Serial.begin(9600);


  while (!Serial) {
    // Wait for Serial Monitor to open
  }
  // AIR QUALITY
  pinMode(airQuality, INPUT);


  // SD CARD
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");


  // Create or open a log file
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("data logging..."); // Add headers
    dataFile.close();
    Serial.println("Headers written to datalog.txt");
  } else {
    Serial.println("Failed to open datalog.txt");
  }


  //DUST DENSITY
  pinMode(ledPower, OUTPUT);
}


void loop() {
  //AIR QUALITY
  int ppm = analogRead(airQuality); //reads the sensor value
 
  //DUST DENSITY
  digitalWrite(ledPower, LOW); // power on the LED
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); // read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);
  calcVoltage = voMeasured * (5.0 / 1024.0); // 0-5V mapped to 0-1023 integer value
  dustDensity = 170 * calcVoltage - 0.1;


  //TEMPERATURE AND HUMIDITY
  int chk = DHT.read11(DHT11_PIN); //read the value returned from sensor


  //oledDisplay(ppm, dustDensity, DHT.temperature, DHT.humidity);
  sdCardTransfer(ppm, dustDensity, DHT.temperature, DHT.humidity);
  delay(2000);
}


void sdCardTransfer(int airQuality, int dustDensity, int temperature, int humidity) {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);


  if (dataFile) {
    // Log data to the SD card
    dataFile.print("Air Quality: ");
    dataFile.println(airQuality);
    dataFile.print("Dust Density: ");
    dataFile.println(dustDensity);
    dataFile.print("Temperature: ");
    dataFile.println(temperature);
    dataFile.print("Humidity: ");
    dataFile.println(humidity);
    dataFile.print(" ");
    dataFile.close();
    Serial.print(airQuality);
    Serial.print(dustDensity);
    Serial.print(temperature);
    Serial.print(humidity);
  } else {
    Serial.println("Failed to open datalog.txt for writing.");
  }
}


/*
void oledDisplay(int airQuality, int dustDensity, int temperature, int humidity) {
  // AIR QUALITY
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 1);
  display.print("Air Quality: ");
  display.println(airQuality);
  display.display();
  if (airQuality > threshold) {
    display.setCursor(100,1);
    display.println("BAD");
    display.display();
  }
  else {
    display.setCursor(100,1);
    display.println("GOOD");
    display.display();
  }


  //DUST DENSITY
  display.setCursor(0,15);
  display.print("Dust Density: ");
  display.println(dustDensity);
  display.display();


  //TEMPERATURE AND HUMIDITY
  display.setCursor(0, 30);
  display.print("Temperature:");
  display.println(temperature); //1 is the decimal place
  display.display();


  display.setCursor(0, 45);
  display.print("Humidity:");
  display.println(humidity);
  display.display();
}*/



