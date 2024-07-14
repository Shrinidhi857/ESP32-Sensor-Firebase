//DHT11 - temp and humidity sensor
//LM393 - sound sensor
//LM393 - speed sensor
//Sw420 - vibration sensor

// Libraries
#include <WiFi.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define Wifi_SSID "WIFI_ID"
#define Wifi_pass "WIFI_PASS"
#define API_KEY "API_KEY"
#define DATABASE_URL "FIREBASE_URL"

// Define pins
#define SENSOR_PIN 2 //speed sensor
#define soundSensorPin 34 //sound sensor
#define vibrationSensorPin 25 //vibration sensor

// Constants
const float referenceVoltage = 3.3;
const int measurementInterval = 2000; // 2 second interval for measurement

// DHT sensor setup
DHT dht(4, DHT11);

// Firebase setup
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;
unsigned long sendDataPrevMillis = 0;

// Variables for RPM measurement
volatile unsigned long count = 0;
unsigned long prevMillis = 0;
unsigned long rpm = 0;

// Variables for frequency measurement
volatile unsigned long pulseCount = 0;
unsigned long previousTime = 0;
float frequency = 0;
unsigned long currentTime = millis();

// Functions
void setup() {
    Serial.begin(115200);
    
    // Sensor setup
    pinMode(SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, RISING);
    
    pinMode(vibrationSensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(vibrationSensorPin), countPulseFrequency, RISING);
    
    count = 0;
    prevMillis = millis();
    rpm = 0;
    frequency = 0;
    pulseCount = 0;
    previousTime = millis();
    
    WIFI();
    DataBaseAuth();
    checklogin();
}

void loop() {
    updateRPM();
    measureFrequency();
    DHT11TH();
    calculateAmplitude();
    
    int sensorValue = analogRead(soundSensorPin);
    float voltage = sensorValue * (referenceVoltage / 4095.0);
    calculateDecibels(voltage);
    
    delay(2000);
}

//RPM measurement
void countPulse() {
    count++;
}

//frequency measurement
void countPulseFrequency() {
    pulseCount++;
}

// Update rpm measurement & send to Firebase
void updateRPM() {
    if (millis() - prevMillis >= 1000) {
        rpm = (count * 60000UL) / (millis() - prevMillis);
        count = 0;
        prevMillis = millis();
        
        if (Firebase.RTDB.setInt(&fbdo, "sensor2/Rpm", rpm))
            Serial.println("RPM data sent to Firebase");
        else
            Serial.println("Failed to send RPM data to Firebase");
    }
}

// Measure frequency of vibration sensor and send to Firebase
void measureFrequency() {
    currentTime = millis();
    if (currentTime - previousTime >= measurementInterval) {
        // Calculate frequency in Hz
        frequency = pulseCount / ((currentTime - previousTime) / 1000.0);
        
        // Reset the pulse count and update the previous time
        pulseCount = 0;
        previousTime = currentTime;
        
        // Send frequency data to Firebase
        if (Firebase.RTDB.setFloat(&fbdo, "sensor2/Frequency", frequency)) {
            Serial.println("Frequency data sent to Firebase");
        } else {
            Serial.println("Failed to send frequency data to Firebase");
        }
    }
}

// Function to calculate amplitude based on pulse duration
void calculateAmplitude() {
    // Measure the pulse width (in microseconds)
    unsigned long pulseDuration = pulseIn(vibrationSensorPin, HIGH);

    // Calculate amplitude based on pulse duration
    float amplitude = pulseDuration / 1000.0; // Convert to milliseconds

    // Send amplitude data to Firebase
    if (Firebase.RTDB.setFloat(&fbdo, "sensor2/Amplitude", amplitude)) {
        Serial.println("Amplitude data sent to Firebase");
    } else {
        Serial.println("Failed to send amplitude data to Firebase");
    }

    Serial.print("Pulse duration (HIGH): ");
    Serial.print(pulseDuration);
    Serial.print(" us, Calculated amplitude: ");
    Serial.println(amplitude);
}

// DHT11 temperature and humidity data
void DHT11TH() {
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    if (Firebase.RTDB.setFloat(&fbdo, "sensor2/temperature_data", temp)) {
        Serial.println("Temperature data sent to Firebase");
    } else {
        Serial.println("Failed to send temperature data to Firebase");
    }
    
    if (Firebase.RTDB.setFloat(&fbdo, "sensor2/Humidity", humidity)) {
        Serial.println("Humidity data sent to Firebase");
    } else {
        Serial.println("Failed to send humidity data to Firebase");
    }
}

// Authentication function
void DataBaseAuth() {
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    
    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("SignUp OK");
        signupOK = true;
    } else {
        Serial.printf("%s\n", config.signer.signupError.message.c_str());
    }
    
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

// Check login status function
void checklogin() {
    if (!(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0))) {
        Serial.println("Failed: " + fbdo.errorReason());
    }
}

// Wi-Fi connection function
void WIFI() {
    WiFi.begin(Wifi_SSID, Wifi_pass);
    
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
}

// Function to calculate decibels and send to Firebase
void calculateDecibels(float voltage) {
    float sensitivity = 0.1; // Sensor sensitivity (adjust as needed)
    float referenceVoltage = 5.0; // Reference voltage of LM393 (usually 5V)
    float voltageAtThreshold = 0.05; // Voltage at the threshold of detection (adjust as needed)bb
    float voltageRatio = voltage / referenceVoltage;
    // Calculate decibels
    float decibels = 20 * log10(voltageRatio / sensitivity / voltageAtThreshold);
    
    // Send decibels data to Firebase
    if (Firebase.RTDB.setFloat(&fbdo, "sensor2/sound_dB", decibels)) {
        Serial.println("Decibels data sent to Firebase");
    } else {
        Serial.println("Failed to send decibels data to Firebase");
    }
}
