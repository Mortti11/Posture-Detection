#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

//#include <WiFi.h>
///#include <PubSubClient.h>

// Replace the next variables with your SSID/Password combination
//const char* ssid ="IOTLABRA";
//const char* password = "iotlabra2020";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
//const char* mqtt_server = "172.16.200.39";

//WiFiClient espClient;
//PubSubClient client(espClient);
//long lastMsg = 0;
//char msg[50];
//int value = 0;

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// Constants for inactivity timer
const unsigned long INACTIVITY_THRESHOLD = 5000; // 5 seconds for testing
unsigned long inactivityTimer = 0;       // Tracks how long the user has been inactive
unsigned long lastInactivityMessageTime = 0; // Tracks the last time inactivity message was sent
const unsigned long INACTIVITY_MESSAGE_INTERVAL = 5000; // Interval for repeated inactivity messages in ms

// Variables for step counting
int stepCount = 0;          // Tracks the number of steps taken
unsigned long lastStepTime = 0;
const unsigned long STEP_DEBOUNCE_TIME = 200; // Minimum time between steps in ms

// Moving average filter for magnitude smoothing
#define WINDOW_SIZE 10
float magnitudeBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;

float calculateMovingAverage(float newValue) {
  magnitudeBuffer[bufferIndex] = newValue;
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum += magnitudeBuffer[i];
  }
  return sum / WINDOW_SIZE;
}

void setup(void) {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Wait until Serial is ready
  
  // Initialize I2C communication on specific GPIO pins
  Wire.begin(16, 17); // SDA on GPIO 16, SCL on GPIO 17
  
  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10); // Stop program if sensor initialization fails
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer and gyroscope ranges and filter bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  //setup_wifi();
  //client.setServer(mqtt_server, 1883);
  //client.setCallback(callback);

  Serial.println("Setup complete.");
  delay(100);
}

//void setup_wifi() {
  //delay(10);
  // We start by connecting to a WiFi network
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);

  //WiFi.begin(ssid, password);

  //while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
   // Serial.print(".");
 // }

  //Serial.println("");
  //Serial.println("WiFi connected");
 // Serial.println("IP address: ");
 // Serial.println(WiFi.localIP());
//}

//void callback(char* topic, byte* message, unsigned int length) {
//  Serial.print("Message arrived on topic: ");
//  Serial.print(topic);
// Serial.print(". Message: ");
// String messageTemp;
  
 // for (int i = 0; i < length; i++) {
 //   Serial.print((char)message[i]);
 //   messageTemp += (char)message[i];
  //}
 // Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  //if (String(topic) == "test/button_press") {
   // Serial.print("Changing output to ");
    //if(messageTemp == "on"){
     /// Serial.println("on");
   // }
   // else if(messageTemp == "off"){
   //   Serial.println("off");
  //  }
 // }
//}

//void reconnect() {
  // Loop until we're reconnected
 // while (!client.connected()) {
   // Serial.print("Attempting MQTT connection...");
    // Attempt to connect
   // if (client.connect("ESP8266Client")) {
   //   Serial.println("connected");
      // Subscribe
   //   client.subscribe("esp32/output");
   // } else {
   //   Serial.print("failed, rc=");
   //   Serial.print(client.state());
    //  Serial.println(" try again in 5 seconds");
   //   // Wait 5 seconds before retrying
   //   delay(5000);
   // }
  //}
//}

void loop() {

  //if (!client.connected()) {
   // reconnect();
  //}
  //client.loop();

  // Read sensor data for acceleration, gyroscope, and temperature
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate magnitude of acceleration vector
  float rawMagnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
  float magnitude = calculateMovingAverage(rawMagnitude);


  // Inactivity detection: Check if the sensor detects minimal movement
  if (magnitude >= 11.7 && magnitude <= 12.2) {
    // If magnitude is within range (indicating low movement), start or continue inactivity timer
    if (inactivityTimer == 0) {
      inactivityTimer = millis(); // Start inactivity timer
    }

    // If inactivity exceeds the threshold, print inactivity message periodically
    if ((millis() - inactivityTimer) >= INACTIVITY_THRESHOLD) {
      if ((millis() - lastInactivityMessageTime) >= INACTIVITY_MESSAGE_INTERVAL) {
        Serial.println("Inactivity detected. Please walk.");
        lastInactivityMessageTime = millis(); // Update last inactivity message time
      }
    }
  } else {
    // Movement detected, reset inactivity timers
    inactivityTimer = 0;
    lastInactivityMessageTime = 0;
  }

   // Debugging: Print inactivity timer
  Serial.print("Inactivity Timer: ");
  Serial.println(inactivityTimer);

   // Step detection with debounce logic and peak detection
  static float previousMagnitude = 0;
  if (magnitude > 12.2 && previousMagnitude < magnitude && (millis() - lastStepTime) > STEP_DEBOUNCE_TIME) { // Increased threshold for steps
    stepCount++;
    lastStepTime = millis();
    Serial.print("Step detected! Total steps: ");
    Serial.println(stepCount);
  }
  previousMagnitude = magnitude;

 
  // Debugging: Print raw and filtered magnitude
  Serial.print("Raw Magnitude: ");
  Serial.println(rawMagnitude);
  Serial.print("Filtered Magnitude: ");
  Serial.println(magnitude);

  // Debugging: Print temperature
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  delay(500); // Sampling delay
}
