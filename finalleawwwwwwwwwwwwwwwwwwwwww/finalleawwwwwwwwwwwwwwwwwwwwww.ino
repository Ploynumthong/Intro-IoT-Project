#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 5
#define greenled 11
#define yellowled 12
#define redled 13
#define DHTTYPE DHT11
#define WATERLV A1
#define raindrop A3
#define IRsensorPin 2

//ปั้ม
#define pump 7
//const unsigned long onehour = 3600000; //3600000
//unsigned long previous = 0;


// ค่าคงที่และตัวแปร
volatile int count = 0;
unsigned long previousMillis = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastSensorUpdate = 0;
const unsigned long lcdInterval = 500;      // อัพเดท LCD ทุก 0.5 วินาที
const unsigned long sensorInterval = 500;  // อัพเดทเซนเซอร์ทุก 2 วินาที
const unsigned long interval = 500;
float D = 0.1;
float C;  // ตัวแปรสำหรับความเร็วลม

// ค่าคงที่ของเซนเซอร์ MQ
int MQsensorPin = A0;
float R0 = 10.0, k = 1.0, n = -1.2;
float PPM;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);

// การตั้งค่า WiFi และ MQTT
const char* ssid = "Chaiyotha";
const char* password = "0922833497";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_Client = "Ploynumthong-13ff0552-6e46-4ca2-a472-6b455940034d";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[150];
String DataString;

// ฟังก์ชัน callback ของ MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];  // Convert payload to a String
  }
  
  Serial.println(message);
  Serial.println(topic);

  // Check for specific topic and message
  if (String(topic) == "Project/publish") {
    if (message == "suck") {
      digitalWrite(pump, LOW);
      Serial.println("Pump ON for 10 seconds");
      delay(10000);           // Keep the pump ON for 10 seconds
      digitalWrite(pump, HIGH);
      Serial.println("Pump OFF");
    }else if (message == "OFF"){
      digitalWrite(pump, HIGH);
    }else if(message == "ON"){
      digitalWrite(pump, LOW);
    }
  }
}


  // การเชื่อมต่อใหม่ของ MQTT
  void reconnect() {
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (client.connect(mqtt_Client)) {
        Serial.println("connected");
        client.subscribe("Project/publish");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
      }
    }
  }

  // การตั้งค่า WiFi
  void setupWiFi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(15);
      Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Setup
  void setup() {
    Serial.begin(9600);
    setupWiFi();
    dht.begin();
    lcd.begin();
    lcd.backlight();

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    pinMode(greenled, OUTPUT);
    pinMode(yellowled, OUTPUT);
    pinMode(redled, OUTPUT);
    pinMode(IRsensorPin, INPUT);
    pinMode(pump, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(IRsensorPin), countRpm, RISING);
  }

  // ส่งข้อมูลไปยัง MQTT
  void publishToMQTT(float temperature, float humidity) {
    int waterval = analogRead(WATERLV);
    int raindropval = analogRead(raindrop);

    DataString = "{\"data\":{\"temperature\":" + String(temperature) + ",\"humidity\":" + String(humidity) + ",\"waterval\":" + String(waterval) + ",\"raindropval\":" + String(raindropval) + ",\"PPM\":" + String(PPM) + ",\"C\":" + String(C) + "}}";

    DataString.toCharArray(msg, 150);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("project/subscribe", msg);
  }

  // แสดงข้อมูลบน LCD
  void displayOnLCD(float temperature, float humidity, float C, float PPM) {
    static float prevTemp = -1, prevHumidity = -1, prevWindSpeed = -1, prevPPM = -1;

    // อัพเดทเฉพาะเมื่อค่ามีการเปลี่ยนแปลง
    if (temperature != prevTemp || humidity != prevHumidity || C != prevWindSpeed || PPM != prevPPM) {

      lcd.clear();  // ล้างหน้าจอเพื่อลดปัญหาทับซ้อน

      lcd.setCursor(0, 0);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");

      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print(" C");

      lcd.setCursor(0, 2);
      lcd.print("Wind Spd: ");
      lcd.print(C);
      lcd.print(" m/s");

      lcd.setCursor(0, 3);
      lcd.print("PPM: ");
      lcd.print(PPM);


      prevTemp = temperature;
      prevHumidity = humidity;
      prevWindSpeed = C;
      prevPPM = PPM;
    }
  }

  // Main loop
  void loop() {
    unsigned long now = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      lcd.clear();
      lcd.print("......");
      delay(2000);
      return;
    }

    // อัพเดทการแสดงผลบน LCD ทุก 0.5 วินาที
    if (now - lastLcdUpdate >= lcdInterval) {
      lastLcdUpdate = now;
      displayOnLCD(t, h, C, PPM);
    }

    // ส่งข้อมูลไปยัง MQTT ทุกๆ 10 วินาที
    if (now - lastMsg >= 1000) {
      lastMsg = now;
      publishToMQTT(t, h);
    }

    // ตรวจสอบและคงการเชื่อมต่อกับ MQTT
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    // อัพเดทค่าการอ่านจากเซนเซอร์ทุกๆ 2 วินาที
    if (now - lastSensorUpdate >= sensorInterval) {
      lastSensorUpdate = now;
      Windspeed();
      PPMvalue();
      waterLevelLED();
    }
  }

  // คำนวณความเร็วลม
  void Windspeed() {
    if (millis() - previousMillis >= interval) {
      previousMillis = millis();
      float R = count * 60;
      count = 0;
      C = (3.14159 * D * R) / 60;

      Serial.print("RPM: ");
      Serial.print(R);
      Serial.print(" | Wind Speed: ");
      Serial.print(C);
      Serial.println(" m/s");
      return;
    }
  }

  // คำนวณ PPM
  void PPMvalue() {
    int sensorValue = analogRead(MQsensorPin);
    float Vout = sensorValue * (5.0 / 1023.0);
    float RS = (5.0 - Vout) / Vout * R0;
    PPM = k * pow((R0 / RS), n);

    Serial.print("Vout: ");
    Serial.print(Vout);
    Serial.print(" V, RS: ");
    Serial.print(RS);
    Serial.print(" kΩ, PPM: ");
    Serial.println(PPM);
    return;
  }

  

  // กำหนดไฟ LED แสดงระดับน้ำ
  void waterLevelLED() {
    int waterval = analogRead(WATERLV);
    Serial.print(F("Water Level: "));
    Serial.println(waterval);
    /*if (waterval <= 399) {
      digitalWrite(greenled, HIGH);
      digitalWrite(yellowled, LOW);
      digitalWrite(redled, LOW);
    } else if (waterval >= 400 && waterval <= 599) {
      digitalWrite(greenled, LOW);
      digitalWrite(yellowled, HIGH);
      digitalWrite(redled, LOW);
    } else {
      digitalWrite(greenled, LOW);
      digitalWrite(yellowled, LOW);
      digitalWrite(redled, HIGH);
    }*/
    return;
  }



  // Interrupt สำหรับการนับ RPM
  void countRpm() {
    count++;
  }