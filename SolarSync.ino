#define V_SENSOR_PIN 32
#define A_SENSOR_PIN 33
#define LDR_SENSOR_TOP_PIN 39
#define LDR_SENSOR_BOTTOM_PIN 36
#define LDR_SENSOR_LEFT_PIN 35
#define LDR_SENSOR_RIGHT_PIN 34
#define BATTERY_PIN 25
#define SERVO_180_PIN 19
#define SERVO_360_PIN 18
#define BAT_PIN 25

#include <cmath>
#include <ArduinoJson.h>

// SETUP OLED_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3c
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool isDisplay_on = true;

// SETUP and ADJ Servo
#include <ESP32Servo.h>
Servo myServo180;
Servo myServo360;
const int ANGLE_MAX = 115;
const int ANGLE_MIN = 55;
int angle = ANGLE_MIN;

#include <WiFi.h>
#include <PubSubClient.h>
#define MQTT_SERVER "20.205.129.163"
#define MQTT_PORT 1883
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_NAME "ESP32_SolarSync"
WiFiClient client;
PubSubClient mqtt(client);
const char *ssid = "Perth11T";
const char *password = "208902546";

// Battery Variable
const float V_min = 3.0;
const float V_max = 4.2;

struct Sensor_Data
{
  float raw;
  float con;
};

int LDR_TOP_VALUE = 0;
int LDR_BOT_VALUE = 0;
int LDR_RIG_VALUE = 0;
int LDR_LEF_VALUE = 0;

// For miltitasking
unsigned long current_time = 0;
unsigned long current_time_servo180 = 0;
unsigned long current_time_framerate = 0;
unsigned long current_time_servo360 = 0;
unsigned long current_time_idle = 0;

struct Sensor_Data getVoltage()
{
  struct Sensor_Data Voltage;
  float con_v = 0, con_avg;
  int raw, raw_v = 0, raw_avg, time = 100;
  for (int i = 0; i < time; i++)
  {
    raw = analogRead(V_SENSOR_PIN);
    raw_v += raw;
    // (raw_v / 12 bit) * v_ref * voltageDivider * error_correction
    con_v += 3.3 + (5.0 - 3.3) / (1057 - 686) * (raw - 686);
  }
  raw_avg = raw_v / time;
  con_avg = con_v / time;
  // Serial.printf("RAW_V : %5d\t Voltage : %6.2f\n", raw_avg, con_avg);

  Voltage.raw = raw_avg;
  Voltage.con = con_avg;
  return Voltage;
}

struct Sensor_Data getCurrent()
{
  struct Sensor_Data Current;
  const float Vref = 3.3;          // แรงดันอ้างอิงของ ESP32
  const int ADC_RESOLUTION = 4096; // ความละเอียด 12-bit ของ ESP32
  // กำหนดค่าเซ็นเซอร์ ACS712 รุ่นที่ใช้ (เช่น รุ่น 5A มี Sensitivity 185 mV/A)
  const float sensitivity = 0.185; // สำหรับ ACS712 รุ่น 5A: 185 mV/A
  const float offset = 1.62;
  float con_c = 0, con_avg;
  int raw, raw_c = 0, raw_avg, time = 100;
  for (int i = 0; i < time; i++)
  {
    raw = analogRead(A_SENSOR_PIN);
    raw_c += raw;
    float v = (raw * Vref) / ADC_RESOLUTION;
    con_c += (v - (Vref / 2)) / sensitivity + offset;
  }
  raw_avg = raw_c / time;
  con_avg = con_c / time;
  // Serial.printf("RAW_C : %5d\t Current : %6.2f\n", raw_avg, con_avg);

  Current.raw = raw_avg;
  Current.con = con_avg;
  return Current;
}

float getWatt()
{
  float voltage, current;
  voltage = getVoltage().con;
  current = getCurrent().con;
  return voltage * current;
}

int get_battery_precentage()
{
  int adcValue = analogRead(BAT_PIN);
  float V_adc = adcValue * (3.3 / 4095.0); // แปลงค่าจาก ADC เป็นแรงดันไฟฟ้า
  float V_batt = V_adc * 2;                // เนื่องจากวงจรแบ่งแรงดัน
  float batteryPercentage = ((V_batt - V_min) / (V_max - V_min)) * 100;
  // ตรวจสอบไม่ให้เปอร์เซ็นต์เกิน 100% หรือ ต่ำกว่า 0%
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  return batteryPercentage;
}

void Servo360_spin_right()
{
  myServo360.write(0);
}
void Servo360_spin_left()
{
  myServo360.write(180);
}
void Servo360_stop()
{
  myServo360.write(90);
}

bool test_LDR(int analog_Value)
{
  Serial.printf("[TESTING] LDR Value : %4d", analog_Value);
  if (analog_Value > 0 && analog_Value < 4095)
  {
    Serial.printf("\t[RESULT] LDR : PASS\n");
    return true;
  }
  else
  {
    Serial.printf("\t[RESULT] LDR : FAIL\n");
    return false;
  }
}

void display_reboot(char str_cause[] = "something worng!.", int delay_ms = 3000)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F(" Rebooting"));
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.printf("Cos: %s", str_cause);
  display.display();
  Serial.printf("[REBOOTING][%dms] %s\n", delay_ms, str_cause);
  delay(delay_ms);
}

void display_logo()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F(" SolarSync"));
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.printf(" [PPP] PerthPhuPhoom");
  display.display();
}

void display_main()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("SolarSync");
  display.setCursor(65, 0);
  display.printf("[");
  int battery = ceil(get_battery_precentage() / 25.0);
  for (int i = 0; i < 4; i++)
  {
    if (i < battery)
    {
      display.printf("I");
    }
    else
    {
      display.printf(" ");
    }
  }
  display.printf("]%3d%%", get_battery_precentage());
  display.setCursor(0, 10);
  display.printf("%5.1fV %5.1fmA %5.1fW", getVoltage().con, getCurrent().con, getWatt());
  display.setCursor(0, 20);
  display.printf("DEG: %2d", angle);
  display.display();
}

void check_all_LDR()
{
  bool top = false, bot = false, right = false, left = false;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Checking LDR ..."));
  display.display();
  delay(1000);
  for (int i = 0; i <= 5; i++)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Checking LDR"));
    display.display();
    display.setCursor(0, 10);
    display.println(F("TOP : "));
    display.setCursor(50, 10);
    display.println(F("BOT : "));
    display.setCursor(0, 20);
    display.println(F("RIG : "));
    display.setCursor(50, 20);
    display.println(F("LEF : "));
    display.display();
    if (!test_LDR(analogRead(LDR_SENSOR_TOP_PIN)))
    {
      display.setCursor(30, 10);
      display.println(F("F"));
      display.display();
      top = false;
    }
    else
    {
      display.setCursor(30, 10);
      display.println(F("P"));
      display.display();
      top = true;
    }
    if (!test_LDR(analogRead(LDR_SENSOR_BOTTOM_PIN)))
    {
      display.setCursor(80, 10);
      display.println(F("F"));
      display.display();
      bot = false;
    }
    else
    {
      display.setCursor(80, 10);
      display.println(F("P"));
      display.display();
      bot = true;
    }
    if (!test_LDR(analogRead(LDR_SENSOR_RIGHT_PIN)))
    {
      display.setCursor(30, 20);
      display.println(F("F"));
      display.display();
      right = false;
    }
    else
    {
      display.setCursor(30, 20);
      display.println(F("P"));
      display.display();
      right = true;
    }
    if (!test_LDR(analogRead(LDR_SENSOR_LEFT_PIN)))
    {
      display.setCursor(80, 20);
      display.println(F("F"));
      display.display();
      left = false;
    }
    else
    {
      display.setCursor(80, 20);
      display.println(F("P"));
      display.display();
      left = true;
    }
    delay(500);
    if (top && bot && right && left)
    {
      Serial.println("Pass");
      display.setCursor(80, 0);
      display.printf("pass");
      display.display();
      delay(1000);
      break;
    }
    else if (i + 1 == 6)
    {
      display.setCursor(80, 0);
      display.printf("Fail !!");
      display.display();
      delay(2000);
      display_reboot("Some LDR not    working.");
      ESP.restart();
    }
    else
    {
      Serial.printf("Not Pass try again %d\n", i + 1);
      display.setCursor(80, 0);
      display.printf("try %d", i + 1);
      display.display();
      delay(500);
    }
  }
}

void check_all_Servo()
{
  int temp_LDR_TOP = 0, temp_LDR_BOT = 0;
  int error_count = 0;
  int angle;
  // Checking Servo 180
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Checking Servo ...\n");
  display.display();
  delay(1000);
  for (angle = 0; angle <= 150; angle++)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("Test Servo 180");
    display.setCursor(0, 10);
    display.printf("writing angle : ");
    display.setCursor(0, 20);
    display.printf("error_count : ");
    display.display();
    display.setCursor(90, 0);
    for (int i = 0; i < angle % 4; i++)
    {
      display.printf(".");
      display.display();
    }
    display.setCursor(100, 10);
    display.printf("      ");
    display.setCursor(100, 10);
    display.printf("%d", angle);
    myServo180.write(angle);
    Serial.printf("Now angle : %d\n", angle);
    delay(8);
    if (analogRead(LDR_SENSOR_BOTTOM_PIN) - temp_LDR_BOT <= 2 && analogRead(LDR_SENSOR_TOP_PIN) - temp_LDR_TOP <= 2)
    {
      error_count++;
      Serial.printf("[ERR] Light is not Change attempt : %d\n", error_count);
    }
    display.setCursor(100, 20);
    display.printf("      ");
    display.setCursor(100, 20);
    display.printf("%d", error_count);
    display.display();
    temp_LDR_BOT = analogRead(LDR_SENSOR_BOTTOM_PIN);
    temp_LDR_TOP = analogRead(LDR_SENSOR_TOP_PIN);
  }
  for (angle = 150 - 1; angle >= ANGLE_MIN; angle--)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("Test Servo 180");
    display.setCursor(0, 10);
    display.printf("writing angle : ");
    display.setCursor(0, 20);
    display.printf("error_count : ");
    display.display();
    display.setCursor(90, 0);
    for (int i = 0; i < angle % 4; i++)
    {
      display.printf(".");
      display.display();
    }
    display.setCursor(100, 10);
    display.printf("      ");
    display.setCursor(100, 10);
    display.printf("%d", angle);
    myServo180.write(angle);
    Serial.printf("Now angle : %d\n", angle);
    delay(8);
    if (analogRead(LDR_SENSOR_BOTTOM_PIN) - temp_LDR_BOT <= 2 && analogRead(LDR_SENSOR_TOP_PIN) - temp_LDR_TOP <= 2)
    {
      error_count++;
      Serial.printf("[ERR] Light is not Change attempt : %d\n", error_count);
    }
    display.setCursor(100, 20);
    display.printf("      ");
    display.setCursor(100, 20);
    display.printf("%d", error_count);
    display.display();
    temp_LDR_BOT = analogRead(LDR_SENSOR_BOTTOM_PIN);
    temp_LDR_TOP = analogRead(LDR_SENSOR_TOP_PIN);
  }
  if (error_count <= 130)
  {
    Serial.printf("[RESULT][PASS] servo180\n");
    display.setCursor(90, 0);
    display.printf("PASS");
    display.display();
    delay(2000);
  }
  else
  {
    Serial.printf("[RESULT][FAIL] servo180\n");
    display.setCursor(90, 0);
    display.printf("FAIL!!");
    display.display();
    delay(2000);
    display_reboot("Servo(180) is   not working");
    ESP.restart();
  }

  // Checking Servo 360
  int temp_LDR_right = 0, temp_LDR_left = 0;
  error_count = 0;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Test Servo 360 ...\n");
  display.display();
  Serial.printf("[TESTING] Servo 360 spin *LEFT* until reaching the limit\n");
  delay(1000);
  temp_LDR_right = analogRead(LDR_SENSOR_RIGHT_PIN);
  temp_LDR_left = analogRead(LDR_SENSOR_LEFT_PIN);
  Servo360_spin_left();
  delay(2000);
  Servo360_stop();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Test Servo 360 ");
  if (analogRead(LDR_SENSOR_RIGHT_PIN) - temp_LDR_right <= 2 && analogRead(LDR_SENSOR_LEFT_PIN) - temp_LDR_left <= 2)
  {
    error_count++;
    Serial.printf("[ERR] Light is not Change attempt : %d\n", error_count);
  }
  display.setCursor(0, 20);
  display.printf("error_count = %4d", error_count);
  display.display();
  Serial.printf("[TESTING] Servo 360 spin *RIGHT* until reaching the limit\n");
  delay(1000);
  Servo360_spin_right();
  delay(2000);
  Servo360_stop();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Test Servo 360 ");
  if (analogRead(LDR_SENSOR_RIGHT_PIN) - temp_LDR_right <= 2 && analogRead(LDR_SENSOR_LEFT_PIN) - temp_LDR_left <= 2)
  {
    error_count++;
    Serial.printf("[ERR] Light is not Change attempt : %d\n", error_count);
  }
  display.setCursor(0, 20);
  display.printf("error_count = %4d", error_count);
  display.display();
  if (error_count < 2)
  {
    Serial.printf("[RESULT][PASS] servo360\n");
    display.setCursor(90, 0);
    display.printf("PASS");
    display.display();
    delay(2000);
  }
  else
  {
    Serial.printf("[RESULT][FAIL] servo360\n");
    display.setCursor(90, 0);
    display.printf("FAIL!!");
    display.display();
    delay(2000);
    display_reboot("Servo(360) is   not working");
    // ESP.restart();
  }
  Serial.printf("[WARNING][SAVE] servo360 now_round save to EEPROM at 0x000\n");
}

void check_all_sensor()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  check_all_LDR();
  check_all_Servo();
}

void setup()
{
  Serial.begin(115200);
  Serial.print("By Solar Sync");
  myServo180.attach(SERVO_180_PIN);
  myServo360.attach(SERVO_360_PIN);
  Wire.begin();

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
  // Serial.println("WiFi connected");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());

  // mqtt.setServer(MQTT_SERVER, MQTT_PORT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println("SSD1306 allocation failed");
    display_reboot("SSD1306 allocation failed");
    ESP.restart();
  }
  display.display();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display_logo();
  delay(2000);
  check_all_sensor();
  display_main();
}

void loop()
{
  // if (mqtt.connected() == false) {
  //   Serial.print("MQTT connection... ");
  //   if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
  //     Serial.println("connected");
  //     mqtt.subscribe("ESP32_SolarSync");
  //   } else {
  //     Serial.println("failed");
  //     delay(5000);
  //   }
  // } else {
  //   mqtt.loop();
  // }
  // for servo180
  if (millis() - current_time_servo180 >= 20)
  {
    LDR_TOP_VALUE = analogRead(LDR_SENSOR_TOP_PIN);
    LDR_BOT_VALUE = analogRead(LDR_SENSOR_BOTTOM_PIN);
    current_time_servo180 = millis();
    if (LDR_BOT_VALUE - LDR_TOP_VALUE >= 75)
    {
      current_time_idle = millis();
      if (angle + 1 <= ANGLE_MAX)
      {
        angle += 1;
      }
      myServo180.write(angle);
    }
    else if (LDR_TOP_VALUE - LDR_BOT_VALUE >= 75)
    {
      current_time_idle = millis();
      if (angle - 1 >= ANGLE_MIN)
      {
        angle -= 1;
      }
      myServo180.write(angle);
    }
  }
  // for servo360
  if (millis() - current_time_servo360 >= 20)
  {
    LDR_RIG_VALUE = analogRead(LDR_SENSOR_RIGHT_PIN);
    LDR_LEF_VALUE = analogRead(LDR_SENSOR_LEFT_PIN);
    current_time_servo360 = millis();
    if (LDR_RIG_VALUE - LDR_LEF_VALUE >= 75)
    {
      current_time_idle = millis();
      Servo360_spin_left();
      delay(50);
      // Servo360_stop();
    }
    else if (LDR_LEF_VALUE - LDR_RIG_VALUE >= 75)
    {
      current_time_idle = millis();
      Servo360_spin_right();
      delay(50);
      // Servo360_stop();
    }
    else
    {
      Servo360_stop();
    }
  }
  if (millis() - current_time_framerate >= 100)
  {
    current_time_framerate = millis();
    display_main();
  }

  // for Serial Monitor
  if (millis() - current_time >= 1000)
  {
    current_time = millis();
    getVoltage();
    getCurrent();

    Serial.printf("LDR_TOP : %4d\tLDR_BOT : %4d\nLDR_RIG : %4d\tLDR_LEF : %4d\n", LDR_TOP_VALUE, LDR_BOT_VALUE, LDR_RIG_VALUE, LDR_LEF_VALUE);
    Serial.printf("now_angle : %4d\n\n", angle);

    // StaticJsonDocument<200> doc;
    // doc["LDR_TOP"] = LDR_TOP_VALUE;
    // doc["LDR_BOT"] = LDR_BOT_VALUE;
    // doc["LDR_RIG"] = LDR_RIG_VALUE;
    // doc["LDR_LEF"] = LDR_LEF_VALUE;
    // doc["Voltage"] = getVoltage().con;
    // doc["Current"] = getCurrent().con;
    // doc["Battery_percentage"] = get_battery_precentage();
    // doc["Battery_voltage"] = analogRead(BAT_PIN);
    // doc["Angle"] = angle;
    // char jsonBuffer[512];
    // serializeJson(doc, jsonBuffer);

    // Serial.print("Publishing JSON message: ");
    // Serial.println(jsonBuffer);

    // // ส่งข้อมูล JSON ไปยัง topic "test/json_topic"
    // mqtt.publish(MQTT_NAME, jsonBuffer);
  }
}
