#include <SPI.h>
#include <RadioLib.h>
#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <PCF8575.h>

/**************************************/
/*            GPIO define             */
/**************************************/
#define RL_NI1 27
#define RL_NI2 32
#define RL_NI3 33

#define SW_Auto 26
#define SW_Man 25

#define SW_INOUT 12
#define SW_ONOFF 13
#define SW_TOGGLE 35
// #define Buzzer 23

#define AC_LOAD 4            // Ouput load AC
#define zero_cross_detect 34 // Input detect

// int bz;

const int NSS_PIN = 5;
const int RST_PIN = 14;
const int DIO0_PIN = 15; // IRQ (Interrupt pin)

volatile bool loraPacketReceived = false;
volatile bool loraActive = false; // ตัวแปรสำหรับควบคุม LoRa

/**************************************/
/*       Constand define value        */
/**************************************/
// 20 seconds WDT
#define WDT_TIMEOUT 20
volatile bool zero_cross = 0;

bool _flag; // ตัวแปลสำหรับเช็คการทำงาน

int lastCase = -1; 

/**************************************/
/*        define global variable      */
/**************************************/
String Percentage_input;
int percentage;
float time_delay_us = 0;

int power;        // ค่ากำลังที่ขับมอเตอร์พัดลม
int state;        // ตัวแปรสำหรับเก็บสถานะการทำงานของสวิตช์ IN-OUT
int state_onoff;  // ตัวแปรสำหรับเก็บสถานะการทำงานของสวิตช์ ON-OFF
int state_toggle; // ตัวแปรสำหรับเก็บสถานะการทำงานของสวิตช์ TOGGLE

// LoRa command variable
volatile int loraCommand = -1;
volatile bool newLoRaCommand = false;

Module module(NSS_PIN, DIO0_PIN, RST_PIN);
SX1278 lora(&module);

PCF8575 pcf8575(0x20);

static bool blinkEnable = true;
static unsigned long lastBlink = 0;
static bool ledState = false;

/**************************************/
/*           define function          */
/**************************************/
void zero_crosss_int();
float PERCENT_to_TIME(int value);

void start();
void _off();
void _off_2();
void _ledblink();
void beepPCF8575();
void beepPCF8575_2();

/**************************************/
/*             mutitask               */
/**************************************/
TaskHandle_t dimmer_Task = NULL;
TaskHandle_t lora_Task = NULL;

void dimmer_w_Task(void *p);
void lora_Task_Handler(void *p);

/*****************************************/
/*  Interrupt Service Routine (ISR)      */
/*****************************************/
void IRAM_ATTR setLoRaFlag()
{
  loraPacketReceived = true;
  // // Serial.println("[ISR] LoRa packet interrupt!");

  // static unsigned long lastInterrupt = 0;
  //   unsigned long now = millis();
    
  //   // Debounce interrupt - ป้องกันการเรียกซ้ำใน 50ms
  //   if (now - lastInterrupt > 50) {
  //       loraPacketReceived = true;
  //       lastInterrupt = now;
  //   }
}

/***********************************************************************
 * FUNCTION:    setup
 * DESCRIPTION: setup process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);

  Wire.begin(21, 22);

  // ตรวจสอบว่าสามารถเชื่อมต่อกับ PCF8575 ได้หรือไม่
  if (pcf8575.begin())
  {
    Serial.println("PCF8575 is ready!");
  }
  else
  {
    Serial.println("PCF8575 not found. Please check wiring and I2C address.");
  }

  // pcf8575
  pcf8575.write(0, HIGH); // Buzzer
  pcf8575.write(1, HIGH); // LED Relay 1 ON Direction Fan
  pcf8575.write(2, HIGH); // LED Relay 1 OFF ONE Direction Fan
  pcf8575.write(3, HIGH); // LED ONE Direction Fan
  pcf8575.write(4, HIGH); // LED TWO Direction Fan
  pcf8575.write(5, HIGH); // LED Blow OUT
  pcf8575.write(6, HIGH); // LED Blow IN

  pinMode(RL_NI1, OUTPUT);
  pinMode(RL_NI2, OUTPUT);
  pinMode(RL_NI3, OUTPUT);

  pinMode(SW_Man, INPUT);
  pinMode(SW_Auto, INPUT);
  pinMode(SW_INOUT, INPUT);
  pinMode(SW_ONOFF, INPUT);
  pinMode(SW_TOGGLE, INPUT);

  pinMode(DIO0_PIN, INPUT);

  pinMode(AC_LOAD, OUTPUT);
  pinMode(zero_cross_detect, INPUT_PULLUP);
  attachInterrupt(zero_cross_detect, zero_crosss_int, RISING);

  digitalWrite(AC_LOAD, LOW);

  // Initialize LoRa
  Serial.print("[SX1278 Slave] Initializing ... ");
  int state = lora.begin(433.0, 125.0, 12, 5, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("success!");
    attachInterrupt(digitalPinToInterrupt(DIO0_PIN), setLoRaFlag, RISING);
    lora.startReceive();
    Serial.println("[LoRa] Entering receive mode.");
  }
  else
  {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true)
      ;
  }

  // Dimmer Task
  xTaskCreatePinnedToCore(
      dimmer_w_Task, // Function to implement the task
      "DimmerTask",  // Name of the task
      2048,          // Stack size in words
      NULL,          // Task input parameter
      1,             // Priority of the task
      &dimmer_Task,  // Task handle
      1              // Core where the task should run
  );

  // LoRa Task
  xTaskCreatePinnedToCore(
      lora_Task_Handler, // Function to implement the task
      "LoRaTask",        // Name of the task
      4096,              // Stack size in words (เพิ่มขนาดสำหรับ LoRa)
      NULL,              // Task input parameter
      2,                 // Priority of the task (สูงกว่า dimmer)
      &lora_Task,        // Task handle
      0                  // Core where the task should run
  );

  if ((digitalRead(SW_Auto) == 1) && (digitalRead(SW_Man) == 0))
  {
    // Serial.printf("Auto Mode\n\r");
  }
  else if ((digitalRead(SW_Auto) == 0) && (digitalRead(SW_Man) == 1))
  {
    // Serial.printf("Manual Mode\n\r");
  }
  else if ((digitalRead(SW_Auto) == 0) && (digitalRead(SW_Man) == 0))
  {
    _off();
    _flag = 0;
  }
}

/***********************************************************************
 * FUNCTION:    loop
 * DESCRIPTION: loop process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void loop()
{
  bool _auto = digitalRead(SW_Auto);
  bool _man = digitalRead(SW_Man);

  if (_auto)
  {
    if (!loraActive) {
            Serial.println("Mode -> AUTO: Activating LoRa Task.");
            loraActive = true; // << เปิดสวิตช์ LoRa
        }
    _flag = 1;
    _ledblink();
    // Serial.printf("Auto Mode\n\r");
    // int command = 4;
    if (newLoRaCommand)
    {
      newLoRaCommand = false;
      int command = loraCommand;
      bool isCommandChanged = (command != lastCase);
      if (isCommandChanged) {
        lastCase = command; 
        Serial.printf("-> Command Changed to: %d\n", command);
      }
      // Serial.printf("Executing command seen by loop(): %d\n", command);
      // ตรวจสอบและแสดงเฉพาะเมื่อสถานะเปลี่ยน
      switch (command)
      {
      case 0: // ONE Directional Fan ON
        Serial.println("-> Command received: Turn ONE Directional Fan ON");
        pcf8575.write(2, LOW);
        pcf8575.write(3, LOW);
        pcf8575.write(4, HIGH);
        pcf8575.write(5, HIGH);
        pcf8575.write(6, HIGH);
        blinkEnable = false;
        digitalWrite(RL_NI2, 0);
        digitalWrite(RL_NI3, 0);
        power = 5;
        break;
      case 1: // TWO Directional Fan ON
        Serial.println("-> Command received: Turn TWO Directional Fan ON");
        pcf8575.write(1, HIGH);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, HIGH);
        pcf8575.write(4, LOW);
        pcf8575.write(5, HIGH);
        pcf8575.write(6, HIGH);
        blinkEnable = false;
        digitalWrite(RL_NI1, 0);
        power = 5;
        
        break;
      case 2: // Fan ONE ON
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Fan ONE ON");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 90;
        break;
      case 32: // Fan ONE ON
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Fan ONE ON");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 55;
        break;
      case 33: // Fan ONE ON
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Fan ONE ON");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 45;
        break;
      case 34: // Fan ONE ON
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Fan ONE ON");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 40;
        break;
      case 35: // Fan ONE ON
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Fan ONE ON");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 30;
        break;
      case 3: // Fan ONE OFF
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Fan ONE OFF");
        beepPCF8575();
        }
        pcf8575.write(1, HIGH);
        pcf8575.write(2, LOW);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 0);
        power = 5;
        
        break;
      case 4: // Relay 2 ON Blow OUT
        if (isCommandChanged)
        {
        Serial.println("-> Command: Blow OUT");
        // digitalWrite(RL_NI1, 0);
        beepPCF8575();
        }
        pcf8575.write(5, LOW);
        pcf8575.write(6, HIGH);
        digitalWrite(RL_NI2, 1);
        digitalWrite(RL_NI3, 0);
        power = 80;
        break;
      case 5: // Relay 2 OFF and 3 OFF (TWO Directional Fan OFF)
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Blow IN , Blow OUT OFF");
        beepPCF8575();
        }
        pcf8575.write(5, HIGH);
        pcf8575.write(6, HIGH);
        // digitalWrite(RL_NI1, 0);
        digitalWrite(RL_NI2, 0);
        digitalWrite(RL_NI3, 0);
        power = 5;
        break;
      case 6: // Relay 2,3 ON = Blow IN
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Blow IN");
        beepPCF8575();
        }
        pcf8575.write(5, HIGH);
        pcf8575.write(6, LOW);
        // digitalWrite(RL_NI1, 0);
        digitalWrite(RL_NI2, 0);
        digitalWrite(RL_NI3, 1);
        power = 80;
        break;
        // ---------------------------------------------- PM2.5 MODE -----------------------------------------------------
      case 7: // PM2.5 MODE IN ONE Directional Fan ON (pm2p5 <= 24.9) 0% 
        if (isCommandChanged)
        {
        Serial.println("-> Command received: PM2.5 MODE IN ONE Directional Fan");
        beepPCF8575();
        }
        pcf8575.write(1, HIGH);
        pcf8575.write(2, LOW);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 0);
        power = 5;
        break;
      case 8: // PM2.5 MODE IN ONE Directional Fan ON (pm2p5 >= 25.0 && pm2p5 <= 37.4) // 30%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: PM2.5 MODE IN ONE Directional Fan");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 30;
        break;
      case 9: // PM2.5 MODE IN ONE Directional Fan ON (pm2p5 >= 37.5 && pm2p5 <= 49.9) // 40%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: PM2.5 MODE IN ONE Directional Fan");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 40;
        break;
      case 10: // PM2.5 MODE IN ONE Directional Fan ON (pm2p5 >= 50.0 && pm2p5 <= 62.4) // 45%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: PM2.5 MODE IN ONE Directional Fan");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 45;
        break;
      case 11: // PM2.5 MODE IN ONE Directional Fan ON (pm2p5 >= 62.5 && pm2p5 <= 74.9) // 55%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: PM2.5 MODE IN ONE Directional Fan");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 55;
        break;
      case 12: // PM2.5 MODE IN ONE Directional Fan ON (pm2p5 >= 75.0) // 90%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: PM2.5 MODE IN ONE Directional Fan");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 90;
        break;

        // ----------------------------------------- CO2 MODE --------------------------------------------------------
      case 13: // CO2 MODE IN ONE Directional Fan ON (_co2 <= 749) 0%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: CO2 MODE IN ONE Directional Fan 0%");
        beepPCF8575();
        }
        pcf8575.write(1, HIGH);
        pcf8575.write(2, LOW);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 0);
        power = 5;
        break;
      case 14: // CO2 MODE IN ONE Directional Fan ON else if (_co2 >= 750 && _co2 <= 899) 30%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: CO2 MODE IN ONE Directional Fan 30%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 30;
        break;
      case 15: // CO2 MODE IN ONE Directional Fan ON else if (_co2 >= 900 && _co2 <= 1049) 40%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: CO2 MODE IN ONE Directional Fan 40%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 40;
        break;
      case 16: // CO2 MODE IN ONE Directional Fan ON else if (_co2 >= 1050 && _co2 <= 1199) 45%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: CO2 MODE IN ONE Directional Fan 45%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 45;
        break;
      case 17: // CO2 MODE IN ONE Directional Fan ON else if (one_co2 >= 1200 && one_co2 <= 1349) 55%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: CO2 MODE IN ONE Directional Fan 55%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 55;
        break;
      case 18: // CO2 MODE IN ONE Directional Fan ON else if (one_co2 >= 1350) 90%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: CO2 MODE IN ONE Directional Fan 90%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 90;
        break;

        // ----------------------------------------- Radon MODE --------------------------------------------------------
      case 19: // Radon MODE IN ONE Directional Fan ON if (radon >= 0.0 && radon <= 47.9) 0%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Radon MODE IN ONE Directional Fan 0%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 5;
        break;
      case 20: // Radon MODE IN ONE Directional Fan ON if (radon >= 48.0 && radon <= 72.9) 30%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Radon MODE IN ONE Directional Fan 30%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 30;
        break;
      case 21: // Radon MODE IN ONE Directional Fan ON if (radon >= 73.0 && radon <= 97.9) 40%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Radon MODE IN ONE Directional Fan 40%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 40;
        break;
      case 22: // Radon MODE IN ONE Directional Fan ON if (radon >= 98.0 && radon <= 122.9) 45%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Radon MODE IN ONE Directional Fan 45%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 45;
        break;
      case 23: // Radon MODE IN ONE Directional Fan ON if (radon >= 123.0 && radon <= 147.9) 55%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Radon MODE IN ONE Directional Fan 55%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 55;
        break;
      case 24: // Radon MODE IN ONE Directional Fan ON if (radon >=148) 90%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Radon MODE IN ONE Directional Fan 90%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 90;
        break;
        // ----------------------------------------- Auto Mode OFF --------------------------------------------------------
      case 31: // Auto Mode OFF
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode OFF");
        beepPCF8575();
        }
        pcf8575.write(1, HIGH);
        pcf8575.write(2, LOW);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 0);
        power = 5;
        break;
        // ----------------------------------------- Auto Mode ALL --------------------------------------------------------
      case 25: // Auto Mode ALL if ((radon >= 0.0 && radon <= 47.9) || (_co2 >= 0 && _co2 <= 749) || (pm2p5 <= 24.9)) // 0%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode ALL IN ONE Directional Fan 0%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 5;
        break;
      case 26: // Auto Mode ALL ((radon >= 48.0 && radon <= 72.9) || (_co2 >= 750 && _co2 <= 899) || (pm2p5 >= 25.0 && pm2p5 <= 37.4)) // 30%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode ALL IN ONE Directional Fan 30%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 30;
        break;
      case 27: // Auto Mode ALL ((radon >= 73.0 && radon <= 97.9) || (_co2 >= 900 && _co2 <= 1049) || (pm2p5 >= 37.5 && pm2p5 <= 49.9)) // 40%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode ALL IN ONE Directional Fan 40%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 40;
        break;
      case 28: // Auto Mode ALL ((radon >= 98.0 && radon <= 122.9) || (_co2 >= 1050 && _co2 <= 1199) || (pm2p5 >= 50.0 && pm2p5 <= 62.4)) // 45%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode ALL IN ONE Directional Fan 45%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 45;
        break;
      case 29: // Auto Mode ALL ((radon >= 123.0 && radon <= 147.9) || (_co2 >= 1200 && _co2 <= 1349) || (pm2p5 >= 62.5 && pm2p5 <= 74.9)) // 55%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode ALL IN ONE Directional Fan 55%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 55;
        break;
      case 30: // Auto Mode ALL ((radon >= 148) || (_co2 >= 1350) || (pm2p5 >= 75.0)) // 90%
        if (isCommandChanged)
        {
        Serial.println("-> Command received: Auto Mode ALL IN ONE Directional Fan 90%");
        beepPCF8575();
        }
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        pcf8575.write(3, LOW);
        digitalWrite(RL_NI1, 1);
        power = 90;
        break;
      // case 16: // CO2 MODE IN TWO Directional Fan (co_2 <= 499) 0%
      //   Serial.println("-> Command received: CO2 MODE IN TWO Directional Fan (co_2 <= 499) 0%");
      //   digitalWrite(RL_NI2, 0);
      //   digitalWrite(RL_NI3, 0);
      //   pcf8575.write(5, HIGH);
      //   pcf8575.write(6, HIGH);
      //   power = 5;
      //   break;
      // case 17: // CO2 MODE IN TWO Directional Fan (co_2 >= 500 && co_2 <= 749) 25%
      //   Serial.println("-> Command received: Radon MODE IN TWO Directional Fan (co_2 >= 500 && co_2 <= 749) 25%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 0);
      //   power = 30;
      //   pcf8575.write(5, LOW);
      //   pcf8575.write(6, HIGH);
      //   break;
      // case 18: // CO2 MODE IN TWO Directional Fan (co_2 >= 750 && co_2 <= 999) 50%
      //   Serial.println("-> Command received: Radon+CO2 MODE IN TWO Directional Fan (co_2 >= 750 && co_2 <= 999) 50%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 0);
      //   power = 41;
      //   pcf8575.write(5, LOW);
      //   pcf8575.write(6, HIGH);
      //   break;
      // case 19: // CO2 MODE IN TWO Directional Fan (co_2 >= 1000 && co_2 <= 1249) 75%
      //   Serial.println("-> Command received: Radon+CO2 MODE IN TWO Directional Fan (co_2 >= 1000 && co_2 <= 1249) 75%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 0);
      //   power = 70;
      //   pcf8575.write(5, LOW);
      //   pcf8575.write(6, HIGH);
      //   break;
      // case 20: // CO2 MODE IN TWO Directional Fan (co_2 >= 1250) 100%
      //   Serial.println("-> Command received: Radon+CO2 MODE IN TWO Directional Fan (co_2 >= 1000 && co_2 <= 1249) 75%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 0);
      //   power = 80;
      //   pcf8575.write(5, LOW);
      //   pcf8575.write(6, HIGH);
      //   break;
      // case 21: // Radon Mode ONE Directional Fan ON (radon >= 0 && radon <= 0.3) 0%
      //   Serial.println("-> Command received: Turn ONE Directional Fan ON");
      //   digitalWrite(RL_NI1, 0);
      //   pcf8575.write(1, HIGH);
      //   pcf8575.write(2, LOW);
      //   pcf8575.write(3, LOW);
      //   power = 5;
      // break;
      // case 22: // Radon Mode ONE Directional Fan ON (radon >= 0.4 && radon <= 1.2) 25%
      //   Serial.println("-> Command received: Radon Mode TWO Directional Fan ON 25%");
      //   digitalWrite(RL_NI1, 1);
      //   pcf8575.write(1, LOW);
      //   pcf8575.write(2, HIGH);
      //   pcf8575.write(3, LOW);
      //   power = 20;
      //   break;
      // case 23: // Radon Mode ONE Directional Fan ON (radon >= 1.3 && radon <= 1.9) 50%
      //   Serial.println("-> Command received: Radon Mode TWO Directional Fan ON 50%");
      //   digitalWrite(RL_NI1, 1);
      //   pcf8575.write(1, LOW);
      //   pcf8575.write(2, HIGH);
      //   pcf8575.write(3, LOW);
      //   power = 50;
      //   break;
      // case 24: // Radon Mode ONE Directional Fan ON (radon >= 2 && radon <= 3.9) 75%
      //   Serial.println("-> Command received: Radon Mode TWO Directional Fan ON 75%");
      //   digitalWrite(RL_NI1, 1);
      //   pcf8575.write(1, LOW);
      //   pcf8575.write(2, HIGH);
      //   pcf8575.write(3, LOW);
      //   power = 60;
      //   break;
      // case 25: // Radon Mode ONE Directional Fan ON (radon >=4 ) 100%
      //   Serial.println("-> Command received: Radon Mode TWO Directional Fan ON 100%");
      //   digitalWrite(RL_NI1, 1);
      //   pcf8575.write(1, LOW);
      //   pcf8575.write(2, HIGH);
      //   pcf8575.write(3, LOW);
      //   power = 80;
      //   break;
      // case 26: // Radon Mode TWO Directional Fan ON (radon >= 0 && radon <= 0.3) 0%
      //   Serial.println("-> Radon Mode TWO Directional Fan ON (radon >= 0 && radon <= 0.3) 0%");
      //   digitalWrite(RL_NI2, 0);
      //   digitalWrite(RL_NI3, 0);
      //   pcf8575.write(5, HIGH);
      //   pcf8575.write(6, LOW);
      //   power = 5;
      //   break;
      // case 27: // Radon Mode TWO Directional Fan ON (radon >= 0.4 && radon <= 1.2) 25%
      //   Serial.println("-> Radon Mode TWO Directional Fan ON (radon >= 0.4 && radon <= 1.2) 25%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 1);
      //   pcf8575.write(5, HIGH);
      //   pcf8575.write(6, LOW);
      //   power = 30;
      //   break;
      // case 28: // Radon Mode TWO Directional Fan ON (radon >= 1.3 && radon <= 1.9) 50%
      //   Serial.println("-> Radon Mode TWO Directional Fan ON (radon >= 1.3 && radon <= 1.9) 50%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 1);
      //   pcf8575.write(5, HIGH);
      //   pcf8575.write(6, LOW);
      //   power = 40;
      //   break;
      // case 29: // Radon Mode TWO Directional Fan ON (radon >= 2 && radon <= 3.9) 75%
      //   Serial.println("-> Radon Mode TWO Directional Fan ON (radon >= 2 && radon <= 3.9) 75%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 1);
      //   pcf8575.write(5, HIGH);
      //   pcf8575.write(6, LOW);
      //   power = 60;
      //   break;
      // case 30: // Radon Mode TWO Directional Fan ON (radon >=4 ) 100%
      //   Serial.println("-> Radon Mode TWO Directional Fan ON (radon >=4 ) 100%");
      //   digitalWrite(RL_NI2, 1);
      //   digitalWrite(RL_NI3, 1);
      //   pcf8575.write(5, HIGH);
      //   pcf8575.write(6, LOW);
      //   power = 80;
      //   break;
      // default:
      //   Serial.println("-> Unknown command received.");
      //   break;
      }
    }
    esp_task_wdt_reset();
  }
  else if (_man)
  {
    if (loraActive) {
            Serial.println("Mode -> MANUAL: Deactivating LoRa Task.");
            loraActive = false; // << ปิดสวิตช์ LoRa
            lora.standby();     // << สั่งให้ชิป LoRa หยุดทำงานเพื่อประหยัดพลังงาน
        }
    _flag = 1;
    Serial.printf("Manaul Mode\n\r");
    _ledblink();

    // -------------------------- Manual Control Switch ON-OFF --------------------------
    if (state_toggle == 1)
    {
      if (digitalRead(SW_ONOFF))
      {
        beepPCF8575();
        // _bz();
        delay(3000);
        state_onoff++;

        while (digitalRead(SW_ONOFF));
      }
      if (state_onoff == 1)
      {
        Serial.printf(" FAN ONE ON\r\n");
        digitalWrite(RL_NI1, 1); // Line AC
        pcf8575.write(1, LOW);
        pcf8575.write(2, HIGH);
        power = 90;
      }
      else if (state_onoff >= 2)
      {
        Serial.printf(" FAN ONE OFF\r\n");
        pcf8575.write(1, HIGH);
        pcf8575.write(2, LOW);
        _off_2();
        state_onoff = 0;
      }
      Serial.printf("debug state_onoff: %d \r\n", state_onoff);
    }
    else
    {
      state_onoff = 0;
    }

    // -------------------------- Manual Control Switch IN-OUT --------------------------
    if (state_toggle == 2)
    {
      if (digitalRead(SW_INOUT))
      {
        beepPCF8575();
        delay(3000);
        state++;
        while (digitalRead(SW_INOUT));
      }

      if (state == 1)
      {
        Serial.printf(" FAN OUT\r\n");
        digitalWrite(RL_NI2, 1); // Line AC
        digitalWrite(RL_NI3, 0);
        pcf8575.write(5, LOW);
        pcf8575.write(6, HIGH);

        power = 60;
      }
      else if (state == 2)
      {
        Serial.printf(" FAN IN\r\n");
        digitalWrite(RL_NI2, 0);
        digitalWrite(RL_NI3, 1);
        pcf8575.write(5, HIGH);
        pcf8575.write(6, LOW);
        power = 40;
      }
      else if (state >= 3)
      {
        pcf8575.write(5, HIGH);
        pcf8575.write(6, HIGH);
        _off_2();
        state = 0;
      }
      Serial.printf("debug state inout: %d \r\n", state);
    }
    else
    {
      state = 0;
    }

    // -------------------------- Manual Control Switch TOGGLE --------------------------
    if (state_onoff == 0 && state == 0)
    {
      if (digitalRead(SW_TOGGLE))
      {
        beepPCF8575_2();
        // _bz();
        delay(3000);
        state_toggle++;
        blinkEnable = false;

        while (digitalRead(SW_TOGGLE))
          ;
      }
      if (state_toggle == 1)
      {
        Serial.printf(" FAN ONE \r\n");
        pcf8575.write(2, LOW);
        pcf8575.write(3, LOW);
        pcf8575.write(4, HIGH);
      }
      else if (state_toggle == 2)
      {
        Serial.printf(" FAN TWO \r\n");
        pcf8575.write(2, HIGH);
        pcf8575.write(3, HIGH);
        pcf8575.write(4, LOW);
      }
      else if (state_toggle >= 3)
      {
        state_toggle = 1;
      }
      Serial.printf("debug state_toggle: %d \r\n", state_toggle);
    }
  }
  else if ((_auto == 0) && (_man == 0))
  {

    if (loraActive) {
             Serial.println("Mode -> OFF: Deactivating LoRa Task.");
            loraActive = false; // << ปิดสวิตช์ LoRa
            lora.standby();
        }

    if (_flag)
    {
      // ถ้า _flag เป็น 1
      ESP.restart();
    }
  }
  else
  {
    state = 0;
    _off();
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  esp_task_wdt_reset();
}

/***********************************************************************
 * FUNCTION:    dimmer_w_Task
 * DESCRIPTION: Dimmer Power
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void dimmer_w_Task(void *p)
{
  while (1)
  {
    time_delay_us = PERCENT_to_TIME(power);
    start();
    // vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

/***********************************************************************
 * FUNCTION:    start
 * DESCRIPTION: run zero cross
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void start()
{
  if (zero_cross)
  {
    /* Run */ // สั่งทำงาน

    delayMicroseconds((uint32_t)time_delay_us);
    digitalWrite(AC_LOAD, HIGH);
    // delayMicroseconds(8500);
    delayMicroseconds(500);
    zero_cross = 0;
    digitalWrite(AC_LOAD, LOW);
  }
}

/***********************************************************************
 * FUNCTION:    PERCENT_to_TIME
 * DESCRIPTION: fan speed level
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
float PERCENT_to_TIME(int value)
{
  float _percent_value;

  _percent_value = 0.01 - ((float)value * 0.01 / 100); // 1 Cycle มี 50Hz = 20mS คิดที่ Half cycle 100Hz = 10mS~ 0.01s
  // _percent_value = 0.0083 - ((float)value * 0.0083 / 100); // 1 Cycle มี 60Hz = 16.66mS คิดที่ Half cycle 120Hz = 8.33mS~ 0.0083s
  _percent_value = _percent_value * 1000000;

  return _percent_value;
}

/***********************************************************************
 * FUNCTION:    zero_crosss_int
 * DESCRIPTION: Input zero crossing trig
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void zero_crosss_int() // function to be fired at the zero crossing to dim the light
{
  zero_cross = 1;
  // digitalWrite(AC_LOAD,LOW);
}

/***********************************************************************
 * FUNCTION:    lora_Task_Handler
 * DESCRIPTION: LoRa Task Handler for receiving LoRa messages
 * PARAMETERS:  void *p - task parameter
 * RETURNED:    nothing
 ***********************************************************************/
void lora_Task_Handler(void *p)
{
  while (1)
  {
    if (loraPacketReceived)
    {
      loraPacketReceived = false;
      String message;
      int state = lora.readData(message);

      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.printf("Received LoRa message: %s\n", message.c_str());
        loraCommand = message.toInt();
        newLoRaCommand = true;
      }
      else
      {
        Serial.printf("Failed to read LoRa message, state: %d\n", state);
      }
      // Restart receive mode after reading data
      lora.startReceive();
    }

    // Delay เล็กน้อยเพื่อให้ task อื่นๆ ทำงาน
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// void lora_Task_Handler(void *p)
// {
//     while (1)
//     {
//         // ตรวจสอบ "สวิตช์" ก่อนเริ่มทำงาน
//         if (loraActive)
//         {
//             // --- เริ่มกระบวนการรับสัญญาณ LoRa ---
//             lora.startReceive();
            
//             // รอจนกว่าจะมีสัญญาณเข้ามา (เช็คจาก ISR) หรือโหมดเปลี่ยน
//             while (loraActive && !loraPacketReceived) {
//                 vTaskDelay(20 / portTICK_PERIOD_MS);
//             }
            
//             // ถ้ามีข้อมูลเข้ามา และยังอยู่ในโหมด Auto
//             if (loraPacketReceived && loraActive)
//             {
//                 loraPacketReceived = false; // เคลียร์แฟล็ก
//                 String message;
//                 int state = lora.readData(message);

//                 if (state == RADIOLIB_ERR_NONE)
//                 {
//                     loraCommand = message.toInt();
//                     newLoRaCommand = true;
//                 }
//             }
//         }
//         else
//         {
//             // ถ้า LoRa ไม่ได้เปิดใช้งาน ให้ Task หน่วงเวลานานขึ้น
//             // เพื่อลดการใช้ CPU โดยไม่จำเป็น
//             vTaskDelay(500 / portTICK_PERIOD_MS);
//         }
//     }
// }

/***********************************************************************
 * FUNCTION:      lora_Task_Handler (ฉบับปรับปรุง)
 * DESCRIPTION:   จัดการการรับข้อมูล LoRa อย่างมีประสิทธิภาพ
 ***********************************************************************/
// void lora_Task_Handler(void *p)
// {
//     while (1)
//     {
//         // 1. ตรวจสอบว่าอยู่ในโหมด Auto หรือไม่
//         if (loraActive)
//         {
//             // 2. รอจนกว่าจะมี Flag จาก ISR ว่ามีข้อมูลเข้ามา
//             // โดยการใช้ Interrupt จะทำให้ Task นี้ไม่ทำงานเลยจนกว่าจะมีข้อมูลจริงๆ
//             if (loraPacketReceived)
//             {
//                 // 3. เมื่อมีข้อมูลเข้ามา ให้เคลียร์ Flag ทันที
//                 loraPacketReceived = false;

//                 // 4. อ่านข้อมูลที่เข้ามา
//                 String message;
//                 int state = lora.readData(message);

//                 if (state == RADIOLIB_ERR_NONE)
//                 {
//                     // Serial.printf("Received LoRa message: %s\n", message.c_str());
//                     // แปลงข้อความเป็นตัวเลขแล้วเก็บในตัวแปร global
//                     loraCommand = message.toInt();
//                     newLoRaCommand = true; // ตั้ง Flag ว่ามีคำสั่งใหม่เข้ามา
//                 }
//                 else
//                 {
//                     Serial.printf("Failed to read LoRa message, state: %d\n", state);
//                 }
                
//                 // 5. หลังจากอ่านข้อมูลเสร็จ ให้กลับเข้าสู่โหมดรับสัญญาณอีกครั้ง
//                 // เพื่อเตรียมรับข้อมูลถัดไป
//                 lora.startReceive();
//             }
//         }
//         else
//         {
//             // ถ้าไม่ได้อยู่ในโหมด Auto ให้ Task หยุดรอสักพักใหญ่ๆ
//             // เพื่อคืน CPU ให้กับ Task อื่นๆ
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
        
//         // เพิ่ม Delay เล็กน้อยในลูปหลักของ Task เพื่อป้องกันการทำงานหนักเกินไป
//         // ในกรณีที่เกิดการวนลูปถี่ๆ
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// void lora_Task_Handler(void *p)
// {
//     while (1)
//     {
//         if (loraActive)
//         {
//             if (loraPacketReceived)
//             {
//                 loraPacketReceived = false;
//                 String message;
//                 int state = lora.readData(message);

//                 if (state == RADIOLIB_ERR_NONE)
//                 {
//                     // ตรวจสอบความยาวของ message
//                     message.trim(); // ลบช่องว่างหน้า-หลัง
                    
//                     if (message.length() > 0)
//                     {
//                         Serial.printf("Raw LoRa message: '%s' (length: %d)\n", 
//                                      message.c_str(), message.length());
                        
//                         int newCommand = message.toInt();
                        
//                         // ตรวจสอบว่าเป็นคำสั่งที่ถูกต้องหรือไม่
//                         if (newCommand >= 0 && newCommand <= 100) // กำหนดช่วงที่ยอมรับ
//                         {
//                             // ตรวจสอบว่าไม่ใช่คำสั่งซ้ำ
//                             if (newCommand != loraCommand || !newLoRaCommand)
//                             {
//                                 loraCommand = newCommand;
//                                 newLoRaCommand = true;
//                                 Serial.printf("Valid command accepted: %d\n", newCommand);
//                             }
//                             else
//                             {
//                                 Serial.printf("Duplicate command ignored: %d\n", newCommand);
//                             }
//                         }
//                         else
//                         {
//                             Serial.printf("Invalid command range: %d\n", newCommand);
//                         }
//                     }
//                     else
//                     {
//                         Serial.println("Empty LoRa message received");
//                     }
//                 }
//                 else
//                 {
//                     Serial.printf("Failed to read LoRa message, state: %d\n", state);
//                 }
                
//                 // Clear any remaining data in buffer
//                 lora.standby();
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//                 lora.startReceive();
//             }
//         }
//         else
//         {
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
        
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// void lora_Task_Handler(void *p)
// {
//     static unsigned long lastValidMessage = 0;
//     const unsigned long MIN_MESSAGE_INTERVAL = 100; // 100ms minimum between messages
    
//     while (1)
//     {
//         if (loraActive)
//         {
//             if (loraPacketReceived)
//             {
//                 loraPacketReceived = false;
                
//                 // เพิ่ม delay เล็กน้อยเพื่อให้ข้อมูลมาครบ
//                 vTaskDelay(5 / portTICK_PERIOD_MS);
                
//                 String message;
//                 int state = lora.readData(message);

//                 if (state == RADIOLIB_ERR_NONE)
//                 {
//                     message.trim(); // ลบช่องว่างและ whitespace
                    
//                     // ตรวจสอบว่าไม่ใช่ empty message และไม่ได้มาใกล้เคียงกับข้อความก่อนหน้า
//                     if (message.length() > 0 && 
//                         (millis() - lastValidMessage > MIN_MESSAGE_INTERVAL))
//                     {
//                         Serial.printf("Raw LoRa message: '%s' (length: %d)\n", 
//                                      message.c_str(), message.length());
                        
//                         int newCommand = message.toInt();
                        
//                         // ตรวจสอบว่าเป็นตัวเลขจริง (ไม่ใช่ garbage data)
//                         if (message.charAt(0) >= '0' && message.charAt(0) <= '9')
//                         {
//                             // ตรวจสอบช่วงคำสั่งที่ยอมรับ
//                             if (newCommand >= 0 && newCommand <= 50)
//                             {
//                                 // ตรวจสอบว่าไม่ใช่คำสั่งซ้ำ
//                                 if (newCommand != loraCommand || !newLoRaCommand)
//                                 {
//                                     loraCommand = newCommand;
//                                     newLoRaCommand = true;
//                                     lastValidMessage = millis();
//                                     Serial.printf("Valid command accepted: %d\n", newCommand);
//                                 }
//                                 else
//                                 {
//                                     Serial.printf("Duplicate command ignored: %d\n", newCommand);
//                                 }
//                             }
//                             else
//                             {
//                                 Serial.printf("Command out of range: %d\n", newCommand);
//                             }
//                         }
//                         else
//                         {
//                             Serial.printf("Invalid message format: '%s'\n", message.c_str());
//                         }
//                     }
//                     else if (message.length() == 0)
//                     {
//                         Serial.println("Empty LoRa message ignored");
//                     }
//                     else
//                     {
//                         Serial.printf("Message too soon after last valid message, ignored: '%s'\n", 
//                                      message.c_str());
//                     }
//                 }
//                 else
//                 {
//                     Serial.printf("Failed to read LoRa message, state: %d\n", state);
//                 }
                
//                 // Comprehensive buffer clearing
//                 lora.standby();
//                 vTaskDelay(20 / portTICK_PERIOD_MS); // เพิ่มเวลา standby
                
//                 // Clear any pending interrupts
//                 loraPacketReceived = false;
                
//                 // Restart receive mode
//                 int restartState = lora.startReceive();
//                 if (restartState != RADIOLIB_ERR_NONE) {
//                     Serial.printf("Failed to restart LoRa receive, state: %d\n", restartState);
//                     vTaskDelay(100 / portTICK_PERIOD_MS);
//                 }
//             }
//         }
//         else
//         {
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
        
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// void lora_Task_Handler(void *p)
// {
//     static unsigned long lastCommandTime = 0;
//     const unsigned long COMMAND_TIMEOUT = 200; // 200ms between commands
    
//     while (1)
//     {
//         if (loraActive)
//         {
//             if (loraPacketReceived)
//             {
//                 // Disable interrupt temporarily
//                 detachInterrupt(digitalPinToInterrupt(DIO0_PIN));
//                 loraPacketReceived = false;
                
//                 String message;
//                 int state = lora.readData(message);

//                 if (state == RADIOLIB_ERR_NONE && message.length() > 0)
//                 {
//                     message.trim();
//                     int newCommand = message.toInt();
                    
//                     // ป้องกันคำสั่งมาเร็วเกินไป
//                     unsigned long now = millis();
//                     if (now - lastCommandTime > COMMAND_TIMEOUT)
//                     {
//                         if (newCommand >= 2 && newCommand <= 3) // เฉพาะคำสั่งที่ต้องการ
//                         {
//                             loraCommand = newCommand;
//                             newLoRaCommand = true;
//                             lastCommandTime = now;
//                             Serial.printf("Command accepted: %d\n", newCommand);
//                         }
//                     }
//                 }
                
//                 // Simple restart - ไม่ต้องซับซ้อน
//                 lora.startReceive();
                
//                 // Re-enable interrupt
//                 attachInterrupt(digitalPinToInterrupt(DIO0_PIN), setLoRaFlag, RISING);
//             }
//         }
        
//         vTaskDelay(50 / portTICK_PERIOD_MS); // เพิ่ม delay ให้มากขึ้น
//     }
// }

/***********************************************************************
 * FUNCTION:    _off
 * DESCRIPTION: turn off all 
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void _off()
{
  digitalWrite(RL_NI1, 0);
  digitalWrite(RL_NI2, 0);
  digitalWrite(RL_NI3, 0);
  pcf8575.write(0, HIGH);
  pcf8575.write(1, HIGH);
  pcf8575.write(2, HIGH);
  pcf8575.write(3, HIGH);
  pcf8575.write(4, HIGH);
  pcf8575.write(5, HIGH);
  pcf8575.write(6, HIGH);
  Serial.printf("OFF MODE\n\r");
}

/***********************************************************************
 * FUNCTION:    _off2
 * DESCRIPTION: turn off relay
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void _off_2()
{
  digitalWrite(RL_NI1, 0);
  digitalWrite(RL_NI2, 0);
  digitalWrite(RL_NI3, 0);
}

/***********************************************************************
 * FUNCTION:    eepPCF8575
 * DESCRIPTION: buzzer sound
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void beepPCF8575()
{
  pcf8575.write(0, LOW);
  delay(200);
  pcf8575.write(0, HIGH);
  delay(200);
  pcf8575.write(0, LOW);
  delay(200);
  pcf8575.write(0, HIGH);
  delay(200);
}

/***********************************************************************
 * FUNCTION:    beepPCF8575_2
 * DESCRIPTION: buzzer sound
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void beepPCF8575_2()
{
  pcf8575.write(0, LOW);
  delay(500);
  pcf8575.write(0, HIGH);
  delay(500);
  pcf8575.write(0, LOW);
  delay(500);
  pcf8575.write(0, HIGH);
  delay(500);
}

/***********************************************************************
 * FUNCTION:    _lebblink
 * DESCRIPTION: start before select mode
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void _ledblink()
{
  if (blinkEnable && millis() - lastBlink >= 200)
    {
      ledState = !ledState;
      pcf8575.write(3, ledState ? LOW : HIGH); // Active LOW
      delay(100);                              // หน่วงเวลาเพื่อให้ LED สว่างชัดเจน
      pcf8575.write(4, ledState ? LOW : HIGH);
      lastBlink = millis();
      delay(100); // หน่วงเวลาเพื่อให้ LED สว่างชัดเจน
    }
}

// /***********************************************************************
//  * FUNCTION:    beepPCF8575_custom
//  * DESCRIPTION: buzzer sound with customizable count and duration
//  * PARAMETERS:  count - number of beeps
//  * on_duration_ms - how long the beep stays on in ms
//  * off_duration_ms - how long the pause is between beeps in ms
//  * RETURNED:    nothing
//  ***********************************************************************/
// void beepPCF8575_custom(int count, int on_duration_ms, int off_duration_ms)
// {
//   for (int i = 0; i < count; i++) 
//   {
//     pcf8575.write(0, LOW);      // Turn buzzer ON (Active LOW)
//     delay(on_duration_ms);
//     pcf8575.write(0, HIGH);     // Turn buzzer OFF
    
//     // Add delay between beeps, but not after the last one
//     if (i < count - 1)
//     {
//       delay(off_duration_ms);
//     }
//   }
// }