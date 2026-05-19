<img width="1385" height="772" alt="Screenshot 2026-05-19 010349" src="https://github.com/user-attachments/assets/2586f6bd-f799-45fb-80de-069ab11c529a" />

```cpp
int main(void) 
#include <Wire.h> 
#include <TinyGPS++.h> 
#include <HardwareSerial.h> 
// MPU6050 
#define MPU_ADDR   
  0x68 
#define MPU_INT_PIN  27 
// I2C 
#define SDA_PIN 21 
#define SCL_PIN 22 
// SIM800L (GSM) 
#define GSM_RX_PIN 33 
#define GSM_TX_PIN 32 
// GPS 
#define GPS_RX_PIN    
   16 
#define GPS_TX_PIN    
   17 
#define GPS_STANDBY_PIN  18   
// GPS ijungimo / standby valdymas 
 
 
TinyGPSPlus gps;                // GPS parseris 
HardwareSerial gpsSerial(2);    // UART2 – GPS 
HardwareSerial sim800(1);       // UART1 – GSM 
 
/* 
   NUSTATYMAI  
*/ 
 
static const long GSM_BAUD = 115200; 
static const long GPS_BAUD = 9600; 
 
// GPS standby logika (pagal modulio datasheet) 
#define GPS_ACTIVE   LOW 
#define GPS_STANDBY  HIGH 
 
const char* phoneNumber = "";//telefono numeris, i kuri siunciamos žinutes 
 
/* 
   JUDESIO ANALIZE  
*/ 
 
#define ANALYZE_TIME_MS     8000    // kiek laiko analizuojam judesi 
#define SAMPLE_DELAY_MS    200     // kas kiek ms skaitom akselerometra 
#define MOVEMENT_SUM_LIMIT 30000   // riba, kada laikom, kad tai vagyste 
#define MOTION_COOLDOWN_MS 3000    // pauze tarp analizes bandymu 
 
/* 
   GPS  
*/ 
 
#define GPS_FIX_TIMEOUT    50000   // max laikas laukti GPS fix 
 
/* 
   SISTEMOS BUSENOS  
*/ 
 
volatile bool motionIRQ = false;  // nustatoma per pertrauka 
bool analyzingMotion = false; 
bool alarmActive = false; 
 
bool gpsSearching = false; 
bool gpsRequested = false; 
 
unsigned long gpsStart = 0; 
unsigned long lastSMSCheck = 0; 
unsigned long lastMotionTime = 0; 
 
/* 
   SMS KOMANDOS  
*/ 
 
enum SmsCmd { 
  CMD_NONE, 
  CMD_GPS, 
  CMD_RESET 
}; 
 
/* 
   PERTRAUKA IŠ MPU6050  
*/ 
void IRAM_ATTR onMotionISR() { 
  motionIRQ = true;    // užfiksuotas judesys 
} 
 
/* 
   MPU6050 FUNKCIJOS  
*/ 
 
// išvalom pertraukos busena MPU6050, kad butu galima aptikti kita judesi 
void clearMPUInt() { 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x3A);          // INT_STATUS 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU_ADDR, 1); 
  if (Wire.available()) Wire.read(); 
} 
 
// MPU inicializacija 
void setupMPU6050() { 
 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B); Wire.write(0x00);   // išjungiam sleep 
  Wire.endTransmission(); 
 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x1C); Wire.write(0x00);   // ±2g 
  Wire.endTransmission(); 
 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x1F); Wire.write(10);     // judesio slenkstis 
  Wire.endTransmission(); 
 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x20); Wire.write(10);     // judesio trukme 
  Wire.endTransmission(); 
 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x38); Wire.write(0x40);   // ijungiam motion interrupt 
  Wire.endTransmission(); 
 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x37); Wire.write(0x00);   // INT active HIGH 
  Wire.endTransmission(); 
 
} 
 
// nuskaitymas iš akselerometro 
void readAccel(int16_t &ax, int16_t &ay, int16_t &az) { 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x3B); 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU_ADDR, 6, true); 
 
  if (Wire.available() == 6) { 
    ax = Wire.read() << 8 | Wire.read(); 
    ay = Wire.read() << 8 | Wire.read(); 
    az = Wire.read() << 8 | Wire.read(); 
  } 
} 
 
/* 
   GSM FUNKCIJOS  
*/ 
 
// nuskaityti atsakyma iš SIM800 
String sim800Read(unsigned long timeout = 2000) { 
  String r = ""; 
  unsigned long t = millis(); 
  while (millis() - t < timeout) { 
    while (sim800.available()) { 
      r += (char)sim800.read(); 
    } 
  } 
  return r; 
} 
 
// SMS siuntimas 
void sendSMS(const String &msg) { 
 
 
  sim800.println("AT+CMGF=1"); 
  delay(500); 
 
  sim800.print("AT+CMGS=\""); 
  sim800.print(phoneNumber); 
  sim800.println("\""); 
  delay(1000); 
 
  sim800.println(msg); 
  sim800.write(26);   // CTRL+Z 
  delay(4000); 
} 
 
// SMS komandu nuskaitymas 
SmsCmd readSmsCommand() { 
  sim800.println("AT+CMGL=\"REC UNREAD\""); 
  delay(1500); 
 
  String r = sim800Read(); 
  String u = r; 
  u.toUpperCase(); 
 
   
 
  if (u.indexOf("RESET") != -1) { 
    sim800.println("AT+CMGD=1,4"); 
    return CMD_RESET; 
  } 
 
  if (u.indexOf("GPS") != -1) { 
    sim800.println("AT+CMGD=1,4"); 
    return CMD_GPS; 
  } 
 
  return CMD_NONE; 
} 
 
/* 
   GPS FUNKCIJOS  
*/ 
 
void gpsStartSearch() { 
  digitalWrite(GPS_STANDBY_PIN, GPS_ACTIVE); 
 
  gps = TinyGPSPlus();    // resetinam parseri 
  gpsSearching = true; 
  gpsRequested = true; 
  gpsStart = millis(); 
  lastStatusPrint = 0; 
} 
 
void gpsStop() { 
  digitalWrite(GPS_STANDBY_PIN, GPS_STANDBY); 
  gpsSearching = false; 
} 
 
 
 
/* 
   SETUP  
*/ 
 
void setup() { 
 
 
  Wire.begin(SDA_PIN, SCL_PIN); 
 
  pinMode(MPU_INT_PIN, INPUT); 
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMotionISR, RISING); 
 
  pinMode(GPS_STANDBY_PIN, OUTPUT); 
  digitalWrite(GPS_STANDBY_PIN, GPS_STANDBY); 
 
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
  sim800.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN); 
 
  setupMPU6050(); 
} 
 
/* 
   LOOP  
*/ 
 
void loop() { 
 
  // jei suveike judesio pertrauka 
  if (motionIRQ && !alarmActive && !analyzingMotion) { 
 
    // apsauga nuo per dažnu suveikimu 
    if (millis() - lastMotionTime < MOTION_COOLDOWN_MS) { 
      motionIRQ = false; 
      clearMPUInt(); 
      return; 
    } 
 
    analyzingMotion = true; 
    motionIRQ = false; 
    lastMotionTime = millis(); 
    clearMPUInt(); 
 
 
    int16_t ax, ay, az; 
    int16_t pax, pay, paz; 
    readAccel(pax, pay, paz); 
 
    long movementSum = 0; 
    bool firstSample = true; 
 
    unsigned long start = millis(); 
 
    while (millis() - start < ANALYZE_TIME_MS) { 
      delay(SAMPLE_DELAY_MS); 
      readAccel(ax, ay, az); 
 
      long delta = abs(ax - pax) + abs(ay - pay) + abs(az - paz); 
 
      if (!firstSample) { 
        movementSum += delta; 
         
      } else { 
        firstSample = false; 
      } 
 
      pax = ax; pay = ay; paz = az; 
    } 
 
     
 
    if (movementSum > MOVEMENT_SUM_LIMIT) { 
      
      sendSMS("GALIMA VAGYSTE!\nAptiktas judesys."); 
      alarmActive = true; 
    } 
 
    analyzingMotion = false; 
  } 
 
  // jei aliarmas aktyvus – laukiam SMS komandu 
  if (alarmActive && millis() - lastSMSCheck > 5000) { 
    lastSMSCheck = millis(); 
    SmsCmd cmd = readSmsCommand(); 
 
    if (cmd == CMD_RESET) { 
      alarmActive = false; 
      gpsRequested = false; 
      gpsSearching = false; 
      digitalWrite(GPS_STANDBY_PIN, GPS_STANDBY); 
    } 
 
    if (cmd == CMD_GPS && !gpsRequested) { 
      gpsStartSearch(); 
    } 
  } 
 
  // GPS paieška 
  if (gpsSearching) { 
    while (gpsSerial.available()) { 
      gps.encode(gpsSerial.read()); 
    } 
 
     
 
    if (gps.location.isValid()) { 
      String msg = "GPS LOKACIJA\n"; 
      msg += String(gps.location.lat(), 6); 
      msg += " "; 
      msg += String(gps.location.lng(), 6); 
 
      sendSMS(msg); 
      gpsStop(); 
    } 
 
    if (millis() - gpsStart > GPS_FIX_TIMEOUT) { 
      sendSMS("GPS FIX nerastas"); 
      gpsStop(); 
    } 
  } 
}
```
