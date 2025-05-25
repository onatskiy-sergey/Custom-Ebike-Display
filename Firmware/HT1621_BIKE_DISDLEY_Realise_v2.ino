/***********************************************************
 * Пример «полного» кода, где:
 *  - Продвинутый расчёт скорости (micros),
 *  - dailyDistance / totalOdometerKm,
 *  - внешняя EEPROM (0x57),
 *  - 10-секундный стартовый экран,
 *  - команды SETTIME / RESETTOTAL,
 *  - кнопки (лев/прав/фара) + буззер,
 *  - Rob Tillaart ADS1X15 (адрес 0x48),
 *  - В measureTemperatureRaw() - формула Beta + offset,
 *  - Стрелки скорости «накапливающиеся»,
 *  - Светодиод на PIN 13 мигает (70 мс каждые 2 s),
 *  - Эмблема «!» (Addr=15,Bit=0) мигает 300 мс on /300 мс off, если Volt <= 61В.
 ***********************************************************/

#include <Wire.h>
#include <RTClib.h>     
#include <HT1621.h>
#include <EEPROM.h>    
#include "ADS1X15.h"   

//--------------------------------------------------
// Подключение HT1621 (LCD)
#define PIN_CS    10
#define PIN_WR    9
#define PIN_DATA  8
HT1621 lcd(PIN_CS, PIN_WR, PIN_DATA);

//--------------------------------------------------
// Пины
#define PIN_HALL        2  
#define PIN_BUZZER      3
#define PIN_LIGHT       4  
#define PIN_TURN_LEFT   6  
#define PIN_TURN_RIGHT  5  
#define LED_PIN         13 

//--------------------------------------------------
// Аналоговые входы (не удаляем, но можем использовать иначе)
#define PIN_VOLTAGE  A1
#define PIN_TEMP     A2

//--------------------------------------------------
// Переменные (пробег, скорость и т.д.)
volatile unsigned long hallPulseCount = 0; 
float totalOdometerKm   = 0.0;
float dailyDistance     = 0.0;

const float WHEEL_CIRCUM_km = 0.00193; 
const int   PULSES_PER_REV  = 23;

float currentSpeed = 0.0; 
int   currentTemp  = 0;   
float currentVolt  = 0.0; 
int   hoursRTC     = 0;
int   minutesRTC   = 0;
int   secondsRTC   = 0;

bool  dailyResetDone = false;

// 10-сек старт
bool showStartupTotal = true;
unsigned long startupTimer = 0;
bool firstStartupClearDone = false;

RTC_DS3231 rtc;
uint8_t addrBuffer[16];

//--------------------------------------------------
// ADS1115
ADS1115 ads(0x48); 
// при необходимости ads.setGain(…);

//--------------------------------------------------
// EEPROM (0x57)
const int ADDR_DAILY_DISTANCE = 0;
const int ADDR_TOTAL_DISTANCE = 4;
#define ADDR_DATE_YEAR   8
#define ADDR_DATE_MONTH 10
#define ADDR_DATE_DAY   11

//---------------------------------------------
void writeFloatEEPROM(int eepromAddr, float val)
{
  byte* p = (byte*)&val; 
  for (int i=0; i<4; i++) {
    Wire.beginTransmission(0x57);
    Wire.write((byte)((eepromAddr >> 8) & 0xFF)); 
    Wire.write((byte)( eepromAddr       & 0xFF));
    Wire.write(p[i]);
    Wire.endTransmission();
    eepromAddr++;
    delay(5);
  }
}
float readFloatEEPROM(int eepromAddr)
{
  float val = 0.0;
  byte* p = (byte*)&val;
  for (int i=0; i<4; i++){
    Wire.beginTransmission(0x57);
    Wire.write((byte)((eepromAddr >> 8) & 0xFF)); 
    Wire.write((byte)( eepromAddr       & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom((int)0x57, (int)1);
    if (Wire.available()) {
      p[i] = Wire.read();
    } else {
      p[i] = 0;
    }
    eepromAddr++;
  }
  return val;
}

void writeDateToEEPROM(uint16_t year, uint8_t month, uint8_t day)
{
  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_YEAR >> 8) & 0xFF));
  Wire.write((uint8_t)(ADDR_DATE_YEAR & 0xFF));
  Wire.write((uint8_t)(year >> 8));
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_YEAR + 1) >> 8));
  Wire.write((uint8_t)((ADDR_DATE_YEAR + 1) & 0xFF));
  Wire.write((uint8_t)(year & 0xFF));
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_MONTH >> 8) & 0xFF));
  Wire.write((uint8_t)(ADDR_DATE_MONTH & 0xFF));
  Wire.write(month);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_DAY >> 8) & 0xFF));
  Wire.write((uint8_t)(ADDR_DATE_DAY & 0xFF));
  Wire.write(day);
  Wire.endTransmission();
  delay(5);
}

void readDateFromEEPROM(uint16_t *year, uint8_t *month, uint8_t *day)
{
  *year = 2000; *month = 1; *day = 1;

  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_YEAR >> 8) & 0xFF));
  Wire.write((uint8_t)(ADDR_DATE_YEAR & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(0x57, 2);
  if (Wire.available() >= 2) {
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();
    *year = (uint16_t(yhi) << 8) | ylo;
  }

  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_MONTH >> 8) & 0xFF));
  Wire.write((uint8_t)(ADDR_DATE_MONTH & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(0x57, 1);
  if (Wire.available()) *month = Wire.read();

  Wire.beginTransmission(0x57);
  Wire.write((uint8_t)((ADDR_DATE_DAY >> 8) & 0xFF));
  Wire.write((uint8_t)(ADDR_DATE_DAY & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(0x57, 1);
  if (Wire.available()) *day = Wire.read();
}

void checkDateAndResetDailyIfNeeded()
{
  DateTime now = rtc.now();
  uint16_t y_now = now.year();
  uint8_t  m_now = now.month();
  uint8_t  d_now = now.day();

  uint16_t y_eep;
  uint8_t  m_eep, d_eep;
  readDateFromEEPROM(&y_eep, &m_eep, &d_eep);

  Serial.print("EEPROM date: "); Serial.print(y_eep); Serial.print("-");
  Serial.print(m_eep); Serial.print("-"); Serial.println(d_eep);
  Serial.print("RTC now: "); Serial.print(y_now); Serial.print("-");
  Serial.print(m_now); Serial.print("-"); Serial.println(d_now);

  if (y_now != y_eep || m_now != m_eep || d_now != d_eep) {
    Serial.println("Date changed: RESET dailyDistance");
    dailyDistance = 0.0;
    writeFloatEEPROM(ADDR_DAILY_DISTANCE, dailyDistance);
    writeDateToEEPROM(y_now, m_now, d_now);
  } else {
    Serial.println("Same date — no reset needed.");
  }
}
//--------------------------------------------------
// Функция для сохранения суточного пробега раз в минуту (не удаляем)
int lastWriteMinute = -1;
void storeDailyDistanceOncePerMinute(int currentMin)
{
  if (currentMin != lastWriteMinute) {
    writeFloatEEPROM(ADDR_DAILY_DISTANCE, dailyDistance);
    lastWriteMinute = currentMin;
    Serial.println("storeDailyDistanceOncePerMinute called");
  }
}

//--------------------------------------------------
// Сохраняем dailyDistance & totalOdometerKm каждые 30 сек
void storeDailyDistanceEvery30Sec(int currentSec)
{
  static int prevHalfMin = -1;
  int halfMin = currentSec / 10;  
  if (halfMin != prevHalfMin) {
    writeFloatEEPROM(ADDR_DAILY_DISTANCE, dailyDistance);
    writeFloatEEPROM(ADDR_TOTAL_DISTANCE, totalOdometerKm);

    Serial.print("Saved dailyDistance & totalOdometer in EXT EEPROM(10s): ");
    Serial.print(dailyDistance, 1);
    Serial.print(" / ");
    Serial.println(totalOdometerKm, 1);

    prevHalfMin = halfMin;
  }
}

//--------------------------------------------------
// Продвинутый расчёт скорости (micros)
volatile unsigned long lastPulseMicros=0; 
volatile unsigned long pulseInterval=0; 
volatile unsigned long lastPulseTimeMs=0;

void hallInterrupt()
{
  unsigned long nowUs=micros();
  if(lastPulseMicros!=0){
    unsigned long interval= nowUs - lastPulseMicros;
    pulseInterval=interval; 
  }
  lastPulseMicros= nowUs;
  lastPulseTimeMs= millis();

  dailyDistance   += (WHEEL_CIRCUM_km / (float)PULSES_PER_REV);
  totalOdometerKm += (WHEEL_CIRCUM_km / (float)PULSES_PER_REV);

  hallPulseCount++;
}

//--------------------------------------------------
// measureTemperatureRaw(...) - Beta формула + offset
int measureTemperatureRaw(int ignored)
{
  int16_t raw = ads.readADC(0);
  float v = ads.toVoltage(raw);

  float Rfix = 10000.0;  
  float Vcc  = 5.0;      
  float Rntc = Rfix * (Vcc - v) / v;

  const float Beta = 3950.0;  
  const float R0   = 10000.0;  
  const float T0   = 298.15;   

  float invT = (1.0 / T0) + (1.0 / Beta)*logf(Rntc / R0);
  float TK   = 1.0 / invT; 
  float TC   = TK - 273.15;

  // Калибровочный сдвиг
  float offset = -12.0;
  TC += offset;

  return (int)(TC + 0.5); 
}

//--------------------------------------------------
// measureVoltage
float measureVoltage()
{
  int16_t raw = ads.readADC(1);
  float vAtPin = ads.toVoltage(raw);

  const float VOLTAGE_DIVIDER_K = 14.0;
  float battery = vAtPin * VOLTAGE_DIVIDER_K;
  return battery;
}

//--------------------------------------------------
// Управление HT1621
void clearAll()
{
  for (int i=0; i<16; i++){
    addrBuffer[i] = 0;
    lcd.write(i, 0);
  }
}
void setSegment(int addr,int bitNum,bool on)
{
  if (addr<0||addr>15||bitNum<0||bitNum>7) return;
  if(on) addrBuffer[addr]|=(1<<bitNum);
  else   addrBuffer[addr]&=~(1<<bitNum);
  lcd.write(addr,addrBuffer[addr]);
}

const uint8_t digitToSegments[10]={
  0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
  0b01101101,0b01111101,0b00000111,0b01111111,0b01101111
};

struct SegmentAddrBit {
  int addr; 
  int bit;
};

//--------------------------------------------------
// showDigit(...) - универсальный показ 7сег
void showDigit(uint8_t digit,SegmentAddrBit mapping[8],bool dpOn=false)
{
  if(digit>9)return;
  uint8_t mask=digitToSegments[digit];
  for(int i=0;i<7;i++){
    int a=mapping[i].addr; 
    int b=mapping[i].bit;
    if(a<0||b<0)continue;
    bool on=((mask&(1<<i))!=0);
    setSegment(a,b,on);
  }
  int da=mapping[7].addr; 
  int db=mapping[7].bit;
  if(da>=0&&db>=0){setSegment(da,db,dpOn);}
}

//--------------------------------------------------
// showOdometer(...)
SegmentAddrBit odometerDigit1[8]={{0,7},{4,2},{4,1},{4,0},{0,4},{0,6},{0,5},{-1,-1}};
SegmentAddrBit odometerDigit2[8]={{4,7},{5,6},{5,5},{5,4},{4,4},{4,6},{4,5},{5,7}};
SegmentAddrBit odometerDigit3[8]={{5,3},{6,6},{6,5},{6,4},{5,0},{5,2},{5,1},{-1,-1}};
void showOdometer(int value)
{
  if(value<0)value=0;if(value>999)value=999;
  int d1=value/100;int d2=(value/10)%10;int d3=value%10;
  showDigit(d1,odometerDigit1);
  showDigit(d2,odometerDigit2,false);
  showDigit(d3,odometerDigit3,false);
  setSegment(6,0,true); // "km"
}

//--------------------------------------------------
// showDailyDistance(...)
void showDailyDistance(float dist)
{
  if(dist<0)dist=0;if(dist>999.0)dist=999.0;
  int d1,d2,d3;bool dp2=false;
  if(dist<100.0){
    int ip=(int)dist;
    int dec=(int)((dist-ip)*10+0.5);
    d1=ip/10;d2=ip%10;d3=dec;
    dp2=true;
  }else{
    int val=(int)(dist+0.5);
    if(val>999)val=999;
    d1=val/100;d2=(val/10)%10;d3=val%10;dp2=false;
  }
  showDigit(d1,odometerDigit1,false);
  showDigit(d2,odometerDigit2,dp2);
  showDigit(d3,odometerDigit3,false);
  setSegment(6,0,true);
}

//--------------------------------------------------
// showSpeed(...)
#define ADDR_SPEED1_BC 7
#define BIT_SPEED1_BC  7
SegmentAddrBit speedDigit2[8]={{7,3},{7,2},{7,1},{7,0},{7,4},{7,6},{7,5},{-1,-1}};
SegmentAddrBit speedDigit3[8]={{8,3},{8,2},{8,1},{8,0},{8,4},{8,6},{8,5},{-1,-1}};
void showSpeed(float spd)
{
  if(spd<0)spd=0;if(spd>199.0f)spd=199.0f;
  int speedInt=(int)spd;
  bool over99=(speedInt>=100);
  setSegment(ADDR_SPEED1_BC,BIT_SPEED1_BC,over99);
  int twoDigitVal= over99?(speedInt-100):speedInt;
  if(twoDigitVal<0)twoDigitVal=0;if(twoDigitVal>99)twoDigitVal=99;
  int d2=twoDigitVal/10;int d3=twoDigitVal%10;
  showDigit(d2,speedDigit2);
  showDigit(d3,speedDigit3);
  setSegment(12,6,true); // "km/h"
}

//--------------------------------------------------
// showVoltage(...)
#define ADDR_VOLT_DIGIT1_BC 12
#define BIT_VOLT_DIGIT1_BC  4
SegmentAddrBit voltDigit2[8]={{12,3},{13,6},{13,5},{13,4},{12,0},{12,2},{12,1},{-1,-1}};
SegmentAddrBit voltDigit3[8]={{13,3},{14,6},{14,5},{14,4},{13,0},{13,2},{13,1},{-1,-1}};
SegmentAddrBit voltDigit4[8]={{14,3},{15,6},{15,5},{15,4},{14,0},{14,2},{14,1},{-1,-1}};
#define ADDR_VOLT_POINT 15
#define BIT_VOLT_POINT  7
void showVoltage(float volts)
{
  bool over99=(volts>99.9f);
  setSegment(ADDR_VOLT_DIGIT1_BC,BIT_VOLT_DIGIT1_BC,over99);
  if(volts>150.0f)volts=150.0f;
  int iv=(int)(volts*10+0.5f);
  int d2=(iv/100)%10;int d3=(iv/10)%10;int d4=iv%10;
  showDigit(d2,voltDigit2,false);
  showDigit(d3,voltDigit3,true);
  showDigit(d4,voltDigit4,false);
  setSegment(ADDR_VOLT_POINT,BIT_VOLT_POINT,true);
}

//--------------------------------------------------
// showTemperature(...)
#define ADDR_TEMP1_BC   9
#define BIT_TEMP1_BC    3
#define ADDR_TEMP_MINUS 10
#define BIT_TEMP_MINUS  3
SegmentAddrBit tempDigit2[8]={{10,4},{10,5},{10,6},{10,7},{9,2},{9,0},{9,1},{-1,-1}};
SegmentAddrBit tempDigit3[8]={{11,4},{11,5},{11,6},{11,7},{10,2},{10,0},{10,1},{-1,-1}};
void showTemperature(int tempC)
{
  bool negative=(tempC<0);
  if(tempC<-9)tempC=-9;if(tempC>99)tempC=99;
  bool showOne=(tempC>=100||tempC<=-100); 
  setSegment(ADDR_TEMP1_BC,BIT_TEMP1_BC,showOne);
  int absT= negative?(-tempC):tempC;
  int d2=absT/10;int d3=absT%10;
  showDigit(d2,tempDigit2);
  showDigit(d3,tempDigit3);
  setSegment(ADDR_TEMP_MINUS,BIT_TEMP_MINUS,negative);
  setSegment(11,0,true); // °C
}

//--------------------------------------------------
// showTime(...)
#define ADDR_TIME_COLON 3
#define BIT_TIME_COLON  3
SegmentAddrBit timeDigit1[8]={{3,2},{2,3},{3,1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
SegmentAddrBit timeDigit2[8]={{3,7},{2,2},{2,1},{2,0},{3,4},{3,6},{3,5},{-1,-1}};
SegmentAddrBit timeDigit3[8]={{2,7},{1,2},{1,1},{1,0},{2,4},{2,6},{2,5},{-1,-1}};
SegmentAddrBit timeDigit4[8]={{1,7},{0,2},{0,1},{0,0},{1,4},{1,6},{1,5},{-1,-1}};
void showTime(int hh,int mm)
{
  if(hh<0)hh=0;if(hh>23)hh=23;
  if(mm<0)mm=0;if(mm>59)mm=59;

  int h1=hh/10;int h2=hh%10;int m1=mm/10;int m2=mm%10;

  if(h1==0&&hh<10){
    for(int i=0;i<8;i++){
      if(timeDigit1[i].addr>=0&&timeDigit1[i].bit>=0){
        setSegment(timeDigit1[i].addr,timeDigit1[i].bit,false);
      }
    }
  }else{
    showDigit(h1,timeDigit1,false);
  }
  showDigit(h2,timeDigit2);
  showDigit(m1,timeDigit3);
  showDigit(m2,timeDigit4);
  setSegment(ADDR_TIME_COLON,BIT_TIME_COLON,true);
}

//--------------------------------------------------
// Иконки (лев/прав, фара)
#define ADDR_LTURN  9
#define BIT_LTURN   4
#define ADDR_RTURN  11
#define BIT_RTURN   3
#define ADDR_LIGHT  11
#define BIT_LIGHT   2
void setLeftTurn(bool on)
{
  setSegment(ADDR_LTURN,BIT_LTURN,on);
}
void setRightTurn(bool on)
{
  setSegment(ADDR_RTURN,BIT_RTURN,on);
}
void setHeadLight(bool on)
{
  setSegment(ADDR_LIGHT,BIT_LIGHT,on);
}

//--------------------------------------------------
// Стрелки скорости (накопление)
void updateSpeedArrows(int spd)
{
  setSegment(1,3,false);
  setSegment(0,3,false);
  setSegment(8,7,false);
  setSegment(12,7,false);
  setSegment(14,7,false);

  if (spd >= 10)  setSegment(1,3,true);   
  if (spd >= 20)  setSegment(0,3,true);   
  if (spd >= 40)  setSegment(8,7,true);   
  if (spd >= 60)  setSegment(12,7,true);  
  if (spd >= 90)  setSegment(14,7,true);  
}

//--------------------------------------------------
// Проверка команд SETTIME / RESETTOTAL
void checkSerialForTimeCommand()
{
  if(Serial.available()>0){
    String line=Serial.readStringUntil('\n');
    line.trim();
    Serial.print("Serial command input: [");
    Serial.print(line);
    Serial.println("]");
    if(line.startsWith("SETTIME")){
      String args=line.substring(7);
      args.trim();
      int vals[6]; 
      int i=0;
      char* cstr= const_cast<char*>(args.c_str());
      char* token= strtok(cstr," ");
      while(token!=NULL&&i<6){
        vals[i]=atoi(token);
        token=strtok(NULL," ");
        i++;
      }
      if(i==6){
        int yr=vals[0]; 
        int mon=vals[1]; 
        int day=vals[2];
        int hr=vals[3]; 
        int mn=vals[4]; 
        int sc=vals[5];
        rtc.adjust(DateTime(yr,mon,day,hr,mn,sc));
        Serial.print("Time set to: ");
        Serial.print(yr);Serial.print("/");
        Serial.print(mon);Serial.print("/");
        Serial.print(day);Serial.print(" ");
        Serial.print(hr);Serial.print(":");
        Serial.print(mn);Serial.print(":");
        Serial.println(sc);
      }else{
        Serial.println("Invalid SETTIME format. Use: SETTIME YYYY MM DD HH mm ss");
      }
    }
    else if(line.startsWith("RESETTOTAL")){
      totalOdometerKm=0.0;
      writeFloatEEPROM(ADDR_TOTAL_DISTANCE,totalOdometerKm);
      Serial.println("Total odometer reset to 0!");
    }
    else{
      Serial.println("Unknown command (use SETTIME or RESETTOTAL)!");
    }
  }
}

//--------------------------------------------------
// Сброс суточного в полночь
void resetDailyDistanceAtMidnight(int hh,int mm)
{
  if(hh==0&&mm==0){
    if(!dailyResetDone){
      dailyDistance=0.0;
      Serial.println("Daily distance RESET at midnight!");
      writeFloatEEPROM(ADDR_DAILY_DISTANCE,dailyDistance);
      dailyResetDone=true;
    }
  }else{
    dailyResetDone=false;
  }
}

//--------------------------------------------------
// Основное обновление дисплея
void updateDisplay()
{
  showDailyDistance(dailyDistance);
  showSpeed(currentSpeed);
  showVoltage(currentVolt);
  showTemperature(currentTemp);
  showTime(hoursRTC,minutesRTC);
  updateSpeedArrows((int)currentSpeed);
}

//--------------------------------------------------
// showStartupTotalDistance(...)
SegmentAddrBit totalDistDigit1[8]={{3,7},{2,2},{2,1},{2,0},{3,4},{3,6},{3,5},{-1,-1}};
SegmentAddrBit totalDistDigit2[8]={{2,7},{1,2},{1,1},{1,0},{2,4},{2,6},{2,5},{-1,-1}};
SegmentAddrBit totalDistDigit3[8]={{1,7},{0,2},{0,1},{0,0},{1,4},{1,6},{1,5},{-1,-1}};
SegmentAddrBit totalDistDigit4[8]={{0,7},{4,2},{4,1},{4,0},{0,4},{0,6},{0,5},{-1,-1}};
SegmentAddrBit totalDistDigit5[8]={{4,7},{5,6},{5,5},{5,4},{4,4},{4,6},{4,5},{5,7}};
SegmentAddrBit totalDistDigit6[8]={{5,3},{6,6},{6,5},{6,4},{5,0},{5,2},{5,1},{-1,-1}};
void showStartupTotalDistance(float totalVal)
{
  if(totalVal<0)totalVal=0;
  if(totalVal>999999.9)totalVal=999999.9;
  unsigned long x=(unsigned long)(totalVal*10+0.5);
  int d6=x%10;x/=10;int d5=x%10;x/=10;
  int d4=x%10;x/=10;int d3=x%10;x/=10;
  int d2=x%10;x/=10;int d1=x%10;x/=10;
  int d0=x%10;
  showDigit(d0,totalDistDigit1,false);
  showDigit(d1,totalDistDigit2,false);
  showDigit(d2,totalDistDigit3,false);
  showDigit(d3,totalDistDigit4,false);
  showDigit(d4,totalDistDigit5,false);
  showDigit(d5,totalDistDigit6,false);
  setSegment(6,0,true);
}

//--------------------------------------------------
// Логика строба LED + логика мигания "!" (Addr=15,Bit=0)
static bool strobeActive    = false; 
static unsigned long lastStrobeTime = 0;
const unsigned long STROBE_ON_DURATION = 70;   
const unsigned long STROBE_INTERVAL    = 2000;  

// Для "!" мигания 300/300:
static bool exclMarkState   = false; 
static unsigned long lastExclTime   = 0;
const unsigned long EXCL_BLINK_TIME = 300;  // 300 on, 300 off => 600 total

//--------------------------------------------------
// setup()  <-- важно для Arduino
void setup()
{
  Serial.begin(9600);
  Serial.println("=== Full code with Beta formula in measureTemperatureRaw() + offset ===");

  pinMode(PIN_BUZZER,OUTPUT); digitalWrite(PIN_BUZZER,LOW);
  pinMode(PIN_LIGHT, INPUT_PULLUP);
  pinMode(PIN_TURN_LEFT, INPUT_PULLUP);
  pinMode(PIN_TURN_RIGHT,INPUT_PULLUP);
  
  pinMode(LED_PIN, OUTPUT);  

  pinMode(PIN_HALL,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL),hallInterrupt,RISING);

  pinMode(PIN_VOLTAGE,INPUT);
  pinMode(PIN_TEMP,   INPUT);

  Wire.begin();
  if(!rtc.begin()){
    Serial.println("RTC not found!");
  }else{
    if(rtc.lostPower()){
      rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
      Serial.println("RTC lost power, set compile time!");
    }
  }

  // Инициализация HT1621
  lcd.begin();
  lcd.sendCommand(HT1621::BIAS_THIRD_4_COM); 
  lcd.sendCommand(HT1621::RC256K); 
  lcd.sendCommand(HT1621::SYS_DIS); 
  lcd.sendCommand(HT1621::WDT_DIS); 
  lcd.sendCommand(HT1621::SYS_EN); 
  lcd.sendCommand(HT1621::LCD_ON);
  lcd.clear();
  for(int i=0;i<16;i++) addrBuffer[i]=0;

  // Инициализация ADS1115
  ads.begin();  

  dailyDistance   = readFloatEEPROM(ADDR_DAILY_DISTANCE);
  totalOdometerKm = readFloatEEPROM(ADDR_TOTAL_DISTANCE);
  Serial.print("Loaded dailyDistance: ");Serial.println(dailyDistance,1);
  Serial.print("Loaded totalOdometerKm: ");Serial.println(totalOdometerKm,1);

  startupTimer=millis();
  showStartupTotal=true;
  firstStartupClearDone=false;
  checkDateAndResetDailyIfNeeded();
}

//--------------------------------------------------
// loop()  <-- важно для Arduino
void loop()
{
  checkSerialForTimeCommand();

  unsigned long nowMs=millis();

  //----- Скорость -----
  if((nowMs - lastPulseTimeMs)>1500){
    currentSpeed=0.0;
  }else{
    if(pulseInterval>0){
      float freqImp=1000000.0f/(float)pulseInterval;
      float freqRev=freqImp/(float)PULSES_PER_REV;
      float kmps=freqRev*WHEEL_CIRCUM_km;
      currentSpeed=kmps*3600.0f;
    }
  }

  //----- Раз в 1 c => RTC, temp, voltage -----
  static unsigned long lastMisc=0;
  if((nowMs - lastMisc)>=1000){
    lastMisc=nowMs;

    DateTime t=rtc.now();
    hoursRTC   = t.hour();
    minutesRTC = t.minute();
    secondsRTC = t.second();

    currentTemp = measureTemperatureRaw( analogRead(PIN_TEMP) );
    currentVolt = measureVoltage();

    resetDailyDistanceAtMidnight(hoursRTC,minutesRTC);
    storeDailyDistanceEvery30Sec(secondsRTC);
  }

  //----- 10 c стартовый экран -----
  if(showStartupTotal){
    if(!firstStartupClearDone){
      clearAll();
      firstStartupClearDone=true;
    }
    if((nowMs - startupTimer)<10000UL){
      showStartupTotalDistance(totalOdometerKm);
    }else{
      showStartupTotal=false;
      clearAll();
    }
  }else{
    // Обычный режим
    updateDisplay();
  }

  //----- Кнопки => буззер -----
  bool leftPressed  =(digitalRead(PIN_TURN_LEFT)==LOW);
  bool rightPressed =(digitalRead(PIN_TURN_RIGHT)==LOW);
  bool lightPressed =(digitalRead(PIN_LIGHT)==LOW);

  setLeftTurn(leftPressed);
  setRightTurn(rightPressed);
  setHeadLight(lightPressed);

  if(leftPressed||rightPressed){
    digitalWrite(PIN_BUZZER,HIGH);
  }else{
    digitalWrite(PIN_BUZZER,LOW);
  }

  //----- 1) Логика LED_PIN строб (70ms /2s) -----
  if(!strobeActive && (nowMs - lastStrobeTime >= STROBE_INTERVAL)){
    // вкл LED
    digitalWrite(LED_PIN, HIGH);
    strobeActive = true;
    lastStrobeTime = nowMs;
  }else if(strobeActive && (nowMs - lastStrobeTime >= STROBE_ON_DURATION)){
    // выкл LED
    digitalWrite(LED_PIN, LOW);
    strobeActive = false;
  }

  //----- 2) Логика мигания эмблемы "!" (Addr=15,Bit=0) каждые 300ms -----
  static unsigned long lastExclTime = 0;
  static bool exclMarkState = false;

  bool needBlink = (currentVolt <= 61.0f);

  if(!needBlink){
    // если напряжение >61 => всегда off
    exclMarkState = false;
  }else{
    // напряжение <=61 => мигаем 300ms on /300ms off
    if((nowMs - lastExclTime) >= EXCL_BLINK_TIME){
      lastExclTime = nowMs;
      exclMarkState = !exclMarkState; // toggle
    }
  }

  // Устанавливаем сегмент "!" 
  setSegment(15,0, exclMarkState);

  // (конец loop)
}
