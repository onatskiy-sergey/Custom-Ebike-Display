#include <Wire.h>
#include <RTClib.h>     // для DS3231
#include <HT1621.h>
#include <EEPROM.h>     // для хранения данных в ПЗУ

/***************************************************
 * Подключение DS3231 (I2C) на Arduino UNO:
 *  SDA -> A4
 *  SCL -> A5
 ***************************************************/

// ---------------- HT1621 -----------------
#define PIN_CS    10
#define PIN_WR    9
#define PIN_DATA  8
HT1621 lcd(PIN_CS, PIN_WR, PIN_DATA);

// ---------------- ДРУГИЕ ПИНЫ -------------
#define PIN_HALL        2  // геркон/датчик Холла (прерывание)
#define PIN_BUZZER      3
#define PIN_LIGHT       4
#define PIN_TURN_LEFT   6
#define PIN_TURN_RIGHT  5

// Аналоговые входы
#define PIN_VOLTAGE  A1
#define PIN_TEMP     A2

// ------------------------------------------------
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (из старых версий)
// ------------------------------------------------

// Общий пробег (если нужен):
volatile unsigned long hallPulseCount = 0; 
unsigned long lastHallCount   = 0;         
float totalOdometerKm         = 0.0;    

// **Суточный** пробег (до 99.9 с точкой, свыше 100 — без)
float dailyDistance           = 0.0;  

// Параметры колеса:
const float WHEEL_CIRCUM_km = 0.00193; 
const int   PULSES_PER_REV  = 1;      

// Период обновления скорости (1 сек)
unsigned long lastSpeedUpdate    = 0;
const unsigned long SPEED_PERIOD = 1000;

// Текущее состояние
float currentSpeed = 0.0; 
int   currentTemp  = 0;   
float currentVolt  = 0.0; 
int   hoursRTC     = 0;
int   minutesRTC   = 0;
int   secondsRTC   = 0;

// RTC
RTC_DS3231 rtc;

// Локальный буфер для HT1621 (Addr=0..15)
uint8_t addrBuffer[16];

// Делитель напряжения
const float VOLTAGE_DIVIDER_K =1.4; 

// Флаг сброса dailyDistance в полночь
bool dailyResetDone = false;

/************************************************************
 *  Новые глобальные переменные для "стартового экрана"
 ************************************************************/
bool showStartupTotal = true;      // TRUE = первые 10с показываем общий пробег
unsigned long startupTimer = 0;    // когда включились (millis())

/************************************************
 * EEPROM: адреса и функции для хранения float
 ************************************************/
const int ADDR_DAILY_DISTANCE = 0;  // 4 байта под suточный пробег

void writeFloatEEPROM(int addr, float val)
{
  byte* p = (byte*)(void*)&val; 
  for (int i=0; i<4; i++) {
    EEPROM.update(addr + i, p[i]);
  }
}

float readFloatEEPROM(int addr)
{
  float val = 0.0;
  byte* p = (byte*)(void*)&val;
  for (int i=0; i<4; i++) {
    p[i] = EEPROM.read(addr + i);
  }
  return val;
}

/************************************************
 *  СТАРАЯ функция: сохранение суточного пробега
 *  раз в минуту (не удаляем!)
 ************************************************/
int lastWriteMinute = -1;
void storeDailyDistanceOncePerMinute(int currentMin)
{
  if (currentMin != lastWriteMinute) {
    writeFloatEEPROM(ADDR_DAILY_DISTANCE, dailyDistance);
    lastWriteMinute = currentMin;
    Serial.println("storeDailyDistanceOncePerMinute called");
  }
}

/************************************************
 *  НОВАЯ функция: сохраняем dailyDistance
 *  каждые 30 секунд
 ************************************************/
void storeDailyDistanceEvery30Sec(int currentSec)
{
  static int prevHalfMin = -1;
  int halfMin = currentSec / 30;  
  if (halfMin != prevHalfMin) {
    writeFloatEEPROM(ADDR_DAILY_DISTANCE, dailyDistance);

    Serial.print("Saved dailyDistance in EEPROM (every30sec): ");
    Serial.println(dailyDistance, 1);

    prevHalfMin = halfMin;
  }
}

/************************************************
 *  Прерывание (Холл/геркон)
 ************************************************/
void hallInterrupt()
{
  hallPulseCount++;
}

/************************************************
 *  Измерение температуры (заглушка)
 ************************************************/
int measureTemperatureRaw(int analogValue)
{
  float voltage = (5.0 * analogValue) / 1023.0; 
  float temp = 25.0 + (2.5 - voltage)*(-20.0); 
  return (int)temp;
}

/************************************************
 *  Измерение напряжения
 ************************************************/
float measureVoltage()
{
  int raw = analogRead(PIN_VOLTAGE);
  float voltageAtPin = (5.0 * raw) / 1023.0;
  float battery = voltageAtPin * VOLTAGE_DIVIDER_K;
  return battery;
}

/************************************************
 *  Управление HT1621
 ************************************************/
void clearAll()
{
  for (int i=0; i<16; i++){
    addrBuffer[i] = 0x00;
    lcd.write(i, 0x00);
  }
}

void setSegment(int addr, int bitNum, bool on)
{
  if (addr<0 || addr>15 || bitNum<0 || bitNum>7) return;
  if (on) addrBuffer[addr] |=  (1 << bitNum);
  else    addrBuffer[addr] &= ~(1 << bitNum);
  lcd.write(addr, addrBuffer[addr]);
}

// 7-сегм. цифры A..G
const uint8_t digitToSegments[10] = {
  0b00111111,
  0b00000110,
  0b01011011,
  0b01001111,
  0b01100110,
  0b01101101,
  0b01111101,
  0b00000111,
  0b01111111,
  0b01101111
};

struct SegmentAddrBit {
  int addr;
  int bit;
};

void showDigit(uint8_t digit, SegmentAddrBit mapping[8], bool dpOn = false)
{
  if (digit>9) return;
  uint8_t mask = digitToSegments[digit]; 
  for (int i=0; i<7; i++){
    int a = mapping[i].addr;
    int b = mapping[i].bit;
    if (a<0 || b<0) continue;
    bool on = (mask & (1<<i)) != 0;
    setSegment(a, b, on);
  }
  // Точка
  int da = mapping[7].addr;
  int db = mapping[7].bit;
  if (da>=0 && db>=0) {
    setSegment(da, db, dpOn);
  }
}

/************************************************
 * СТАРАЯ ФУНКЦИЯ showOdometer(int), не удаляем
 ************************************************/
SegmentAddrBit odometerDigit1[8] = {
  {0,7},{4,2},{4,1},{4,0},{0,4},{0,6},{0,5},{-1,-1}
};
SegmentAddrBit odometerDigit2[8] = {
  {4,7},{5,6},{5,5},{5,4},{4,4},{4,6},{4,5},{5,7}
};
SegmentAddrBit odometerDigit3[8] = {
  {5,3},{6,6},{6,5},{6,4},{5,0},{5,2},{5,1},{-1,-1}
};

void showOdometer(int value)
{
  if (value<0)   value=0;
  if (value>999) value=999;
  int d1 = value/100;
  int d2 = (value/10)%10;
  int d3 = value%10;

  showDigit(d1, odometerDigit1);
  showDigit(d2, odometerDigit2, false);
  showDigit(d3, odometerDigit3, false);

  // "km"
  setSegment(6, 0, true);
}

/************************************************
 * НОВАЯ ФУНКЦИЯ: showDailyDistance(float)
 * до 99.9 (с точкой), свыше 100 — без точки
 ************************************************/
void showDailyDistance(float dist)
{
  if (dist < 0) dist = 0;
  if (dist > 999.0) dist = 999.0;  // ограничим

  int d1, d2, d3;
  bool dp2 = false;

  if (dist < 100.0) {
    // формат XX.X
    int integerPart = (int)dist;
    int decimalPart = (int)((dist - integerPart)*10 + 0.5);
    d1 = integerPart / 10;
    d2 = integerPart % 10;
    d3 = decimalPart;
    dp2 = true; 
  } else {
    // dist >= 100 => XXX
    int val = (int)(dist + 0.5);
    if (val>999) val=999;
    d1 = val / 100;
    d2 = (val / 10) % 10;
    d3 = val % 10;
    dp2 = false;
  }

  showDigit(d1, odometerDigit1, false);
  showDigit(d2, odometerDigit2, dp2);
  showDigit(d3, odometerDigit3, false);

  // "km"
  setSegment(6, 0, true);
}

/************************************************
 * Скорость, Вольты, Темп, Часы — не меняем
 ************************************************/
#define ADDR_SPEED1_BC  7
#define BIT_SPEED1_BC   7
SegmentAddrBit speedDigit2[8] = {
  {7,3},{7,2},{7,1},{7,0},{7,4},{7,6},{7,5},{-1,-1}
};
SegmentAddrBit speedDigit3[8] = {
  {8,3},{8,2},{8,1},{8,0},{8,4},{8,6},{8,5},{-1,-1}
};
void showSpeed(float spd)
{
  if (spd<0) spd=0;
  bool over99 = (spd>99);
  setSegment(ADDR_SPEED1_BC, BIT_SPEED1_BC, over99);

  int speedInt = (int)spd;
  if (speedInt>99) speedInt=99;
  int d2 = speedInt/10;
  int d3 = speedInt%10;
  showDigit(d2, speedDigit2);
  showDigit(d3, speedDigit3);

  setSegment(12, 6, true); // km/h
}

// Вольтметр (4 цифры + V)
#define ADDR_VOLT_DIGIT1_BC 12
#define BIT_VOLT_DIGIT1_BC  4
SegmentAddrBit voltDigit2[8] = {
  {12,3},{13,6},{13,5},{13,4},{12,0},{12,2},{12,1},{-1,-1}
};
SegmentAddrBit voltDigit3[8] = {
  {13,3},{14,6},{14,5},{14,4},{13,0},{13,2},{13,1},{-1,-1}
};
SegmentAddrBit voltDigit4[8] = {
  {14,3},{15,6},{15,5},{15,4},{14,0},{14,2},{14,1},{-1,-1}
};
#define ADDR_VOLT_POINT 15
#define BIT_VOLT_POINT  7
void showVoltage(float volts)
{
  bool over99 = (volts>99.9);
  setSegment(ADDR_VOLT_DIGIT1_BC, BIT_VOLT_DIGIT1_BC, over99);

  if (volts>150.0) volts=150.0;
  int iv = (int)(volts*10 + 0.5);
  int d2 = (iv/100)%10;
  int d3 = (iv/10)%10;
  int d4 = iv%10;
  showDigit(d2, voltDigit2, false);
  showDigit(d3, voltDigit3, true);
  showDigit(d4, voltDigit4, false);

  setSegment(ADDR_VOLT_POINT, BIT_VOLT_POINT, true);
}

// Температура (3 символа + °C)
#define ADDR_TEMP1_BC   9
#define BIT_TEMP1_BC    3
#define ADDR_TEMP_MINUS 10
#define BIT_TEMP_MINUS  3
SegmentAddrBit tempDigit2[8] = {
  {10,4},{10,5},{10,6},{10,7},{9,2},{9,0},{9,1},{-1,-1}
};
SegmentAddrBit tempDigit3[8] = {
  {11,4},{11,5},{11,6},{11,7},{10,2},{10,0},{10,1},{-1,-1}
};
void showTemperature(int tempC)
{
  bool negative = (tempC<0);
  if (tempC < -9)  tempC = -9;
  if (tempC > 99)  tempC = 99;

  bool showOne = (tempC>=10 || tempC<=-10);
  setSegment(ADDR_TEMP1_BC, BIT_TEMP1_BC, showOne);

  int absT = negative ? -tempC : tempC;
  int d2 = absT/10;
  int d3 = absT%10;
  showDigit(d2, tempDigit2);
  showDigit(d3, tempDigit3);

  setSegment(ADDR_TEMP_MINUS, BIT_TEMP_MINUS, negative);
  setSegment(11, 0, true); // °C
}

// Часы (4 цифры HH:MM + двоеточие)
SegmentAddrBit timeDigit1[8] = {
  {3,2},{2,3},{3,1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}
};
SegmentAddrBit timeDigit2[8] = {
  {3,7},{2,2},{2,1},{2,0},{3,4},{3,6},{3,5},{-1,-1}
};
SegmentAddrBit timeDigit3[8] = {
  {2,7},{1,2},{1,1},{1,0},{2,4},{2,6},{2,5},{-1,-1}
};
SegmentAddrBit timeDigit4[8] = {
  {1,7},{0,2},{0,1},{0,0},{1,4},{1,6},{1,5},{-1,-1}
};
#define ADDR_TIME_COLON 3
#define BIT_TIME_COLON  3
void showTime(int hh, int mm)
{
  if (hh<0) hh=0; if (hh>23) hh=23;
  if (mm<0) mm=0; if (mm>59) mm=59;

  int h1 = hh/10;
  int h2 = hh%10;
  int m1 = mm/10;
  int m2 = mm%10;

  if (h1 == 0 && hh < 10) {
    // "гасим" первый символ
    for (int i=0; i<8; i++){
      if (timeDigit1[i].addr>=0 && timeDigit1[i].bit>=0) {
        setSegment(timeDigit1[i].addr, timeDigit1[i].bit, false);
      }
    }
  } else {
    showDigit(h1, timeDigit1, false);
  }

  showDigit(h2, timeDigit2);
  showDigit(m1, timeDigit3);
  showDigit(m2, timeDigit4);

  setSegment(ADDR_TIME_COLON, BIT_TIME_COLON, true);
}

// Иконки (поворотники, фара)
#define ADDR_LTURN  9
#define BIT_LTURN   4
#define ADDR_RTURN  11
#define BIT_RTURN   3
#define ADDR_LIGHT  11
#define BIT_LIGHT   2
void setLeftTurn(bool on)
{
  digitalWrite(PIN_TURN_LEFT, on ? HIGH : LOW);
  setSegment(ADDR_LTURN, BIT_LTURN, on);
}
void setRightTurn(bool on)
{
  digitalWrite(PIN_TURN_RIGHT, on ? HIGH : LOW);
  setSegment(ADDR_RTURN, BIT_RTURN, on);
}
void setHeadLight(bool on)
{
  digitalWrite(PIN_LIGHT, on ? HIGH : LOW);
  setSegment(ADDR_LIGHT, BIT_LIGHT, on);
}

// Стрелки скорости (пример)
void updateSpeedArrows(int spd)
{
  setSegment(1,3,false);
  setSegment(0,3,false);
  setSegment(8,7,false);
  setSegment(12,7,false);
  setSegment(14,7,false);

  if (spd>=10 && spd<=20)   setSegment(1,3,true);
  if (spd>=30 && spd<=40)   setSegment(0,3,true);
  if (spd>=50 && spd<=60)   setSegment(8,7,true);
  if (spd>=70 && spd<=90)   setSegment(12,7,true);
  if (spd>=110 && spd<=130) setSegment(14,7,true);
}

/************************************************
 *  Установка времени по Serial:
 *    SETTIME YYYY MM DD HH mm ss
 ************************************************/
void checkSerialForTimeCommand()
{
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("SETTIME")) {
      // Пример: SETTIME 2025 1 27 15 30 00
      String args = line.substring(7);
      args.trim();
      int vals[6];
      int i=0;
      char * cstr = const_cast<char*>(args.c_str());
      char * token = strtok(cstr, " ");
      while (token != NULL && i<6) {
        vals[i] = atoi(token);
        token = strtok(NULL, " ");
        i++;
      }
      if (i==6) {
        int yr  = vals[0];
        int mon = vals[1];
        int day = vals[2];
        int hr  = vals[3];
        int mn  = vals[4];
        int sc  = vals[5];
        // Установим
        rtc.adjust(DateTime(yr, mon, day, hr, mn, sc));
        Serial.print("Time set to: ");
        Serial.print(yr);  Serial.print("/");
        Serial.print(mon); Serial.print("/");
        Serial.print(day); Serial.print(" ");
        Serial.print(hr);  Serial.print(":");
        Serial.print(mn);  Serial.print(":");
        Serial.println(sc);
      } else {
        Serial.println("Invalid SETTIME format. Use: SETTIME YYYY MM DD HH mm ss");
      }
    }
    // если есть другие команды — тоже здесь
  }
}

/************************************************
 *   Сброс суточного пробега в полночь
 ************************************************/
void resetDailyDistanceAtMidnight(int hh, int mm)
{
  if (hh==0 && mm==0) {
    if (!dailyResetDone) {
      dailyDistance = 0.0;
      Serial.println("Daily distance RESET at midnight!");
      // Сразу пишем 0 в EEPROM
      writeFloatEEPROM(ADDR_DAILY_DISTANCE, dailyDistance);
      dailyResetDone = true;
    }
  } else {
    dailyResetDone = false;
  }
}

/************************************************
 *  ОБНОВЛЕНИЕ ДИСПЛЕЯ (обычный режим)
 ************************************************/
void updateDisplay()
{
  // СТАРЫЙ вызов (показывал общий int) - не удаляем
  // showOdometer( (int) totalOdometerKm ); 

  // НОВЫЙ вызов: показываем «суточный» пробег
  showDailyDistance(dailyDistance);

  // Скорость
  showSpeed(currentSpeed);

  // Напряжение
  showVoltage(currentVolt);

  // Температура
  showTemperature(currentTemp);

  // Время
  showTime(hoursRTC, minutesRTC);

  // Стрелки скорости
  updateSpeedArrows((int)currentSpeed);
}


/***********************************************
 *   НОВАЯ ФУНКЦИЯ: showStartupTotalDistance()
 *   Показываем только общий пробег + "km"
 *
 *   Допустим, хотим 6 цифр + точка = 7 символов
 *   "000000.0" => и в конце "km"
 *
 *   Для упрощения делаем "000000.0" (с точкой)
 *   или реальное число totalOdometerKm
***********************************************/
SegmentAddrBit totalDistDigit1[8] = {
  {3,7},{2,2},{2,1},{2,0},{3,4},{3,6},{3,5},{-1,-1}
};
SegmentAddrBit totalDistDigit2[8] = {
  {2,7},{1,2},{1,1},{1,0},{2,4},{2,6},{2,5},{-1,-1}
};
SegmentAddrBit totalDistDigit3[8] = {
   {1,7},{0,2},{0,1},{0,0},{1,4},{1,6},{1,5},{-1,-1}
};
SegmentAddrBit totalDistDigit4[8] = {
  {0,7},{4,2},{4,1},{4,0},{0,4},{0,6},{0,5},{-1,-1}
};
SegmentAddrBit totalDistDigit5[8] = {
  {4,7},{5,6},{5,5},{5,4},{4,4},{4,6},{4,5},{5,7}
};
SegmentAddrBit totalDistDigit6[8] = {
  {5,3},{6,6},{6,5},{6,4},{5,0},{5,2},{5,1},{-1,-1}
};


// Вызываем при старте (10 секунд)
void showStartupTotalDistance(float totalVal)
{
  // Очищаем дисплей перед выводом
  clearAll();

  // Допустим, максимум 999999.9
  if (totalVal < 0) totalVal = 0;
  if (totalVal > 999999.9) totalVal = 999999.9;

  // Представим, что хотим 1 знак после запятой
  unsigned long x = (unsigned long)(totalVal * 10 + 0.5); 
  // x может быть до 9999999

  // Разбираем на 7 цифр
  // d6 = десятые, d5..d0 = целая часть
  int d6 = x % 10;  // "десятые"
  x /= 10;
  int d5 = x % 10;  x /= 10;
  int d4 = x % 10;  x /= 10;
  int d3 = x % 10;  x /= 10;
  int d2 = x % 10;  x /= 10;
  int d1 = x % 10;  x /= 10;
  int d0 = x % 10;  

  // Пример вывода: d0 d1 d2 d3 d4 d5 . d6 (7 знаков)
  // Но вам нужно заполнить totalDistDigitX[] реальными адресами!
  showDigit(d0, totalDistDigit1, false);
  showDigit(d1, totalDistDigit2, false);
  showDigit(d2, totalDistDigit3, false);
  showDigit(d3, totalDistDigit4, false);
  // хотим точку в пятом символе:
  showDigit(d4, totalDistDigit5, true);
  showDigit(d5, totalDistDigit6, false);
  // d6 уже некуда выводить, если у нас только 6 символов.
  // Или если есть 7-й, тогда добавьте totalDistDigit7.

  // Включим сегмент "km":
  setSegment(6, 0, true);

  // (!) Повторюсь, что totalDistDigit1..6 = {-1,-1,...}, 
  //     их надо под реальную схему вашего LCD.
}

/************************************************
 *   SETUP
 ************************************************/
void setup()
{
  Serial.begin(9600);
  Serial.println("=== Code with dailyDistance decimal & 10s total + SETTIME command ===");

  pinMode(PIN_BUZZER,      OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LIGHT,       INPUT_PULLUP); 
  pinMode(PIN_TURN_LEFT,   OUTPUT); digitalWrite(PIN_TURN_LEFT, LOW);
  pinMode(PIN_TURN_RIGHT,  OUTPUT); digitalWrite(PIN_TURN_RIGHT, LOW);

  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallInterrupt, RISING);

  pinMode(PIN_VOLTAGE, INPUT);
  pinMode(PIN_TEMP,    INPUT);

  Wire.begin(); 
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
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

  for (int i=0; i<16; i++){
    addrBuffer[i] = 0;
  }

  // Читаем суточный пробег из EEPROM
  dailyDistance = readFloatEEPROM(ADDR_DAILY_DISTANCE);
  Serial.print("Loaded dailyDistance from EEPROM: ");
  Serial.println(dailyDistance, 1);

  // (Если нужен общий пробег, тоже можем читать)
  // totalOdometerKm = readFloatEEPROM(...); // по желанию

  // Переходим в "стартовый" режим на 10 секунд
  startupTimer = millis();
  showStartupTotal = true;
}

/************************************************
 *   LOOP
 ************************************************/
void loop()
{
  // Проверяем команду "SETTIME ...", чтобы можно было установить RTC
  checkSerialForTimeCommand();

  unsigned long now = millis();

  // Считаем скорость/пробег/время раз в 1с — независимо от режима
  if (now - lastSpeedUpdate >= SPEED_PERIOD) {
    lastSpeedUpdate = now;

    // Импульсы
    unsigned long pulses = hallPulseCount - lastHallCount;
    lastHallCount = hallPulseCount;

    float kmPerSec = (pulses / (float)PULSES_PER_REV) * WHEEL_CIRCUM_km;
    currentSpeed = kmPerSec * 3600.0;

    // Пробеги
    totalOdometerKm += kmPerSec;  
    dailyDistance   += kmPerSec;  

    // RTC
    DateTime t = rtc.now();
    hoursRTC   = t.hour();
    minutesRTC = t.minute();
    secondsRTC = t.second();

    // Сброс суточного в полночь
    resetDailyDistanceAtMidnight(hoursRTC, minutesRTC);

    // Напряжение, температура
    currentVolt = measureVoltage();
    currentTemp = measureTemperatureRaw( analogRead(PIN_TEMP) );

    // Каждые 30 секунд сохраняем
    storeDailyDistanceEvery30Sec(secondsRTC);
  }

  // --- ЛОГИКА СТАРТОВОГО ЭКРАНА (10 СЕК) ---
  if (showStartupTotal) {
    if ((now - startupTimer) < 10000UL) {
      // 10 секунд ещё не прошло:
      // Показываем ТОЛЬКО общий пробег + km
      showStartupTotalDistance(totalOdometerKm);
    } else {
      // 10 секунд истекло, переходим в обычный режим
      showStartupTotal = false;
      clearAll(); 
    }
  } 
  else {
    // ====== Обычный режим: отображаем часы, суточный пробег и т.д. =====
    updateDisplay();
  }

  // Пример: буззер всегда выкл
  digitalWrite(PIN_BUZZER, LOW);

  // Прочие действия (считывать кнопку фары, поворотники, ...)
}
