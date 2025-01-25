#include <Wire.h>
#include <RTClib.h>      // библиотека для DS3231
#include <HT1621.h>

// ------------------------ НАСТРОЙКИ ПОДКЛЮЧЕНИЙ --------------------------
#define PIN_CS    10   // чип-селект HT1621B
#define PIN_WR    9    // WR
#define PIN_DATA  8    // DATA
HT1621 lcd(PIN_CS, PIN_WR, PIN_DATA);

#define PIN_HALL        7  // вход от датчика Холла (прерывание)
#define PIN_BUZZER      3  // бузер (можно пока не использовать)
#define PIN_LIGHT       4  // выход на фару
#define PIN_TURN_LEFT   6  // выход на реле/лампу левого поворотника
#define PIN_TURN_RIGHT  5  // выход на реле/лампу правого поворотника

// Аналоговые входы:
#define PIN_VOLTAGE  A1   // измерение напряжения
#define PIN_TEMP     A2   // терморезистор 10 кОм

// ------------------------ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ --------------------------

// Пробег:
volatile unsigned long hallPulseCount = 0; // счётчик импульсов от Холла
unsigned long lastHallCount    = 0;        
float totalOdometerKm          = 0.0;      // общий пробег

// Параметры колеса:
const float WHEEL_CIRCUM_km = 0.00193; // 193 см = 1.93 м = 0.00193 км
const int   PULSES_PER_REV  = 23;      // 23 импульса на 1 оборот

// Таймер для расчёта скорости:
unsigned long lastSpeedUpdate    = 0;
const unsigned long SPEED_PERIOD = 1000; // раз в 1 секунду считаем скорость

// Текущее состояние (обновляется раз в секунду):
float currentSpeed = 0.0; // км/ч
int   currentTemp  = 0;   // °C
float currentVolt  = 0.0; // В
int   hoursRTC     = 0;
int   minutesRTC   = 0;

// RTC
RTC_DS3231 rtc;

// Локальный буфер для 16 адресов HT1621 (Addr=0..15)
uint8_t addrBuffer[16];

// Делитель напряжения (пример! калибруйте под свои R1,R2)
const float VOLTAGE_DIVIDER_K = 11.0; 

// ------------------- ОБРАБОТКА ПРЕРЫВАНИЯ ОТ ХОЛЛА -----------------------
void hallInterrupt()
{
  hallPulseCount++;
}

// ------------------- ФУНКЦИИ ИЗМЕРЕНИЯ ----------------------------

// «Линейный» пример расчёта температуры (заглушка)
int measureTemperatureRaw(int analogValue)
{
  float voltage = (5.0 * analogValue) / 1023.0; 
  float temp = 25.0 + (2.5 - voltage)*(-20.0); 
  return (int)temp;
}

// Измеряем напряжение батареи (учитывая делитель)
float measureVoltage()
{
  int raw = analogRead(PIN_VOLTAGE);
  float voltageAtPin = (5.0 * raw) / 1023.0;
  float battery = voltageAtPin * VOLTAGE_DIVIDER_K;
  return battery;
}

// ------------------ ФУНКЦИИ УПРАВЛЕНИЯ ДИСПЛЕЕМ ------------------

void clearAll()
{
  for (int i=0; i<16; i++){
    addrBuffer[i] = 0x00;
    lcd.write(i, 0x00);
  }
}

// Установить/снять бит в локальном буфере и отправить в HT1621
void setSegment(int addr, int bitNum, bool on)
{
  if (addr<0 || addr>15 || bitNum<0 || bitNum>7) return;
  if (on) addrBuffer[addr] |=  (1 << bitNum);
  else    addrBuffer[addr] &= ~(1 << bitNum);
  lcd.write(addr, addrBuffer[addr]);
}

// ----- Массивы сегментов (для 7сег цифр) -----

// 7-сегментные цифры (A=bit0 .. G=bit6)
const uint8_t digitToSegments[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

// Структура для хранения (addr, bit)
struct SegmentAddrBit {
  int addr;
  int bit;
};

// Вывести 1 цифру (digit=0..9) на заданную «карту сегментов».  
// dpOn=true, если хотим зажечь «точку» (если она есть в mapping[7]).
void showDigit(uint8_t digit, SegmentAddrBit mapping[8], bool dpOn = false)
{
  if (digit>9) return;
  uint8_t mask = digitToSegments[digit]; 
  // Семь сегментов A..G
  for (int i=0; i<7; i++){
    int a = mapping[i].addr;
    int b = mapping[i].bit;
    if (a<0 || b<0) continue;
    bool on = (mask & (1<<i)) != 0;
    setSegment(a, b, on);
  }
  // Точка (если есть)
  int da = mapping[7].addr;
  int db = mapping[7].bit;
  if (da>=0 && db>=0) {
    setSegment(da, db, dpOn);
  }
}

// ============ ОДОМЕТР (3 цифры + "km") =============================

SegmentAddrBit odometerDigit1[8] = {
  {0,7}, // A
  {4,2}, // B
  {4,1}, // C
  {4,0}, // D
  {0,4}, // E
  {0,6}, // F
  {0,5}, // G
  {-1,-1} // DP
};

SegmentAddrBit odometerDigit2[8] = {
  {4,7}, // A
  {5,6}, // B
  {5,5}, // C
  {5,4}, // D
  {4,4}, // E
  {4,6}, // F
  {4,5}, // G
  {5,7}  // DP
};

SegmentAddrBit odometerDigit3[8] = {
  {5,3}, // A
  {6,6}, // B
  {6,5}, // C
  {6,4}, // D
  {5,0}, // E
  {5,2}, // F
  {5,1}, // G
  {-1,-1} // DP
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

  // Включить "km" (Addr=6 / Bit=0)
  setSegment(6, 0, true);
}

// ============ СКОРОСТЬ (3 цифры + "km/h") =========================

// 1-я «цифра» – только сегменты B,C => Addr=7/Bit=7 (показывать «1»)
#define ADDR_SPEED1_BC  7
#define BIT_SPEED1_BC   7

SegmentAddrBit speedDigit2[8] = {
  {7,3}, // A
  {7,2}, // B
  {7,1}, // C
  {7,0}, // D
  {7,4}, // E
  {7,6}, // F
  {7,5}, // G
  {-1,-1}
};

SegmentAddrBit speedDigit3[8] = {
  {8,3}, // A
  {8,2}, // B
  {8,1}, // C
  {8,0}, // D
  {8,4}, // E
  {8,6}, // F
  {8,5}, // G
  {-1,-1}
};

void showSpeed(float spd)
{
  if (spd<0) spd=0;
  bool over99 = (spd>99);

  // Первая «1»
  setSegment(ADDR_SPEED1_BC, BIT_SPEED1_BC, over99);

  int speedInt = (int)spd;
  if (speedInt>99) speedInt=99; 
  int d2 = speedInt/10;
  int d3 = speedInt%10;
  showDigit(d2, speedDigit2);
  showDigit(d3, speedDigit3);

  // "km/h" => Addr=12, Bit=6
  setSegment(12, 6, true);
}

// ============ НАПРЯЖЕНИЕ (4 цифры + «V») ===========================
#define ADDR_VOLT_DIGIT1_BC 12
#define BIT_VOLT_DIGIT1_BC  4

SegmentAddrBit voltDigit2[8] = {
  {12,3}, // A
  {13,6}, // B
  {13,5}, // C
  {13,4}, // D
  {12,0}, // E
  {12,2}, // F
  {12,1}, // G
  {-1,-1}
};
SegmentAddrBit voltDigit3[8] = {
  {13,3}, // A
  {14,6}, // B
  {14,5}, // C
  {14,4}, // D
  {13,0}, // E
  {13,2}, // F
  {13,1}, // G
  {-1,-1}
};
SegmentAddrBit voltDigit4[8] = {
  {14,3}, // A
  {15,6}, // B
  {15,5}, // C
  {15,4}, // D
  {14,0}, // E
  {14,2}, // F
  {14,1}, // G
  {-1,-1}
};

#define ADDR_VOLT_POINT 15
#define BIT_VOLT_POINT  7

void showVoltage(float volts)
{
  bool over99 = (volts > 99.9);
  setSegment(ADDR_VOLT_DIGIT1_BC, BIT_VOLT_DIGIT1_BC, over99);

  // Ограничим сверху
  if (volts>150.0) volts=150.0;

  // Например, показываем с точностью 0.1В => int(волты*10)
  int iv = (int)(volts*10); // например, 54.3В => 543
  int d2 = (iv/100)%10;
  int d3 = (iv/10)%10;
  int d4 = iv%10;

  // DP (точку) ставим в третьей цифре
  showDigit(d2, voltDigit2, false);
  showDigit(d3, voltDigit3, true); 
  showDigit(d4, voltDigit4, false);

  // «V» (Addr=15 / Bit=7) – тут используем тот же бит, что и для точки?
  // В исходном ТЗ писалось, что Bit=7 – "эмблема V". 
  // Так что оставим включённым
  setSegment(ADDR_VOLT_POINT, BIT_VOLT_POINT, true);
}

// ============ ТЕМПЕРАТУРА (3 символа + «°C») ======================
#define ADDR_TEMP1_BC   9
#define BIT_TEMP1_BC    3
#define ADDR_TEMP_MINUS 10
#define BIT_TEMP_MINUS  3

SegmentAddrBit tempDigit2[8] = {
  {10,4}, // A
  {10,5}, // B
  {10,6}, // C
  {10,7}, // D
  {9,2},  // E
  {9,0},  // F
  {9,1},  // G
  {-1,-1}
};
SegmentAddrBit tempDigit3[8] = {
  {11,4}, // A
  {11,5}, // B
  {11,6}, // C
  {11,7}, // D
  {10,2}, // E
  {10,0}, // F
  {10,1}, // G
  {-1,-1}
};

void showTemperature(int tempC)
{
  bool negative = (tempC<0);
  if (tempC < -9)  tempC = -9; 
  if (tempC > 99)  tempC = 99;

  // 1 символ => «1» если |tempC|>=10
  bool showOne = (tempC>=10 || tempC<=-10);
  setSegment(ADDR_TEMP1_BC, BIT_TEMP1_BC, showOne);

  int absT = negative ? -tempC : tempC;
  int d2 = (absT/10)%10;
  int d3 = absT%10;
  showDigit(d2, tempDigit2);
  showDigit(d3, tempDigit3);

  // Минус
  setSegment(ADDR_TEMP_MINUS, BIT_TEMP_MINUS, negative);

  // "°C" => Addr=11 / Bit=0
  setSegment(11, 0, true);
}

// ============ ЧАСЫ (4 цифры, HH:MM + двоеточие) ===================

// 1-я цифра (старшие часы): ПРИМЕРНЫЙ маппинг! Проверьте сами!
SegmentAddrBit timeDigit1[8] = {
  {3,2}, // A
  {2,3}, // B
  {3,1}, // C
// {3,2}, // D
// {3,2}, // E
// {3,2}, // F
// {3,2}, // G
  {-1,-1}
};


// 2-я цифра (младшие часы)
SegmentAddrBit timeDigit2[8] = {
  {3,7}, // A
  {2,2}, // B
  {2,1}, // C
  {2,0}, // D
  {3,4}, // E
  {3,6}, // F
  {3,5}, // G
  {-1,-1}
};

// 3-я и 4-я цифра (минуты)
SegmentAddrBit timeDigit3[8] = {
  {2,7}, // A
  {1,2}, // B
  {1,1}, // C
  {1,0}, // D
  {2,4}, // E
  {2,6}, // F
  {2,5}, // G
  {-1,-1}
};

SegmentAddrBit timeDigit4[8] = {
  {1,7}, // A
  {0,2}, // B
  {0,1}, // C
  {0,0}, // D
  {1,4}, // E
  {1,6}, // F
  {1,5}, // G
  {-1,-1}
};

// Адреса для двоеточия:
#define ADDR_TIME_COLON  3
#define BIT_TIME_COLON   3

void showTime(int hh, int mm)
{
  // Безопасные границы
  if (hh<0)   hh=0; 
  if (hh>23)  hh=23;
  if (mm<0)   mm=0;
  if (mm>59)  mm=59;

  int h1 = hh / 10;     // десятки часа
  int h2 = hh % 10;     // единицы часа
  int m1 = mm / 10;     // десятки минут
  int m2 = mm % 10;     // единицы минут

  // Показать 4 цифры
  showDigit(h1, timeDigit1);
  showDigit(h2, timeDigit2);
  showDigit(m1, timeDigit3);
  showDigit(m2, timeDigit4);

  // Двоеточие
  setSegment(ADDR_TIME_COLON, BIT_TIME_COLON, true);
}

// ============ ИКОНКИ (поворотники, фара и т.п.) ==================
#define ADDR_LTURN  9
#define BIT_LTURN   4
#define ADDR_RTURN  11
#define BIT_RTURN   3
#define ADDR_LIGHT  11
#define BIT_LIGHT   2
#define ADDR_ECU    15
#define BIT_ECU     3   // в резерве

void setLeftTurn(bool on)
{
  digitalWrite(PIN_TURN_LEFT,  on?HIGH:LOW);
  setSegment(ADDR_LTURN, BIT_LTURN, on);
}
void setRightTurn(bool on)
{
  digitalWrite(PIN_TURN_RIGHT, on?HIGH:LOW);
  setSegment(ADDR_RTURN, BIT_RTURN, on);
}
void setHeadLight(bool on)
{
  digitalWrite(PIN_LIGHT, on?HIGH:LOW);
  setSegment(ADDR_LIGHT, BIT_LIGHT, on);
}

// ============ ИНИЦИАЛИЗАЦИЯ =======================================

void setup()
{
  Serial.begin(9600);
  Serial.println("=== Real code (no flicker) ===");

  pinMode(PIN_BUZZER,      OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LIGHT,       INPUT_PULLUP);
  pinMode(PIN_TURN_LEFT,   OUTPUT); digitalWrite(PIN_TURN_LEFT, LOW);
  pinMode(PIN_TURN_RIGHT,  OUTPUT); digitalWrite(PIN_TURN_RIGHT, LOW);

  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallInterrupt, RISING);

  pinMode(PIN_VOLTAGE, INPUT);
  pinMode(PIN_TEMP,    INPUT);

  // Инициализация I2C + RTC
  Wire.begin(); 
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else {
    if (rtc.lostPower()) {
      // Установим время по умолчанию (при сбое)
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("RTC lost power, set default time!");
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
  for (int i=0;i<16;i++){
    addrBuffer[i]=0;
  }

  // При желании сразу включить постоянные надписи:
  // - "km/h", "km", "°C" и т.д.
  // Однако у нас они включаются в соответствующих функциях (showXXX).
}

// ============ ОСНОВНОЙ ЦИКЛ =======================================
void loop()
{
  unsigned long now = millis();

  // Раз в 1 секунду считаем скорость, обновляем показания
  if (now - lastSpeedUpdate >= SPEED_PERIOD) {
    lastSpeedUpdate = now;

    // Сколько импульсов от Холла за последнюю секунду
    unsigned long pulses = hallPulseCount - lastHallCount;
    lastHallCount = hallPulseCount;

    // км за 1 сек
    float kmPerSec = (pulses / (float)PULSES_PER_REV) * WHEEL_CIRCUM_km;
    // => км/ч
    currentSpeed = kmPerSec * 3600.0;

    // Одометр
    totalOdometerKm += kmPerSec;
    if (totalOdometerKm > 999.0) totalOdometerKm = 999.0;

    // Напряжение
    currentVolt = measureVoltage();
    // Температура
    currentTemp = measureTemperatureRaw( analogRead(PIN_TEMP) );

    // Часы
    DateTime t = rtc.now();
    hoursRTC   = t.hour();
    minutesRTC = t.minute();

    // Обновляем на дисплее
    updateDisplay();
  }

  // Здесь можете читать реальные входы для поворотников, фары и т.д. 
  // Например (если у вас есть кнопки/входы):
  /*
    bool leftBtn = digitalRead(PIN_X) == HIGH; 
    setLeftTurn(leftBtn);

    bool rightBtn = digitalRead(PIN_Y) == HIGH;
    setRightTurn(rightBtn);

    bool lightBtn = digitalRead(PIN_Z) == HIGH;
    setHeadLight(lightBtn);
  */

  // Или любая ваша логика — главное, что теперь нет демо-мигания.

  // Пример: буззер пока всегда выкл.
  digitalWrite(PIN_BUZZER, LOW);

  // ... остальной функционал по необходимости
}

// ============ ОБНОВЛЕНИЕ ВСЕХ ГРУПП ДИСПЛЕЯ ======================
void updateDisplay()
{
  // 1) Пробег
  showOdometer( (int)totalOdometerKm );

  // 2) Скорость
  showSpeed(currentSpeed);

  // 3) Напряжение
  showVoltage(currentVolt);

  // 4) Температура
  showTemperature(currentTemp);

  // 5) Время
  showTime(hoursRTC, minutesRTC);

  // 6) Указатели скорости (если нужно)
  updateSpeedArrows((int)currentSpeed);
}

// Пример «стрелок» на шкале (по диапазонам скорости)
void updateSpeedArrows(int spd)
{
  // Сброс
  setSegment(1,3,false);
  setSegment(0,3,false);
  setSegment(8,7,false);
  setSegment(12,7,false);
  setSegment(14,7,false);

  // Включаем нужные
  if (spd>=10 && spd<=20)   setSegment(1,3,true);
  if (spd>=30 && spd<=40)   setSegment(0,3,true);
  if (spd>=50 && spd<=60)   setSegment(8,7,true);
  if (spd>=70 && spd<=90)   setSegment(12,7,true);
  if (spd>=110 && spd<=130) setSegment(14,7,true);
}
