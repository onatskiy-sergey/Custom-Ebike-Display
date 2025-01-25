#include <HT1621.h>

// Пины подключения — поменяйте под свои
#define CS_PIN   10
#define RW_PIN    9
#define DATA_PIN  8

HT1621 lcd(CS_PIN, RW_PIN, DATA_PIN);

// Для хранения текущих значений всех 16 адресов (по 8 бит)
uint8_t addrData[16];

void setup() {
  Serial.begin(9600);
  Serial.println("=== Segment Mapping + Serial Control ===");

  lcd.begin();
  lcd.sendCommand(HT1621::BIAS_THIRD_4_COM); 
  lcd.sendCommand(HT1621::RC256K); 
  lcd.sendCommand(HT1621::SYS_DIS); 
  lcd.sendCommand(HT1621::WDT_DIS); 
  lcd.sendCommand(HT1621::SYS_EN); 
  lcd.sendCommand(HT1621::LCD_ON);
  lcd.clear(); // Гасим всё

  // Инициализируем локальный массив addrData нулями
  for (int i = 0; i < 16; i++) {
    addrData[i] = 0x00;
  }

  // 1) Тестовый перебор всех адресов/битов
  Serial.println(">>> STARTING TEST of all segments (Addr0..15, Bit0..7) ...");
  for (int addr = 0; addr < 16; addr++) {
    for (int bit = 0; bit < 8; bit++) {
      // Сформировать байт, в котором включён один бит
      uint8_t val = (1 << bit);

      // Погасить всё (обнулить все адреса)
      for (int j = 0; j < 16; j++) {
        lcd.write(j, 0x00);
      }

      // Зажечь только нужный адрес (addr) с нужным битом (val)
      lcd.write(addr, val);

      // Информируем в Serial
      Serial.print("Addr=");
      Serial.print(addr);
      Serial.print("  Bit=");
      Serial.print(bit);
      Serial.println(" (ON)  [wait 1s]");

      delay(1000);
    }
  }
  // После цикла дисплей погашаем
  lcd.clear();
  Serial.println(">>> TEST FINISHED! Enter commands like: `ADDR 5 BIT 3` ...");
}

void loop() {
  // 2) Теперь ждём команды из Serial
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    // Пример формата: "ADDR 5 BIT 3" или "ADDR 5 BIT 3 OFF"
    // Разделим строку на слова (по пробелам).
    // Самый простой способ - find() и substring(), но тут
    // сделаем быстро через split по пробелам.

    // Сформируем массив слов
    // (Простейший парсер, на 4-5 слов максимум)
    String tokens[5]; 
    int tokenCount = 0;

    int startIndex = 0;
    while (true) {
      int spaceIndex = input.indexOf(' ', startIndex);
      if (spaceIndex == -1) {
        // Нет пробела -> последний токен
        tokens[tokenCount] = input.substring(startIndex);
        tokenCount++;
        break;
      } else {
        // Есть пробел -> выделяем слово
        tokens[tokenCount] = input.substring(startIndex, spaceIndex);
        tokenCount++;
        startIndex = spaceIndex + 1;
      }
      if (tokenCount >= 5) break; // не больше 5 слов
    }

    // Дальше смотрим по содержимому tokens.
    // Минимальный формат: [0]="ADDR", [1]=<адрес>, [2]="BIT", [3]=<бит> ...
    
    if (tokenCount >= 4) {
      if (tokens[0] == "ADDR" && tokens[2] == "BIT") {
        int addr = tokens[1].toInt();
        int bit  = tokens[3].toInt();

        // Проверяем корректность
        if (addr < 0 || addr >= 16) {
          Serial.println("ERR: Address must be 0..15");
          return;
        }
        if (bit < 0 || bit > 7) {
          Serial.println("ERR: Bit must be 0..7");
          return;
        }

        // Проверяем, есть ли 5-й токен (например, "OFF" или "ON")
        bool turnOff = false;
        if (tokenCount >= 5) {
          if (tokens[4] == "OFF") {
            turnOff = true;
          }
        }
        
        // Обновляем локальный буфер addrData и отправляем на дисплей
        // Если "OFF", сбрасываем бит. Иначе включаем.
        if (turnOff) {
          addrData[addr] &= ~(1 << bit);
        } else {
          addrData[addr] |= (1 << bit);
        }

        // Запись в HT1621
        lcd.write(addr, addrData[addr]);

        // Сообщим в Serial
        Serial.print("SET: Addr=");
        Serial.print(addr);
        Serial.print(" Bit=");
        Serial.print(bit);
        if (turnOff) {
          Serial.println(" -> OFF");
        } else {
          Serial.println(" -> ON");
        }
      }
      else {
        Serial.println("ERR: Unknown command format. Use: ADDR X BIT Y [OFF]");
      }
    }
    else {
      // Недостаточно токенов
      Serial.println("ERR: Not enough parameters! Use: ADDR <A> BIT <B> [OFF]");
    }
  }
}
