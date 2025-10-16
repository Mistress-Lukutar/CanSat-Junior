/**
 * @file CanSat-junior.ino
 * @brief Основной код для конкурса CanSat, юниорская лига.
 * 
 * Этот код управляет процессом работы мини-спутника на базе FeatherFDR v1.0, включая обработку
 * данных с датчиков, сохранение на SD-карту, отправку телеметрии по UART,
 * управление звуковым сигналом. Здесь содержится реализация основной миссии.
 * Реализовано с использованием подхода FSM (машина конечных состояний)
 * 
 * @author Nate Hunter
 * @date 2025-03-22
 * @link https://github.com/Mistress-Lukutar/CanSat-Junior/tree/features/featherfdr-adapt GitHub репозиторий проекта
 */

/*___________________________________ЗАГОЛОВОЧНЫЕ ФАЙЛЫ____________________________________*/
#include "BMP280.h"    // Датчик давления и температуры
#include "LSM6DS3.h"       // Акселерометр и гироскоп LSM6DS3
#include <SD.h>            // Работа с microSD картой
#include "POST_Encoder.h"  // Кодирование POST-сообщений для динамика
#include "e22_sx1268.h"    // Передача данных по LoRa

/*_______________________________________НАСТРОЙКИ_________________________________________*/
#define SD_FNAME "GG_WP.log"          // Имя файла на microSD (ВЕРХНИЙ_РЕГИСТР.log, до 8 символов)
#define RS_DRV_PIN 9                  // Пин для управления системой спасения
#define RS_EJC_HEIGHT 7000            // Высота срабатывания системы спасения (в см)
#define POST_TONE 2600                // Частота сигнала POST динамика (Гц)
#define TEAM_NAME "MISTRESS_LUKUTAR"  // Название команды
#define DATA_PERIOD 100               // Период обновления данных (мс)
#define DATA_PERIOD_LND 250           // Период обновления после посадки (мс)
#define REINIT_COUNT 2                // Столько раз будет произведена попытка подключить датчики
#define UART_LOG_FULL                 // Выводить в Serial телеметрию

/*__________________________________ОПАСНАЯ ЗОНА НАСТРОЕК__________________________________*/
#define BUFFER_LEN 100      // Размер буфера для SD и SV610
#define ALT_BUFFER_SIZE 20  // Количество измерений высоты для определения приземления
#define ALT_LAND_DELTA 20   // Максимальная разница высот для установки флага 'Land' (см)
#define IS_SD_REQUIRED 0    // Если установлен '0', то запуск можно проводить без microSD
#define CLEAR_LOGS 0        // Удалять все логи после перезапуска, если не 0
#define SD_SS_PIN 10        // Пин выбора устройства (Slave Select) для microSD
#define SPEAKER_PIN 5       // Пин динамика (по умолчанию D5)
#define START_TH 100        // Установка флага старта при превышении данной высоты (см)
#define APG_TH 5            // Установка флага апогея при разнице давления > [значения] (Па)
#define RS_TIMEOUT 5000     // Время работы системы спасения после активации (мс)

/*_____________________________________НАСТРОЙКИ LORA_____________________________________*/

// Пины модуля
#define E22_NSS_PIN 8    // Chip select
#define E22_BUSY_PIN A1  // PC1
#define E22_NRST_PIN 7   // PD7
#define E22_DIO1_PIN 2   // PD2
#define E22_DIO2_PIN 3   // PD3
#define E22_TXEN_PIN 4   // PD4
#define E22_RXEN_PIN A3  // PC3

// Конфигурация модуляции LoRa
#define LORA_FREQUENCY 432800000UL  // 432.8 MHz
#define LORA_SF SX1268_LORA_SF7
#define LORA_BW SX1268_LORA_BW_250
#define LORA_CR SX1268_LORA_CR_4_8
#define LORA_TX_POWER 14  // dBm
#define LORA_PREAMBLE_LEN 8

/*__________________________________КОДЫ ОШИБОК НЕ МЕНЯТЬ__________________________________*/
#define ERR_BMP (1 << 0)   // Ошибка инициализации BMPx80
#define ERR_IMU (1 << 1)   // Ошибка инициализации LSM6DS3
#define ERR_LORA (1 << 2)  // Ошибка инициализации LoRa
#define ERR_SD (1 << 3)    // Ошибка инициализации microSD
/*_________________________________СОСТОЯНИЯ АВТОМАТА_____________________________________*/
enum states : uint8_t { INIT,
                        MAIN,
                        LANDING };
states lastState = LANDING;
states currentState = INIT;

/*_________________________________СТРУКТУРЫ И ПЕРЕМЕННЫЕ_________________________________*/
/**
 * @brief Структура для хранения данных с датчиков
 */
struct SensorData {
  uint32_t time;         ///< Время с начала работы (мс)
  uint32_t press0;       ///< Давление на высоте стартового стола (Па)
  uint32_t press;        ///< Текущее давление (Па)
  int32_t temp;          ///< Температура (в сотых долях градуса Цельсия)
  int32_t accelData[3];  ///< Ускорение по XYZ (G * 10^2)
  int32_t gyroData[3];   ///< Угловая скорость по XYZ (градусы/сек * 10^-2)
  uint32_t vectAbs;      ///< Абсолютное значение вектора ускорения (G * 10^-2)
  int32_t altitude;      ///< Высота относительно старта (см)
  uint8_t flags;         ///< Флаги состояния (0|0|0|0|Land|ResSys|Apg|Start)
} currData;

LSM6DS3_Device imu;  // Структура датчика LSM6DS3
E22_Device lora;     // Структура приемопередатчика LoRa

char strBuf[BUFFER_LEN];          // Буфер для хранения строк для передачи по радио и записи на SD
int32_t altBuf[ALT_BUFFER_SIZE];  // Кольцевой буфер для хранения последних показаний высоты
/*_____________________________________ФУНКЦИИ СОСТОЯНИЙ___________________________________*/

/**
 * @brief Функция инициализации системы
 */
void init_state() {
  static uint8_t errorCode = 0;
  static uint8_t errCount = 0;

  if (currentState != lastState) {

    // Инициализация барометра
    if (!BMP280_Init()) errorCode |= ERR_BMP;

    // Инициализация 6-DOF IMU
    if (!LSM6DS3_Init(&imu, LSM6DS3_ADDR_HIGH)) errorCode |= ERR_IMU;

    // Инициализация LoRa
    if (!E22_Init(&lora)) errorCode |= ERR_LORA;

    // Инициализация SD
    if (!SD.begin(SD_SS_PIN)) errorCode |= ERR_SD;

    if (!errorCode) {
      Post.send("a", 2);  // Отправка POST_OK
      lastState = currentState;
      currentState = MAIN;

#if CLEAR_LOGS != 0
      SD.remove(SD_FNAME);
#endif
      return;
    }
  }
  errCount++;
  Serial.print("Error: ");
  if (errorCode & ERR_BMP) {  // Ошибка BMP280
    Serial.println("BMP");
    Post.send("b", 2);
  } else if (errorCode & ERR_IMU) {  // Ошибка LSM6DS3
    Serial.println("LSM6DS3");
    Post.send("c", 2);
  } else if (errorCode & ERR_LORA) {  // Ошибка LoRa
    Serial.println("LORA");
    Post.send("e", 2);
  } else if (errorCode & ERR_SD) {  // Ошибка SD-карты
    Serial.println("SD");
    Post.send("d", 2);
#if IS_SD_REQUIRED == 0
    if (errorCode ^ ERR_SD == 0) {
      lastState = currentState;
      currentState = MAIN;  // Возможность продолжения без SD
    }
#endif
  }
  delay(100);
  if (errCount == REINIT_COUNT) {
    Serial.print("Failed to start subsystems:");
    if (errorCode & ERR_BMP) Serial.print(" BMP");
    if (errorCode & ERR_IMU) Serial.print(" LSM6DS3");
    if (errorCode & ERR_LORA) Serial.print(" LORA");
    if (errorCode & ERR_SD) Serial.print(" SD");

    Serial.println();
    while (1) {
      Post.update();
      // Управление звуковым сигналом
      if (Post.getSignal()) {
        tone(SPEAKER_PIN, POST_TONE);  // Включение звука при наличии сигнала
      } else {
        noTone(SPEAKER_PIN);  // Выключение звука
      }
    };  // Цикл критической ошибки, сброс только по перезагрузке
  }
}

/**
 * @brief Основное состояние полета.
 * 
 * Осуществляет первичную инициализацию данных, измеряет давление, высоту и ускорение,
 * управляет состояниями старта, апогея и приземления.
 */
void main_state() {
  static uint32_t pressMin;

  if (currentState != lastState) {
    lastState = currentState;

    // Пропускаем 50 измерений для 'прогрева' датчика
    for (uint8_t i = 0; i < 50; i++) BMP280_ReadData(&currData.temp, &currData.press0);

    // Настройка LSM6DS3
    LSM6DS3_ConfigAccel(&imu, LSM6DS3_ODR_52_HZ, LSM6DS3_ACCEL_FS_16G);
    LSM6DS3_ConfigGyro(&imu, LSM6DS3_ODR_52_HZ, LSM6DS3_GYRO_FS_1000DPS);
    LSM6DS3_EnableAccelAxes(&imu, 1, 1, 1);
    LSM6DS3_EnableGyroAxes(&imu, 1, 1, 1);

    // Устанавливаем начальное давление на старте
    BMP280_ReadData(&currData.temp, &currData.press0);
    pressMin = currData.press0;
    currData.time = millis();
  }

  if (millis() - currData.time >= DATA_PERIOD) {
    currData.time = millis();
    BMP280_ReadData(&currData.temp, &currData.press);
    IMU_ReadData(currData.accelData, currData.gyroData);
    StoreVectAbs(&currData);
    currData.altitude = BMP280_GetAltitude(&currData.press, &currData.press0);
    addAltitudeToBuffer(currData.altitude);

    // Фиксируем старт
    if (currData.altitude > START_TH)
      bitSet(currData.flags, 0);

    // Обновляем минимальное давление
    if (currData.press < pressMin)
      pressMin = currData.press;

    // Фиксируем апогей
    if (bitRead(currData.flags, 0) && (currData.press - pressMin > APG_TH))
      bitSet(currData.flags, 1);

    // Фиксируем развертывание системы спасения
    if (bitRead(currData.flags, 1) && !bitRead(currData.flags, 2) && (currData.altitude <= RS_EJC_HEIGHT)) {
      bitSet(currData.flags, 2);
      RS_Tick(1);
    }

    // Фиксируем приземление
    if (bitRead(currData.flags, 2) && checkLandingCondition()) {
      bitSet(currData.flags, 3);
      currentState = LANDING;
    }

    LORA_SendData(&currData);
    SD_SaveData(SD_FNAME, &currData);
  }
}

/**
 * @brief Состояние приземления.
 * 
 * В этом режиме продолжается запись данных, но с увеличенным интервалом.
 */
void landing_state() {
  if (currentState != lastState) {
    lastState = currentState;
    currData.time = millis();
  }

  // Запись данных в стуктуру по таймеру
  if (millis() - currData.time >= DATA_PERIOD_LND) {
    currData.time = millis();
    BMP280_ReadData(&currData.temp, &currData.press);
    IMU_ReadData(currData.accelData, currData.gyroData);
    StoreVectAbs(&currData);
    currData.altitude = BMP280_GetAltitude(&currData.press, &currData.press0);
    LORA_SendData(&currData);
    SD_SaveData(SD_FNAME, &currData);
  }

  // Писк при приземлении для удобства поиска
  Post.send("e", 2);
}

/*__________________________________Точка входа_________________________________*/
int main() {
  init();                       // Инициализация ядра
  Serial.begin(115200);         // Инициализация последовательного порта с baud rate 115200
  Post.begin(150);              // Инициализация POST пищалки с длительностью писка 'точки' 150 мс
  pinMode(RS_DRV_PIN, OUTPUT);  // Установка пина управления драйвером СС как выход
  digitalWrite(RS_DRV_PIN, 1);  // Выключение CC, из-за P-Mosfet инвертировано

  // Установка параметров E22 модуля
  lora.spi = &SPI;
  lora.nss_pin = E22_NSS_PIN;
  lora.busy_pin = E22_BUSY_PIN;
  lora.nrst_pin = E22_NRST_PIN;
  lora.dio1_pin = E22_DIO1_PIN;
  lora.dio2_pin = E22_DIO2_PIN;
  lora.txen_pin = E22_TXEN_PIN;
  lora.rxen_pin = E22_RXEN_PIN;

  // Установка параметров модуляции LoRa
  lora.frequency = LORA_FREQUENCY;
  lora.spreading_factor = LORA_SF;
  lora.bandwidth = LORA_BW;
  lora.coding_rate = LORA_CR;
  lora.tx_power = LORA_TX_POWER;
  lora.preamble_length = LORA_PREAMBLE_LEN;

  while (1) {

    // Обработка переключений FSM
    switch (currentState) {
      case INIT:
        init_state();
        break;
      case MAIN:
        main_state();
        break;
      case LANDING:
        landing_state();
        break;
      default:
        break;
    }

    // Обновление состояния POST пищалки
    Post.update();
    RS_Tick(0);

    // Управление звуковым сигналом
    if (Post.getSignal()) {
      tone(SPEAKER_PIN, POST_TONE);  // Включение звука при наличии сигнала
    } else {
      noTone(SPEAKER_PIN);  // Выключение звука
    }
  }
}

/*_________________________________Дополнительная логика_______________________________*/

/**
 * @brief Читает данные с датчика LSM6DS3 и преобразует их в стандартный формат
 * 
 * @param accelData Массив для хранения данных акселерометра (G * 10^2)
 * @param gyroData Массив для хранения данных гироскопа (градусы/сек * 10^-2)
 */
void IMU_ReadData(int32_t accelData[3], int32_t gyroData[3]) {
  static LSM6DS3_Data accel, gyro;

  // Читаем данные с датчика
  if (LSM6DS3_IsAccelDataReady(&imu)) {
    LSM6DS3_ReadAccel(&imu, &accel);

    // Преобразуем данные акселерометра из g в формат G * 10^2
    accelData[0] = (int32_t)(accel.x * 100.0f);
    accelData[1] = (int32_t)(accel.y * 100.0f);
    accelData[2] = (int32_t)(accel.z * 100.0f);
  }

  if (LSM6DS3_IsGyroDataReady(&imu)) {
    LSM6DS3_ReadGyro(&imu, &gyro);

    // Преобразуем данные гироскопа из dps в формат градусы/сек * 10^-2
    gyroData[0] = (int32_t)(gyro.x * 100.0f);
    gyroData[1] = (int32_t)(gyro.y * 100.0f);
    gyroData[2] = (int32_t)(gyro.z * 100.0f);
  }
}

/**
 * @brief Отправляет данные по SPI в раиомодуль.
 * 
 * @note Отправляет по UART если включен UART_LOG_FULL.
 * @param dat Указатель на структуру с данными сенсоров.
 */
void LORA_SendData(SensorData* dat) {
#ifdef UART_LOG_FULL
  snprintf(strBuf, BUFFER_LEN, "%s;%lu;%ld;%ld;%lu;%d;%d;%d;%d\n",
           TEAM_NAME, dat->time, dat->altitude, dat->temp, dat->vectAbs,
           bitRead(dat->flags, 0), bitRead(dat->flags, 1),
           bitRead(dat->flags, 2), bitRead(dat->flags, 3));
  Serial.print(strBuf);  // Отправка строки через UART
  E22_SendData(&lora, strBuf, strlen(strBuf) + 1);
#endif
}

/**
 * @brief Сохраняет данные на SD-карту.
 * 
 * @param fileName Имя файла для сохранения.
 * @param dat Указатель на структуру с данными сенсоров.
 * @return true, если запись успешна, иначе false.
 */
bool SD_SaveData(const char* fileName, SensorData* dat) {
  static File logFile;
  logFile = SD.open(fileName, FILE_WRITE);
  if (logFile) {
    snprintf(strBuf, BUFFER_LEN, "%s;%lu;%lu;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%d;%d;%d;%d\n",
             TEAM_NAME, dat->time, dat->press, dat->temp, dat->altitude,
             dat->accelData[0], dat->accelData[1], dat->accelData[2],
             dat->gyroData[0], dat->gyroData[1], dat->gyroData[2],
             bitRead(dat->flags, 0), bitRead(dat->flags, 1),
             bitRead(dat->flags, 2), bitRead(dat->flags, 3));
    logFile.print(strBuf);  // Запись данных в файл
    logFile.close();        // Закрытие файла
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Рассчитывает абсолютное значение вектора ускорения.
 * 
 * @param dat Указатель на структуру с данными сенсоров.
 */
inline void StoreVectAbs(SensorData* dat) {
  dat->vectAbs = 0;
  for (uint8_t i = 0; i < 3; i++) {
    dat->vectAbs += dat->accelData[i] * dat->accelData[i];
  }
  dat->vectAbs = sqrt(dat->vectAbs);  // Вычисление квадратного корня
}

/**
 * @brief Управление системой спасения с таймаутом.
 * 
 * @param isActivate Флаг активации драйвера.
 */
void RS_Tick(bool isActivate) {
  static uint32_t rsMillis;
  static bool isEnabled;

  if (!isEnabled && isActivate) {
    isEnabled = true;
    rsMillis = millis();                   // Запоминаем время включения
    digitalWrite(RS_DRV_PIN, !isEnabled);  // Включаем драйвера (0 - вкл, 1 - выкл)
  }

  if (isEnabled && (millis() - rsMillis) >= RS_TIMEOUT) {
    isEnabled = false;
    digitalWrite(RS_DRV_PIN, !isEnabled);  // Выключаем драйвера по таймауту
  }
}

/**
 * @brief Добавляет новое значение высоты в кольцевой буфер.
 * 
 * @param altitude Новое значение высоты для добавления в буфер.
 */
void addAltitudeToBuffer(int32_t altitude) {
  static uint8_t altBuffIndex = 0;  // Текущий индекс в кольцевом буфере
  altBuf[altBuffIndex] = altitude;
  altBuffIndex = (altBuffIndex + 1) % ALT_BUFFER_SIZE;
}

/**
 * @brief Проверяет условие приземления по данным в буфере.
 * 
 * Анализирует разницу между минимальным и максимальным значениями в буфере.
 * 
 * @return true - если разница меньше ALT_LAND_DELTA (условие приземления)
 * @return false - если разница больше или равна ALT_LAND_DELTA
 */
bool checkLandingCondition(void) {
  int32_t minAlt = altBuf[0];
  int32_t maxAlt = altBuf[0];

  // Поиск минимального и максимального значений в буфере
  for (uint8_t i = 1; i < ALT_BUFFER_SIZE; i++) {
    if (altBuf[i] < minAlt) {
      minAlt = altBuf[i];
    }
    if (altBuf[i] > maxAlt) {
      maxAlt = altBuf[i];
    }
  }

  return (maxAlt - minAlt) < ALT_LAND_DELTA;
}