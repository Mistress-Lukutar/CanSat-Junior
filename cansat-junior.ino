/**
 * @file CanSat-junior.ino
 * @brief Основной код для конкурса CanSat, юниорская лига.
 * 
 * Этот код управляет процессом работы мини-спутника, включая обработку
 * данных с датчиков, сохранение на SD-карту, отправку телеметрии по UART,
 * управление звуковым сигналом. Здесь содержится реализация основной миссии.
 * Реализовано с использованием подхода FSM (машина конечных состояний)
 * 
 * @author Nate Hunter
 * @date 2025-03-22
 * @link https://github.com/Nate-Hunter-max GitHub репозиторий проекта
 */

/*___________________________________ЗАГОЛОВОЧНЫЕ ФАЙЛЫ____________________________________*/
#include "BMP280.h"        // Датчик давления и температуры
#include "MPU6050_9250.h"  // Акселерометр и гироскоп
#include <SD.h>            // Работа с microSD картой
#include "POST_Encoder.h"  // Кодирование POST-сообщений для динамика

/*___________________________________НАСТРОЙКИ___________________________________*/
#define SD_FNAME "GG_WP.log"        // Имя файла для логов на SD-карте
#define RS_DRV_PIN 3                // Пин для управления системой спасения
#define RS_EJC_HEIGHT 7000          // Высота срабатывания системы спасения (в см)
#define POST_TONE 1000              // Частота сигнала POST динамика (Гц)
#define TEAM_ID "SHARAGA_FOREVER!"  // Название команды
#define DATA_PERIOD 70              // Период обновления данных (мс)
#define DATA_PERIOD_LND 250         // Период обновления после посадки (мс)

/*_____________________________ОПАСНАЯ ЗОНА НАСТРОЕК____________________________*/
#define BUFFER_LEN 100   // Размер буфера для SD и SV610
#define CLEAR_LOGS 0     // Удалять все логи после перезапуска, если не 0
#define SD_SS_PIN 10     // Пин выбора устройства (Slave Select) для microSD
#define SPEAKER_PIN 6    // Пин динамика (по умолчанию D6)
#define START_TH 100     // Установка флага старта при превышении данной высоты (см)
#define APG_TH 5         // Установка флага апогея при разнице давления > [значения] (Па)
#define LND_ACCEL_EPS 8  // Установка флага посадки при изменении ускорения < [значения] (G*10^-2)
#define RS_TIMEOUT 5000  // Время работы системы спасения после активации (мс)

/*_________________________________СОСТОЯНИЯ АВТОМАТА_________________________________*/
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

char buffer[BUFFER_LEN];  // Буфер для хранения данных

/*_____________________________________ФУНКЦИИ СОСТОЯНИЙ_____________________________________*/

/**
 * @brief Функция инициализации системы
 */
void init_state() {
  static uint8_t errorCode = 0;
  if (currentState != lastState) {
    lastState = currentState;
    if (!BMP280_Init()) errorCode = 1;
    if (!MPU_Init()) errorCode = 2;
    if (!SD.begin(SD_SS_PIN)) errorCode = 3;
    if (!errorCode) {
      delay(2000);        // Задержка для SV610
      Post.send("a", 2);  // Отправка POST_OK
      currentState = MAIN;
#if CLEAR_LOGS != 0
      SD.remove(SD_FNAME);
#endif
      return;
    }
  }
  switch (errorCode) {
    case 1: Post.send("b", 2); break;  // Ошибка BMP280
    case 2: Post.send("c", 2); break;  // Ошибка MPU6050/9250
    case 3: Post.send("d", 2); break;  // Ошибка SD-карты
  }
}

/**
 * @brief Основное состояние полета.
 * 
 * Осуществляет первичную инициализацию данных, измеряет давление, высоту и ускорение,
 * управляет состояниями старта, апогея и приземления.
 */
void main_state() {
  static uint32_t pressMin, vectAbs0;

  if (currentState != lastState) {
    lastState = currentState;

    // Пропускаем 10 измерений для 'прогрева' датчика
    for (uint8_t i = 0; i < 10; i++)
      BMP280_ReadData(&currData.temp, &currData.press0);

    // Устанавливаем начальное давление на старте
    BMP280_ReadData(&currData.temp, &currData.press0);

    // Записываем начальные данные акселерометра
    MPU_ReadData(currData.accelData, currData.gyroData);
    StoreVectAbs(&currData);
    vectAbs0 = currData.vectAbs;
    pressMin = currData.press0;
    currData.time = millis();
  }

  if (millis() - currData.time >= DATA_PERIOD) {
    currData.time = millis();
    BMP280_ReadData(&currData.temp, &currData.press);
    MPU_ReadData(currData.accelData, currData.gyroData);
    StoreVectAbs(&currData);
    currData.altitude = BMP280_GetAltitude(&currData.press, &currData.press0);

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
    if (bitRead(currData.flags, 2) && (abs(currData.vectAbs - vectAbs0) <= LND_ACCEL_EPS)) {
      bitSet(currData.flags, 3);
      currentState = LANDING;
    }

    UART_SendData(&currData);
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
    MPU_ReadData(currData.accelData, currData.gyroData);
    StoreVectAbs(&currData);
    currData.altitude = BMP280_GetAltitude(&currData.press, &currData.press0);
    UART_SendData(&currData);
    SD_SaveData(SD_FNAME, &currData);
  }

  // Писк при приземлении для удобства поиска
  Post.send("e", 2);
}

/*__________________________________Функции Ардуино_________________________________*/

void setup() {
  Serial.begin(115200);         // Инициализация последовательного порта с baud rate 115200
  Post.begin(150);              // Инициализация POST пищалки с длительностью писка 'точки' 150 мс
  pinMode(RS_DRV_PIN, OUTPUT);  // Установка пина управления драйвером СС как выход
}

void loop() {
  // Основной цикл обработки состояний
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

/*_________________________________Дополнительная логика_______________________________*/

/**
 * @brief Отправляет данные по UART в раиомодуль.
 * 
 * @param dat Указатель на структуру с данными сенсоров.
 */
void UART_SendData(SensorData* dat) {
  snprintf(buffer, BUFFER_LEN, "%s;%lu;%ld;%ld;%lu;%d;%d;%d;%d\n",
           TEAM_ID, dat->time, dat->altitude, dat->temp, dat->vectAbs,
           bitRead(dat->flags, 0), bitRead(dat->flags, 1),
           bitRead(dat->flags, 2), bitRead(dat->flags, 3));
  Serial.print(buffer);  // Отправка строки через UART
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
    snprintf(buffer, BUFFER_LEN, "%s;%lu;%lu;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%d;%d;%d;%d\n",
             TEAM_ID, dat->time, dat->press, dat->temp, dat->altitude,
             dat->accelData[0], dat->accelData[1], dat->accelData[2],
             dat->gyroData[0], dat->gyroData[1], dat->gyroData[2],
             bitRead(dat->flags, 0), bitRead(dat->flags, 1),
             bitRead(dat->flags, 2), bitRead(dat->flags, 3));
    logFile.print(buffer);  // Запись данных в файл
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
    rsMillis = millis();                  // Запоминаем время включения
    digitalWrite(RS_DRV_PIN, isEnabled);  // Включаем драйвера
  }

  if (isEnabled && (millis() - rsMillis) >= RS_TIMEOUT) {
    isEnabled = false;
    digitalWrite(RS_DRV_PIN, isEnabled);  // Выключаем драйвера по таймауту
  }
}
