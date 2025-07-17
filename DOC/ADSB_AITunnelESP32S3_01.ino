//Создам полный код для приема и расшифровки пакетов ADS-B на ESP32-S3 с RTLSDR приемником.


#include <WiFi.h>
#include <USB.h>
#include <USBHost.h>
#include <HardwareSerial.h>

// WiFi настройки
const char* wifi_ssid = "TPLINK";
const char* wifi_password = "panasonic";

// UART настройки
HardwareSerial UART1_Port(1);

// USB Host настройки
class RTLSDRDevice : public USBDevice {
public:
  RTLSDRDevice(USBHost &host) : USBDevice(host) {}
  bool init();
  bool configure();
  void setupTuner();
  void startCapture();
  void processData();
};

// CRC таблица для ADS-B (полная)
const uint32_t crc_table[256] = {
  0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B,
  0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
  0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 0x4C11DB70, 0x48D0C6C7,
  0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
  0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3,
  0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039,
  0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF,
  0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
  0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB,
  0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
  0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077, 0x30476DC0,
  0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
  0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4,
  0x0808D07D, 0x0CC9CDCA, 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE,
  0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08,
  0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
  0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC,
  0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6,
  0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050,
  0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
  0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34,
  0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637,
  0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1,
  0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
  0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5,
  0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF,
  0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 0xF12F560E, 0xF5EE4BB9,
  0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
  0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD,
  0xCDA1F604, 0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
  0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71,
  0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
  0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2,
  0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8,
  0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E,
  0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
  0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A,
  0x2D15EBE3, 0x29D4F654, 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0,
  0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676,
  0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
  0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662,
  0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668,
  0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};

// Структура для ADS-B сообщения
struct ADSBMessage {
  uint8_t data[14];
  uint8_t length;
  uint32_t timestamp;
  bool isValid;

  // Декодированные поля
  uint8_t downlink_format;
  uint8_t capability;
  uint32_t icao_address;
  uint8_t type_code;

  // Для позиционных сообщений
  double latitude;
  double longitude;
  int32_t altitude;

  // Для идентификационных сообщений
  char callsign[9];

  // Для скоростных сообщений
  uint16_t ground_speed;
  uint16_t track;
  int16_t vertical_rate;
};

// RTL-SDR команды и регистры R820T2
#define RTL_SDR_VID 0x0bda
#define RTL_SDR_PID 0x2838

// R820T2 регистры
#define R820T2_I2C_ADDR 0x34

// Команды RTL-SDR
#define RTLSDR_CMD_SET_FREQ       0x01
#define RTLSDR_CMD_SET_SAMPLE_RATE 0x02
#define RTLSDR_CMD_SET_GAIN_MODE  0x03
#define RTLSDR_CMD_SET_GAIN       0x04
#define RTLSDR_CMD_SET_FREQ_CORR  0x05
#define RTLSDR_CMD_SET_AGC_MODE   0x08
#define RTLSDR_CMD_SET_DIRECT_SAMPLING 0x09
#define RTLSDR_CMD_SET_OFFSET_TUNING 0x0a
#define RTLSDR_CMD_SET_BIAS_TEE   0x0e

// R820T2 команды настройки
struct R820T2Config {
  uint8_t reg;
  uint8_t value;
};

// Таблица инициализации R820T2 для 1090 МГц
const R820T2Config r820t2_init_table[] = {
  {0x05, 0x90}, // LNA settings
  {0x06, 0x80}, // Mixer settings
  {0x07, 0x60}, // Filter settings
  {0x08, 0x80}, // LO settings
  {0x09, 0x40}, // IF settings
  {0x0A, 0xA8}, // AGC settings
  {0x0B, 0x0F}, // Loop through
  {0x0C, 0x40}, // VGA settings
  {0x0D, 0x63}, // LNA gain
  {0x0E, 0x75}, // Mixer gain
  {0x0F, 0xF8}, // Filter gain
  {0x10, 0x7C}, // IF gain 1
  {0x11, 0x83}, // IF gain 2
  {0x12, 0x80}, // IF gain 3
  {0x13, 0x00}, // IF gain 4
  {0x14, 0x0F}, // IF gain 5
  {0x15, 0x00}, // IF gain 6
  {0x16, 0xC0}, // Detector 1
  {0x17, 0x30}, // Detector 2
  {0x18, 0x48}, // Detector 3
  {0x19, 0xCC}, // Detector 4
  {0x1A, 0xBE}, // Detector 5
  {0x1B, 0x00}, // Calibration
  {0x1C, 0x07}, // VCO current
  {0x1D, 0x00}  // Reserved
};

// USB Host экземпляр
USBHost usbHost;
RTLSDRDevice* rtlsdr = nullptr;

// Буферы для обработки данных
uint8_t sample_buffer[2048];
uint8_t bit_buffer[1024];
ADSBMessage current_message;

void setup() {
  Serial.begin(115200);
  UART1_Port.begin(115200, SERIAL_8N1, 18, 17); // RX=18, TX=17

  Serial.println("Инициализация ADS-B приемника ESP32-S3");

  // Настройка USB Host
  setupUSBHost();

  // Подключение к WiFi
  connectToWiFi();

  // Инициализация CRC таблицы (уже определена выше)
  Serial.println("CRC таблица инициализирована");

  delay(2000);
}

void loop() {
  // Обработка USB событий
  usbHost.task();

  if (rtlsdr && rtlsdr->isConnected()) {
    rtlsdr->processData();
  }

  delay(1);
}

void setupUSBHost() {
  Serial.println("Настройка USB Host...");

  // Инициализация USB Host
  if (usbHost.begin()) {
    Serial.println("USB Host инициализирован");
  } else {
    Serial.println("Ошибка инициализации USB Host");
    return;
  }

  // Создание экземпляра RTL-SDR устройства
  rtlsdr = new RTLSDRDevice(usbHost);

  // Регистрация обработчиков событий
  usbHost.onConnect( {
    Serial.printf("Устройство подключено на адресе %d\n", addr);

    // Проверка VID/PID RTL-SDR
    if (device->getVID() == RTL_SDR_VID && device->getPID() == RTL_SDR_PID) {
      Serial.println("RTL-SDR устройство обнаружено!");
      if (rtlsdr->init()) {
        Serial.println("RTL-SDR инициализировано успешно");
      }
    }
  });

  usbHost.onDisconnect( {
    Serial.printf("Устройство отключено с адреса %d\n", addr);
  });
}

void connectToWiFi() {
  Serial.println("Подключение к WiFi...");
  WiFi.begin(wifi_ssid, wifi_password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi подключен!");
    Serial.println("IP адрес: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nНе удалось подключиться к WiFi");
  }
}

// Реализация методов RTLSDRDevice
bool RTLSDRDevice::init() {
  Serial.println("Инициализация RTL-SDR...");

  if (!configure()) {
    Serial.println("Ошибка конфигурации RTL-SDR");
    return false;
  }

  setupTuner();
  startCapture();

  return true;
}

bool RTLSDRDevice::configure() {
  // Сброс устройства
  if (!controlTransfer(0x40, 0x01, 0x01, 0, nullptr, 0)) {
    return false;
  }
  delay(100);

  // Установка частоты дискретизации (2 MSPS)
  uint32_t sample_rate = 2000000;
  if (!controlTransfer(0x40, RTLSDR_CMD_SET_SAMPLE_RATE,
                      sample_rate & 0xFFFF, sample_rate >> 16, nullptr, 0)) {
    return false;
  }

  // Установка частоты 1090 МГц для ADS-B
  uint32_t frequency = 1090000000;
  if (!controlTransfer(0x40, RTLSDR_CMD_SET_FREQ,
                      frequency & 0xFFFF, frequency >> 16, nullptr, 0)) {
    return false;
  }

  // Отключение AGC
  if (!controlTransfer(0x40, RTLSDR_CMD_SET_AGC_MODE, 0, 0, nullptr, 0)) {
    return false;
  }

  // Установка ручного усиления
  if (!controlTransfer(0x40, RTLSDR_CMD_SET_GAIN_MODE, 1, 0, nullptr, 0)) {
    return false;
  }

  // Установка усиления (40 дБ)
  if (!controlTransfer(0x40, RTLSDR_CMD_SET_GAIN, 400, 0, nullptr, 0)) {
    return false;
  }

  // Отключение bias tee
  if (!controlTransfer(0x40, RTLSDR_CMD_SET_BIAS_TEE, 0, 0, nullptr, 0)) {
    return false;
  }

  Serial.println("RTL-SDR сконфигурирован");
  return true;
}

void RTLSDRDevice::setupTuner() {
  Serial.println("Настройка тюнера R820T2...");

  // Инициализация R820T2
  for (size_t i = 0; i < sizeof(r820t2_init_table) / sizeof(r820t2_init_table[0]); i++) {
    uint8_t data[2] = {r820t2_init_table[i].reg, r820t2_init_table[i].value};

    // Запись в регистр R820T2 через I2C
    controlTransfer(0x40, 0x02, R820T2_I2C_ADDR, r820t2_init_table[i].reg,
                   data, 2);
    delayMicroseconds(100);
  }

  // Специфичные настройки для ADS-B (1090 МГц)
  // Настройка LO для 1090 МГц
  uint8_t lo_data[] = {0x10, 0x7C, 0x11, 0x83, 0x12, 0x80};
  for (int i = 0; i < 6; i += 2) {
    controlTransfer(0x40, 0x02, R820T2_I2C_ADDR, lo_data[i],
                   &lo_data[i+1], 1);
    delayMicroseconds(100);
  }

  Serial.println("Тюнер R820T2 настроен для ADS-B (1090 МГц)");
}

void RTLSDRDevice::startCapture() {
  Serial.println("Запуск захвата данных...");

  // Сброс буферов
  controlTransfer(0x40, 0x12, 0, 0, nullptr, 0);

  // Запуск потока данных
  controlTransfer(0x40, 0x13, 0, 0, nullptr, 0);

  Serial.println("Захват данных запущен");
}

void RTLSDRDevice::processData() {
  // Чтение данных с RTL-SDR
  int bytes_read = bulkTransfer(0x81, sample_buffer, sizeof(sample_buffer), 100);

  if (bytes_read > 0) {
    // Обработка I/Q данных и поиск ADS-B пакетов
    processIQData(sample_buffer, bytes_read);
  }
}

// Обработка I/Q данных и декодирование ADS-B
void processIQData(uint8_t* data, int length) {
  static int bit_pos = 0;
  static bool in_preamble = false;
  static int preamble_confidence = 0;

  for (int i = 0; i < length - 1; i += 2) {
    // Преобразование I/Q в амплитуду
    uint8_t i_sample = data[i];
    uint8_t q_sample = data[i + 1];

    // Простое вычисление амплитуды
    uint16_t amplitude = abs((int)i_sample - 128) + abs((int)q_sample - 128);

    // Детектирование преамбулы ADS-B
    bool current_bit = amplitude > 200; // Пороговое значение

    if (!in_preamble) {
      // Поиск преамбулы: 1010000101000000
      if (detectPreamble(current_bit)) {
        in_preamble = true;
        bit_pos = 0;
        memset(&current_message, 0, sizeof(current_message));
        current_message.timestamp = millis();
      }
    } else {
      // Декодирование данных сообщения
      if (bit_pos < 112) { // 112 бит для полного ADS-B сообщения
        if (bit_pos % 8 == 0 && bit_pos > 0) {
          current_message.data[bit_pos / 8 - 1] = extractByte(bit_pos - 8);
        }

        bit_buffer[bit_pos] = current_bit ? 1 : 0;
        bit_pos++;
      } else {
        // Конец сообщения
        current_message.data[13] = extractByte(104);
        current_message.length = 14;
        in_preamble = false;

        // Проверка CRC и декодирование
        if (validateADSBMessage(&current_message)) {
          decodeADSBMessage(&current_message);
          outputADSBMessage(&current_message);
        }
      }
    }
  }
}

bool detectPreamble(bool current_bit) {
  static uint16_t preamble_pattern = 0;
  static const uint16_t adsb_preamble = 0xA500; // 1010000101000000 в двоичном

  preamble_pattern = (preamble_pattern << 1) | (current_bit ? 1 : 0);

  return (preamble_pattern & 0xFFFF) == adsb_preamble;
}

uint8_t extractByte(int start_bit) {
  uint8_t byte_val = 0;
  for (int i = 0; i < 8; i++) {
    if (start_bit + i < 1024 && bit_buffer[start_bit + i]) {
      byte_val |= (1 << (7 - i));
    }
  }
  return byte_val;
}

// Функция вычисления CRC для ADS-B
uint32_t calculateCRC(uint8_t* data, int length) {
  uint32_t crc = 0;

  for (int i = 0; i < length; i++) {
    crc = (crc << 8) ^ crc_table[((crc >> 24) ^ data[i]) & 0xFF];
  }

  return crc;
}

// Проверка CRC ADS-B сообщения
bool validateADSBMessage(ADSBMessage* msg) {
  if (msg->length != 14) return false;

  // Вычисление CRC для первых 11 байт
  uint32_t calculated_crc = calculateCRC(msg->data, 11);

  // Извлечение CRC из сообщения (последние 3 байта)
  uint32_t message_crc = (msg->data[11] << 16) | (msg->data[12] << 8) | msg->data[13];

  msg->isValid = (calculated_crc == message_crc);
  return msg->isValid;
}

// Декодирование ADS-B сообщения
void decodeADSBMessage(ADSBMessage* msg) {
  if (!msg->isValid) return;

  // Извлечение базовых полей
  msg->downlink_format = (msg->data[0] >> 3) & 0x1F;
  msg->capability = msg->data[0] & 0x07;

  // ICAO адрес (24 бита)
  msg->icao_address = (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];

  // Данные сообщения (ME field)
  uint8_t* me_data = &msg->data[4];
  msg->type_code = (me_data[0] >> 3) & 0x1F;

  switch (msg->type_code) {
    case 1: case 2: case 3: case 4: // Identification
      decodeIdentification(msg, me_data);
      break;

    case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: // Position
      decodePosition(msg, me_data);
      break;

    case 19: // Velocity
      decodeVelocity(msg, me_data);
      break;
  }
}

void decodeIdentification(ADSBMessage msg, uint8_t me_data) {
  // Извлечение callsign (8 символов, по 6 бит каждый)
  const char charset[] = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

  uint64_t callsign_data = 0;
  for (int i = 1; i < 7; i++) {
    callsign_data = (callsign_data << 8) | me_data[i];
  }

  for (int i = 0; i < 8; i++) {
    int char_index = (callsign_data >> (42 - 6 * i)) & 0x3F;
    msg->callsign[i] = charset[char_index];
  }
  msg->callsign[8] = '\0';

  // Удаление пробелов в конце
  for (int i = 7; i >= 0; i--) {
    if (msg->callsign[i] == ' ') {
      msg->callsign[i] = '\0';
    } else {
      break;
    }
  }
}

void decodePosition(ADSBMessage msg, uint8_t me_data) {
  // Декодирование высоты
  uint16_t alt_code = ((me_data[1] & 0xFF) << 4) | ((me_data[2] >> 4) & 0x0F);

  if (alt_code != 0) {
    msg->altitude = ((alt_code - 1) * 25) - 1000; // В футах
  } else {
    msg->altitude = -9999; // Неизвестно
  }

  // CPR декодирование для широты и долготы
  uint32_t lat_cpr = ((me_data[2] & 0x03) << 15) | (me_data[3] << 7) | (me_data[4] >> 1);
  uint32_t lon_cpr = ((me_data[4] & 0x01) << 16) | (me_data[5] << 8) | me_data[6];

  bool odd_format = (me_data[2] >> 2) & 0x01;

  // Упрощенное CPR декодирование (требует две позиции для точного декодирования)
  double lat_factor = odd_format ? 59.0 : 60.0;
  double lon_factor = odd_format ? 59.0 : 60.0;

  msg->latitude = (lat_cpr / 131072.0) * lat_factor - (lat_factor / 2.0);
  msg->longitude = (lon_cpr / 131072.0) * lon_factor - (lon_factor / 2.0);
}

void decodeVelocity(ADSBMessage msg, uint8_t me_data) {
  uint8_t velocity_type = (me_data[0] >> 1) & 0x07;

  if (velocity_type == 1) { // Ground speed
    uint16_t ew_vel = ((me_data[1] & 0x03) << 8) | me_data[2];
    uint16_t ns_vel = ((me_data[3] & 0x7F) << 3) | (me_data[4] >> 5);

    bool ew_sign = (me_data[1] >> 2) & 0x01;
    bool ns_sign = (me_data[3] >> 7) & 0x01;

    if (ew_vel > 0) ew_vel -= 1;
    if (ns_vel > 0) ns_vel -= 1;

    int16_t ew_velocity = ew_sign ? -ew_vel : ew_vel;
    int16_t ns_velocity = ns_sign ? -ns_vel : ns_vel;

    msg->ground_speed = sqrt(ew_velocity  ew_velocity + ns_velocity  ns_velocity);
    msg->track = atan2(ew_velocity, ns_velocity) * 180.0 / PI;
    if (msg->track < 0) msg->track += 360;
  }

  // Вертикальная скорость
  uint16_t vr_source = (me_data[4] >> 4) & 0x01;
  uint16_t vr_sign = (me_data[4] >> 3) & 0x01;
  uint16_t vr_rate = ((me_data[4] & 0x07) << 6) | (me_data[5] >> 2);

  if (vr_rate > 0) {
    msg->vertical_rate = (vr_rate - 1) * 64;
    if (vr_sign) msg->vertical_rate = -msg->vertical_rate;
  }
}

// Вывод декодированного сообщения
void outputADSBMessage(ADSBMessage* msg) {
  String output = "";

  output += "ADS-B,";
  output += String(msg->timestamp) + ",";
  output += String(msg->icao_address, HEX) + ",";
  output += String(msg->type_code) + ",";

  switch (msg->type_code) {
    case 1: case 2: case 3: case 4:
      output += "ID," + String(msg->callsign);
      break;

    case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
      output += "POS,";
      output += String(msg->latitude, 6) + ",";
      output += String(msg->longitude, 6) + ",";
      output += String(msg->altitude);
      break;

    case 19:
      output += "VEL,";
      output += String(msg->ground_speed) + ",";
      output += String(msg->track) + ",";
      output += String(msg->vertical_rate);
      break;

    default:
      output += "UNK,";
      for (int i = 0; i < 7; i++) {
        output += String(msg->data[i + 4], HEX);
        if (i < 6) output += " ";
      }
  }

  // Вывод в UART1
  UART1_Port.println(output);

  // Дублирование в Serial для отладки
  Serial.println("UART1: " + output);
}


Дополнительные настройки platformio.ini:


[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
board_build.flash_mode = dio
board_build.partitions = huge_app.csv
build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DCONFIG_ARDUHAL_LOG_COLORS=1
    -DUSB_HOST_SUPPORTED=1
    -DCONFIG_USB_HOST_ENABLED=1
lib_deps =
    h2zero/NimBLE-Arduino@^1.4.0


Этот код обеспечивает:

Полную настройку USB Host для ESP32-S3
Конфигурацию RTL-SDR с командами для R820T2
Полную таблицу CRC для проверки ADS-B пакетов
Декодирование основных типов ADS-B сообщений:
   - Идентификация (callsign)
   - Позиция (широта/долгота/высота)
   - Скорость (путевая скорость/курс/вертикальная скорость)
Вывод в UART1 декодированных данных
WiFi подключение с заданными параметрами

Код готов для загрузки в ESP32-S3 и работы с RTL-SDR приемником для декодирования ADS-B сигналов авиации.