/*
====================================================================
 PRO FIX v7.9-4RELAY-CFG
 ESP32 + SIM800L (.AMR audio) + 4 RELAYS + MULTI-SMS + WATCHDOG + OTA

 Изменения v7.9:
  - Повторная отправка аварийных SMS (до 3 попыток: через 1, 5 и 15 мин)
  - VER: время работы, причина последней перезагрузки, свободная память
  - Контроль уровня сигнала GSM (CSQ) и SIM-карты (AT+CPIN?) — предусмотрен, но выключен (DIAG_ENABLED 0)
  - Управление реле по SMS: PC1/PC2 POWER|REBOOT|OFF, SCR1/SCR2 RESET
  - Команда STATUS
  - SMS "НА СВЯЗИ" раз в неделю (HB ON/OFF, HB ADD/DEL, HB LIST, HB NOW)

 Изменения v7.8:
  - Длинные SMS автоматически делятся на части по 70 символов (UCS2)
  - Тексты SMS сокращены (статус фаз помещается в одну SMS)
  - Реле обслуживаются внутри всех ожиданий -> импульс Power не растягивается
  - configureModem и SMS о чужом звонке больше не вызываются из парсера UART
  - Проверка MD5 при OTA (необязательна) и PIN для SMS-команд — предусмотрены, но отключены
  - Очередь входящих SMS + периодический поиск "забытых" SMS на SIM
  - Очистка SIM от старых SMS при загрузке
  - Аппаратный watchdog ESP32 (Task WDT)
  - Реле не щёлкают при старте, реинициализация модема не рвёт звонок
  - Нельзя удалить последний номер из списка SMS
====================================================================
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_timer.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#define FW_VERSION "v7.9-4RELAY-CFG"

// ================= SIM800 =================
HardwareSerial sim800(2);
#define SIM800_RST_PIN    4      // Аппаратный сброс модема (RST)
#define SIM800_RX_PIN    16
#define SIM800_TX_PIN    17

// ================= RELAYS (4 канала — фактическая схема платы) =================
#define RELAY_PC_1       25     // Реле 1: ПК экрана 1 (Power SW)
#define RELAY_PC_2       26     // Реле 2: ПК экрана 2 (Power SW)
#define RELAY_SCREEN_A   32     // Реле 3: Питание Экрана 1 (220V) отключает питание на 60 секунд
#define RELAY_SCREEN_B   33     // Реле 4: Питание Экрана 2 (220V) отключает питание на 60 секунд

#define RELAY_COUNT      4
#define RELAY_ACTIVE_HIGH 1

// ================= PHASES =================
#define PHASE_A 21
#define PHASE_B 22
#define PHASE_C 23

// ================= TIMINGS ===============
#define TIMEOUT_SCREEN_PWR 60000UL  // Отключение питания экрана (60 сек)
#define PULSE_PC_SHORT       500UL  // Короткий импульс Power = вкл/выкл ПК
#define PULSE_PC_LONG_DEF   5000UL  // Заводское значение длинного импульса (мс)
#define PULSE_PC_LONG_MIN   1000UL  
#define PULSE_PC_LONG_MAX  30000UL  
#define PC_REBOOT_PAUSE    5000UL   // Пауза между выключением и включением ПК при перезагрузке

unsigned long pulsePcLong = PULSE_PC_LONG_DEF;

#define DTMF_GUARD           600UL
#define CALL_TIMEOUT      180000UL
#define PHASE_DEBOUNCE      5000UL
#define GREETING_DELAY      1500UL  

// Watchdog
#define WDG_PING_EVERY     20000UL  
#define WDG_CREG_EVERY     60000UL  
#define WDG_UART_DEAD      90000UL  
#define WDG_REG_DEAD      240000UL  

// Аппаратный watchdog ESP32: перезагрузка, если loop() завис дольше этого времени
#define HW_WDT_TIMEOUT_S      60

// Периодический поиск SMS, которые остались на SIM (потерянные +CMTI)
#define SMS_SCAN_EVERY     600000UL

// Максимум символов в одной SMS (UCS2 = 70)
#define SMS_MAX_UNITS        70     // кириллица (UCS2)
#define SMS_MAX_GSM         160     // только латиница/цифры (GSM 7-bit)
#define SMS_PART_GAP       3000UL   // пауза между частями длинного сообщения

// Повторная отправка аварийных SMS при ошибке
#define SMS_RETRY_SLOTS      12
#define SMS_RETRY_MAX         3     // сколько повторов после первой неудачи
const unsigned long SMS_RETRY_DELAYS[SMS_RETRY_MAX] = { 60000UL, 300000UL, 900000UL }; // 1, 5, 15 мин

// Контроль сигнала и SIM-карты
// 0 = ВЫКЛЮЧЕН: нет SMS-тревог о сигнале/SIM и нет перезапуска модема из-за SIM.
//     Уровень сигнала и SIM всё равно опрашиваются, чтобы показывать их в STATUS.
// 1 = включён.
#define DIAG_ENABLED          0
#define DIAG_EVERY        300000UL  // проверка CSQ и SIM каждые 5 минут
#define CSQ_LOW               8     // ниже — "слабый сигнал" (0..31, 99 = нет сигнала)
#define CSQ_OK               12     // выше или равно — "сигнал в норме"
#define CSQ_BAD_CHECKS        3     // столько плохих проверок подряд (15 мин) -> тревога
#define SIM_BAD_CHECKS        2     // столько проверок без SIM подряд -> перезапуск модема

// SMS "НА СВЯЗИ" (heartbeat)
#define HB_INTERVAL_DAYS      7
#define HB_INTERVAL_MS     ((unsigned long)HB_INTERVAL_DAYS * 24UL * 3600UL * 1000UL)
#define HB_DEFAULT_ON         1     // включено по умолчанию
#define SMS_MAX_PARTS         8

#define DENIED_SMS_COOLDOWN 600000UL 
#define CREG_POLL_TMO        1500UL

// ================= GPRS / OTA =================
#define GPRS_APN  "internet"
#define GPRS_USER ""
#define GPRS_PASS ""

#define OTA_CHUNK_SIZE     512
#define OTA_MAX_BYTES      (1536UL * 1024UL)
#define OTA_HDR_MAX        1024
#define OTA_TOTAL_TMO      600000UL

#define SCREEN_COUNT 2

#if (SCREEN_COUNT < 1) || (SCREEN_COUNT > 2)
  #error "SCREEN_COUNT must be 1 or 2"
#endif

#define DEVICE_ADDRESS "Блюхера-Дарвина"   // Заводской адрес (меняется SMS: SET ADDR ...)
#define ADDR_MAX_CHARS 40                     // Чтобы статус фаз помещался в одну SMS
String deviceAddress = DEVICE_ADDRESS;        // Текущий адрес (хранится в NVS)

// ================= БЕЗОПАСНОСТЬ SMS-КОМАНД =================
// PIN для SMS-команд. Сейчас ОТКЛЮЧЁН (пустая строка) — команды пишутся без PIN: "LIST".
// Чтобы включить: укажите PIN, например "1234", тогда команды будут вида "1234 LIST".
#define CMD_PIN ""

// MD5 для OTA сейчас НЕ обязателен: "OTA http://host/fw.bin".
// Если указать MD5 ("OTA http://host/fw.bin <md5>"), он всё равно будет проверен.
// Чтобы сделать MD5 обязательным, поставьте 1.
#define OTA_REQUIRE_MD5 0

// ================= НОМЕРА ДЛЯ СМС (заводской список) =================
const char* TARGET_PHONES[] = {
  "+79514629028",
  "+79000621198"
};
const int TARGET_COUNT = sizeof(TARGET_PHONES) / sizeof(TARGET_PHONES[0]);

// ================= БЕЛЫЙ СПИСОК ЗВОНКОВ (заводской список) =================
const char* ALLOWED[] = {
  "79514629028",
  "79000621198"
};
const int ALLOWED_COUNT = sizeof(ALLOWED) / sizeof(ALLOWED[0]);

#define PHONE_MAX 16
char smsList[PHONE_MAX][16];
char hbList[PHONE_MAX][16];          // кому слать "НА СВЯЗИ" (пусто = список SMS)
int  hbCount = 0;
bool hbEnabled = HB_DEFAULT_ON;
unsigned long lastHbT = 0;
char callList[PHONE_MAX][16];
int smsCount = 0;
int callCount = 0;
Preferences phonePrefs;
Preferences cfgPrefs;

// ================= STATE ==================
enum MenuState { ROOT_MENU, MENU_A, MENU_B, MENU_C };
MenuState menu = ROOT_MENU;

bool callActive   = false;
unsigned long lastDTMF   = 0;
unsigned long callStart  = 0;

bool stablePhaseA = true, stablePhaseB = true, stablePhaseC = true;
bool lastReadA = true, lastReadB = true, lastReadC = true;
unsigned long phaseDebounceTimer = 0;

bool phaseSMSPending = false;
bool reinitPending   = false;
bool cfunPending     = false;
bool smsBusy         = false;
bool otaBusy         = false;
bool otaPending      = false;
// Очередь входящих SMS (индексы на SIM)
#define SMS_QUEUE_LEN 8
int  smsQueue[SMS_QUEUE_LEN];
int  smsQHead = 0, smsQCount = 0;
bool smsScanPending = false;
unsigned long lastSmsScan = 0;

// Очередь повторной отправки SMS
struct SmsRetry {
  bool used;
  char number[18];
  String body;
  byte attempts;
  unsigned long nextTry;
};
SmsRetry smsRetry[SMS_RETRY_SLOTS];

// Исходящие уведомления (отправляются из loop, когда модем свободен)
#define OUTBOX_LEN 6
String outboxBody[OUTBOX_LEN];
byte   outboxTarget[OUTBOX_LEN];   // 0 = список SMS, 1 = список HB
int    outboxCount = 0;

// Диагностика сигнала / SIM
int  lastCsq = 99;
bool csqFresh = false;
byte csqBadCount = 0;
bool csqAlarm = false;
bool simReady = true;
bool cpinFresh = false;
bool cpinAwait = false;
unsigned long cpinAskT = 0;
byte simBadCount = 0;
bool simLost = false;
unsigned long lastDiagT = 0;
unsigned long diagCpinAt = 0;

esp_reset_reason_t bootReason = ESP_RST_UNKNOWN;

bool cfgReapplyPending = false;     // configureModem() после восстановления сети
bool deniedSmsPending  = false;     // SMS о чужом звонке (отправляется из loop)
String deniedSmsBody;

String otaUrl;
String otaMd5;
String otaRequester;

bool pendingGreeting       = false;
unsigned long pendingGreetingTime = 0;

unsigned long lastPingT = 0;
unsigned long lastCregT = 0;
unsigned long lastDeniedNotify = 0;

unsigned long lastModemOK      = 0;
unsigned long lastRegOK        = 0;
bool cregRegistered           = false;
bool cregFresh                = false;
bool waitOkFlag               = false;
bool waitErrFlag              = false;
bool smsDoneFlag              = false;
bool smsSuccessFlag           = false;
static bool cfgAppliedOnce    = false;

const byte outputPins[RELAY_COUNT] = {
  RELAY_PC_1,
  RELAY_PC_2,
  RELAY_SCREEN_A,
  RELAY_SCREEN_B
};

bool          pinState[RELAY_COUNT]    = {0};
unsigned long pinTimer[RELAY_COUNT]    = {0};
unsigned long pinDuration[RELAY_COUNT] = {0};

// Перезагрузка ПК: 0 = нет, 1 = идёт длинный импульс (выкл), 2 = пауза перед коротким (вкл)
byte          pcRebootStage[2] = {0, 0};
unsigned long pcRebootTimer[2] = {0, 0};

// Прототипы функций
void serviceBackground();
bool sendAlert(const String& number, const String& body);
void sendToList(char list[][16], int count, const String& body);
void retryAdd(const String& number, const String& body);
void retryService();
void outboxAdd(const String& body, byte target);
void outboxService();
void checkDiagnostics();
void heartbeatService();
String buildStatus(const String& title = "");
String uptimeStr();
String resetReasonStr(esp_reset_reason_t r);
bool pcPower(int screen);
bool pcReboot(int screen);
bool pcForceOff(int screen);
bool screenPowerCycle(int screen);
bool pcBusy(int screen);
String handleRelayCommand(const String& upper);
String handleHbCommand(const String& upper, const String& cmdLine);
void safeDelay(unsigned long ms);
void hwWdtInit();
bool smsEnqueue(int idx);
int  smsDequeue();
void scanStoredSms();
bool sendSMSPart(const String& number, const String& body);
int  ucs2Units(uint8_t leadByte);
bool isGsmSafe(const String& s);
bool sendSMSSegment(const String& number, const String& body);
String maybeUcs2HexNumber(const String& s);
void relayOn(byte i);
void relayOff(byte i);
void pumpLines();
void handleLine(String line);
bool waitOK(uint32_t tmo);
bool queryCREG();
void configureModem();
void initSIM();
void resetSIMNetwork();
void runWatchdog();
void readSIM();
void handleDTMF(char d);
void pulseOutput(int index, unsigned long duration);
bool anyOutputBusy();
void outputHandler();
void checkTimeout();
void checkPhases();
String buildPhaseStatus();
void tryFlushPendingSMS();
bool sendSMS(const String& number, const String& body);
String utf8ToUcs2Hex(const String& utf8);
bool waitPrompt(uint32_t tmo);
void sendSMSToAll(const String& body);
bool isNumberAllowed(String incoming);
void sendAT(const char* cmd);
void stopAmr();
void playAmr(int fileNum);
void serviceGreeting();
String extractQuoted(const String& s, int nth);
String digitsOnly(const String& raw);
bool isPhoneAllowed(const String& digits);
String toSmsNumber(const String& digits);
String maybeUcs2HexToText(const String& s);
bool waitModemToken(const char* token, uint32_t tmo);
bool readExactly(uint8_t* buf, int n, uint32_t tmo);
bool parseHttpUrl(const String& url, String& host, int& port, String& path);
bool gprsAttach();
void gprsShutdown();
bool tcpOpen(const String& host, int port);
int  cipRead(uint8_t* buf, int maxN, int& remaining);
void processIncomingSms(int index);
void performOta();
void normalizePhone(char* dst, const char* src);
int findPhone(char list[][16], int count, const char* digits);
bool phonesSave();
void phonesSeedFactory();
void phonesLoad();
bool phonesAdd(char list[][16], int& count, const char* digits);
bool phonesDel(char list[][16], int& count, const char* digits, bool keepLast);
String listPhones(char list[][16], int count, bool plus);
String handlePhoneCommand(const String& upper, const String& cmdLine);
void cfgLoad();
bool cfgSave();
String handleCfgCommand(const String& upper, const String& cmdLine);

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  hwWdtInit();
  bootReason = esp_reset_reason();
  delay(500);

  pinMode(SIM800_RST_PIN, OUTPUT);
  digitalWrite(SIM800_RST_PIN, HIGH);

  pinMode(PHASE_A, INPUT_PULLUP);
  pinMode(PHASE_B, INPUT_PULLUP);
  pinMode(PHASE_C, INPUT_PULLUP);

  // Сначала задаём уровень "выключено", потом включаем выход — без щелчка реле
  for (int i = 0; i < RELAY_COUNT; i++) {
    relayOff(i);
    pinMode(outputPins[i], OUTPUT);
    relayOff(i);
  }

  stablePhaseA = (digitalRead(PHASE_A) == LOW);
  stablePhaseB = (digitalRead(PHASE_B) == LOW);
  stablePhaseC = (digitalRead(PHASE_C) == LOW);
  lastReadA = stablePhaseA;
  lastReadB = stablePhaseB;
  lastReadC = stablePhaseC;

  sim800.begin(9600, SERIAL_8N1, SIM800_RX_PIN, SIM800_TX_PIN);
  delay(1000);

  phonesLoad();
  cfgLoad();

  initSIM();

  // Старые SMS, пришедшие до перезагрузки, не выполняем — просто удаляем
  sendAT("AT+CMGF=1");           waitOK(1000);
  sendAT("AT+CMGDA=\"DEL ALL\""); waitOK(5000);
  lastSmsScan = millis();

  String bootMsg = "ЗАПУСК (" + resetReasonStr(bootReason) + ")\n" + buildPhaseStatus();
  sendSMSToAll(bootMsg);
  lastHbT = millis();
  lastDiagT = millis();

  Serial.println(F("PRO FIX " FW_VERSION " READY"));
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  esp_task_wdt_reset();
  readSIM();          
  if (!otaBusy) runWatchdog();
  outputHandler();
  checkTimeout();
  checkPhases();
  serviceGreeting();  

  if (cfgReapplyPending && !callActive && !smsBusy && !otaBusy) {
    cfgReapplyPending = false;
    Serial.println(F("[Net] Network restored -> Re-applying config"));
    configureModem();
  }

  if (smsQCount > 0 && !callActive && !smsBusy && !otaBusy) {
    int idx = smsDequeue();
    if (idx >= 0) processIncomingSms(idx);
  }

  if (smsQCount == 0 && !callActive && !smsBusy && !otaBusy && !anyOutputBusy() &&
      (smsScanPending || millis() - lastSmsScan > SMS_SCAN_EVERY)) {
    smsScanPending = false;
    lastSmsScan = millis();
    scanStoredSms();
  }

  if (deniedSmsPending && !callActive && !smsBusy && !otaBusy) {
    deniedSmsPending = false;
    sendSMSToAll(deniedSmsBody);
    deniedSmsBody = "";
  }

  if (otaPending && !callActive && !anyOutputBusy() && !smsBusy && !otaBusy) {
    otaPending = false;
    performOta();
  }

  // Реинициализацию модема не делаем во время звонка
  if (reinitPending && !otaBusy && !callActive) {
    reinitPending = false;
    initSIM();
  } else if (cfunPending && !otaBusy && !callActive) {
    cfunPending = false;
    resetSIMNetwork();
  }

  tryFlushPendingSMS();
  checkDiagnostics();
  heartbeatService();
  outboxService();
  retryService();
}

// =====================================================
// РЕЛЕ С УЧЕТОМ ПОЛЯРНОСТИ
// =====================================================
void relayOn(byte i)  { digitalWrite(outputPins[i], RELAY_ACTIVE_HIGH ? HIGH : LOW); }
void relayOff(byte i) { digitalWrite(outputPins[i], RELAY_ACTIVE_HIGH ? LOW  : HIGH); }

// =====================================================
// ФОН: реле + watchdog. Вызывается во ВСЕХ циклах ожидания,
// чтобы импульсы реле не растягивались во время SMS/инициализации.
// =====================================================
void serviceBackground() {
  outputHandler();
  esp_task_wdt_reset();
}

void safeDelay(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    serviceBackground();
    delay(5);
  }
}

void hwWdtInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms = HW_WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  if (esp_task_wdt_init(&cfg) != ESP_OK) esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(HW_WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);   // следим за задачей loop()
}

// =====================================================
// ОЧЕРЕДЬ ВХОДЯЩИХ SMS
// =====================================================
bool smsEnqueue(int idx) {
  if (idx < 0) return false;
  for (int i = 0; i < smsQCount; i++) {
    if (smsQueue[(smsQHead + i) % SMS_QUEUE_LEN] == idx) return true;
  }
  if (smsQCount >= SMS_QUEUE_LEN) {
    smsScanPending = true;   // не влезло — подберём позже через AT+CMGL
    return false;
  }
  smsQueue[(smsQHead + smsQCount) % SMS_QUEUE_LEN] = idx;
  smsQCount++;
  return true;
}

int smsDequeue() {
  if (smsQCount == 0) return -1;
  int idx = smsQueue[smsQHead];
  smsQHead = (smsQHead + 1) % SMS_QUEUE_LEN;
  smsQCount--;
  return idx;
}

// Находит SMS, оставшиеся на SIM (например, если +CMTI потерялся)
void scanStoredSms() {
  smsBusy = true;
  sendAT("AT+CMGF=1");
  waitOK(1000);
  sendAT("AT+CMGL=\"ALL\",1");   // 1 = не менять статус сообщений
  unsigned long t = millis();
  String acc;
  bool done = false;
  while (!done && millis() - t < 8000) {
    serviceBackground();
    while (sim800.available()) {
      char c = (char)sim800.read();
      if (c == '\r') continue;
      if (c == '\n') {
        acc.trim();
        if (acc.startsWith("+CMGL:")) {
          int idx = acc.substring(6).toInt();
          if (idx > 0) smsEnqueue(idx);
        } else if (acc == "OK" || acc.indexOf("ERROR") >= 0) {
          if (acc == "OK") lastModemOK = millis();
          done = true;
          break;
        }
        acc = "";
      } else {
        acc += c;
      }
    }
  }
  smsBusy = false;
  if (smsQCount > 0) {
    Serial.print(F("[SMS] Found stored: "));
    Serial.println(smsQCount);
  }
}

// =====================================================
// АСИНХРОННЫЙ ПАРСЕР UART
// =====================================================
void pumpLines() {
  serviceBackground();
  sim800.setTimeout(20);
  while (sim800.available()) {
    String line = sim800.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    handleLine(line);
  }
}

void readSIM() { pumpLines(); }

void handleLine(String line) {
  if (!otaBusy) Serial.println(line);

  if (line.equals("OK")) {
    lastModemOK = millis();
    waitOkFlag = true;
  }
  if (line.indexOf("ERROR") >= 0) {
    waitErrFlag = true;
  }

  if (line.indexOf("+CREG:") >= 0) {
    int comma = line.indexOf(',');
    if (comma > 0) {
      int d = line.charAt(comma + 1) - '0';
      bool reg = (d == 1 || d == 5);
      if (reg) lastRegOK = millis();
      if (reg != cregRegistered) {
        cregRegistered = reg;
        if (reg && cfgAppliedOnce) {
          cfgReapplyPending = true;   // выполняется из loop(), не из парсера
        } else if (reg) {
          cfgAppliedOnce = true;
        }
      }
      cregFresh = true;
    }
  }

  if (line.indexOf("NORMAL POWER DOWN") >= 0 ||
      line.indexOf("UNDER-VOLTAGE")   >= 0 ||
      line.indexOf("UNDER VOLTAGE")   >= 0) {
    Serial.println(F("[POWER] Power drop detected on SIM800L!"));
    if (!reinitPending && !smsBusy && !otaBusy) reinitPending = true;
  }

  if (line.indexOf("+CMTI:") >= 0) {
    int comma = line.lastIndexOf(',');
    if (comma >= 0) {
      int idx = line.substring(comma + 1).toInt();
      smsEnqueue(idx);
      Serial.print(F("[SMS] Incoming index "));
      Serial.println(idx);
    }
    return;
  }

  if (line.indexOf("+CLIP") >= 0) {
    if (otaBusy) return;
    if (callActive) return;
    if (isNumberAllowed(line)) {
      callActive = true;
      callStart  = millis();
      sendAT("ATA");
      menu = ROOT_MENU;
      pendingGreeting     = true;
      pendingGreetingTime = millis();
    } else {
      sendAT("ATH");
      if (millis() - lastDeniedNotify > DENIED_SMS_COOLDOWN) {
        lastDeniedNotify = millis();
        String body = deviceAddress + "\nЧУЖОЙ ЗВОНОК";
        int q1i = line.indexOf('"');
        int q2i = line.indexOf('"', q1i + 1);
        if (q1i >= 0 && q2i > q1i) body += ": " + maybeUcs2HexNumber(line.substring(q1i + 1, q2i));
        deniedSmsBody = body;
        deniedSmsPending = true;      // отправится из loop()
      }
    }
    return;
  }

  if (line.indexOf("NO CARRIER") >= 0 || line.indexOf("BUSY") >= 0 || line.indexOf("NO ANSWER") >= 0) {
    callActive = false;
    pendingGreeting = false;
    menu = ROOT_MENU;
    stopAmr();
    return;
  }

  if (line.indexOf("+DTMF:") >= 0) {
    char d = 0;
    for (int i = line.length() - 1; i >= 0; i--) {
      char c = line.charAt(i);
      if ((c >= '0' && c <= '9') || c == '*' || c == '#' || (c >= 'A' && c <= 'D')) { d = c; break; }
    }
    if (d) handleDTMF(d);
  }

  if (line.startsWith("+CSQ:")) {
    int v = line.substring(5).toInt();
    if (v >= 0 && v <= 99) { lastCsq = v; csqFresh = true; }
  }
  if (line.startsWith("+CPIN:")) {
    simReady = (line.indexOf("READY") >= 0);
    cpinFresh = true;
    cpinAwait = false;
  } else if (cpinAwait && line.indexOf("+CME ERROR") >= 0) {
    simReady = false;               // например "+CME ERROR: 10" = SIM не вставлена
    cpinFresh = true;
    cpinAwait = false;
  }

  if (line.indexOf("+CMGS:")    >= 0) { smsDoneFlag = true; smsSuccessFlag = true; }
  if (line.indexOf("+CMS ERROR") >= 0) { smsDoneFlag = true; smsSuccessFlag = false; }
}

void serviceGreeting() {
  if (pendingGreeting && callActive &&
      (millis() - pendingGreetingTime > GREETING_DELAY)) {
    pendingGreeting = false;
    playAmr(1);
  }
}

bool waitOK(uint32_t tmo) {
  waitOkFlag = false;
  waitErrFlag = false;
  unsigned long t = millis();
  while (millis() - t < tmo) {
    pumpLines();
    if (waitOkFlag)  return true;
    if (waitErrFlag) return false;
  }
  return false;
}

bool queryCREG() {
  cregFresh = false;
  sim800.println("AT+CREG?");
  unsigned long t = millis();
  while (millis() - t < CREG_POLL_TMO) {
    pumpLines();
    if (cregFresh) break;
  }
  return cregRegistered;
}

bool waitPrompt(uint32_t tmo) {
  unsigned long t = millis();
  String buf;
  while (millis() - t < tmo) {
    serviceBackground();
    while (sim800.available()) {
      char c = sim800.read();
      if (c == '>') return true;
      if (c == '\r') continue;
      if (c == '\n') {
        buf.trim();
        if (buf.length() > 0) handleLine(buf);
        buf = "";
      } else {
        buf += c;
      }
    }
  }
  buf.trim();
  if (buf.length() > 0) handleLine(buf);
  return false;
}

String utf8ToUcs2Hex(const String& utf8) {
  String hex = "";
  size_t i = 0;
  size_t len = utf8.length();
  while (i < len) {
    uint8_t c = (uint8_t)utf8[i];
    uint32_t codepoint = 0;
    uint8_t extraBytes = 0;

    if (c < 0x80)              { codepoint = c;        extraBytes = 0; }
    else if ((c & 0xE0) == 0xC0) { codepoint = c & 0x1F; extraBytes = 1; }
    else if ((c & 0xF0) == 0xE0) { codepoint = c & 0x0F; extraBytes = 2; }
    else if ((c & 0xF8) == 0xF0) { codepoint = c & 0x07; extraBytes = 3; }
    else { i++; continue; }

    i++;
    bool valid = true;
    for (uint8_t k = 0; k < extraBytes; k++) {
      if (i >= len) { valid = false; break; }
      uint8_t cc = (uint8_t)utf8[i];
      if ((cc & 0xC0) != 0x80) { valid = false; break; }
      codepoint = (codepoint << 6) | (cc & 0x3F);
      i++;
    }
    if (!valid) continue;

    char buf[5];
    if (codepoint <= 0xFFFF) {
      snprintf(buf, sizeof(buf), "%04X", (unsigned int)codepoint);
      hex += buf;
    } else {
      codepoint -= 0x10000;
      uint16_t hi = 0xD800 + (codepoint >> 10);
      uint16_t lo = 0xDC00 + (codepoint & 0x3FF);
      snprintf(buf, sizeof(buf), "%04X", hi); hex += buf;
      snprintf(buf, sizeof(buf), "%04X", lo); hex += buf;
    }
  }
  return hex;
}

// Сколько UCS2-единиц занимает символ по первому байту UTF-8
int ucs2Units(uint8_t c) {
  if ((c & 0xF8) == 0xF0) return 2;   // суррогатная пара
  return 1;
}
int utf8Len(uint8_t c) {
  if (c < 0x80) return 1;
  if ((c & 0xE0) == 0xC0) return 2;
  if ((c & 0xF0) == 0xE0) return 3;
  if ((c & 0xF8) == 0xF0) return 4;
  return 1;
}

// Текст только из этих символов отправляется в GSM-кодировке (до 160 символов в SMS)
bool isGsmSafe(const String& s) {
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) continue;
    if (strchr(" \n+-():.,/!?=*#%&'\"<>;", c) && c != 0) continue;
    return false;
  }
  return true;
}

// Символ '\f' в тексте = принудительно начать новую SMS
bool sendSMS(const String& number, const String& body) {
  if (number.length() == 0) return false;
  bool allOk = true;
  int from = 0;
  bool first = true;
  while (from <= (int)body.length()) {
    int ff = body.indexOf('\f', from);
    String seg = (ff < 0) ? body.substring(from) : body.substring(from, ff);
    seg.trim();
    if (seg.length()) {
      if (!first) safeDelay(SMS_PART_GAP);
      if (!sendSMSSegment(number, seg)) allOk = false;
      first = false;
    }
    if (ff < 0) break;
    from = ff + 1;
  }
  return allOk;
}

// Делит текст на части: 160 символов для латиницы, 70 для кириллицы (по возможности — по строкам)
bool sendSMSSegment(const String& number, const String& body) {
  if (number.length() == 0) return false;
  const int maxUnits = isGsmSafe(body) ? SMS_MAX_GSM : SMS_MAX_UNITS;
  String rest = body;
  rest.trim();
  bool allOk = true;
  int parts = 0;
  while (rest.length() > 0 && parts < SMS_MAX_PARTS) {
    int units = 0;
    int i = 0;
    int cut = -1;          // байтовая позиция конца части
    int lastNl = -1;       // позиция последнего \n внутри части
    int len = rest.length();
    while (i < len) {
      uint8_t c = (uint8_t)rest[i];
      int u = ucs2Units(c);
      int bl = utf8Len(c);
      if (units + u > maxUnits) { cut = i; break; }
      if (c == '\n') lastNl = i;
      units += u;
      i += bl;
    }
    String part;
    if (cut < 0) {
      part = rest;
      rest = "";
    } else {
      int at = (lastNl > 0) ? lastNl : cut;
      part = rest.substring(0, at);
      rest = rest.substring(at);
      rest.trim();
    }
    part.trim();
    if (part.length() == 0) continue;
    if (parts > 0) safeDelay(SMS_PART_GAP);
    bool ok = sendSMSPart(number, part);
    if (!ok) {                        // одна повторная попытка для этой части
      Serial.println(F("[SMS] Part failed, retrying..."));
      safeDelay(SMS_PART_GAP);
      ok = sendSMSPart(number, part);
    }
    if (!ok) allOk = false;
    parts++;
  }
  return allOk;
}

bool sendSMSPart(const String& number, const String& body) {
  smsBusy = true;

  sim800.print(F("AT+CMGF=1\r"));
  safeDelay(100);
  pumpLines();

  const bool gsm = isGsmSafe(body);
  if (gsm) {
    sendAT("AT+CSCS=\"GSM\"");      waitOK(1000);
    sendAT("AT+CSMP=17,167,0,0");   waitOK(1000);   // 7-bit, до 160 символов
  } else {
    sendAT("AT+CSCS=\"UCS2\"");     waitOK(1000);
  }

  sim800.print(F("AT+CMGS=\""));
  sim800.print(gsm ? number : utf8ToUcs2Hex(number));
  sim800.print(F("\"\r"));

  if (!waitPrompt(8000)) {
    Serial.print(F("[SMS] Timeout waiting for '>' prompt: "));
    Serial.println(number);
    sim800.write(27);
    safeDelay(300);
    pumpLines();
    sendAT("AT+CSCS=\"GSM\"");
    waitOK(1000);
    if (gsm) { sendAT("AT+CSMP=17,167,0,8"); waitOK(1000); }
    smsBusy = false;
    return false;
  }

  if (gsm) sim800.print(body);
  else     sim800.print(utf8ToUcs2Hex(body));
  sim800.write(26);

  smsDoneFlag = false;
  smsSuccessFlag = false;
  unsigned long t = millis();
  while (millis() - t < 15000) {
    pumpLines();
    if (smsDoneFlag) break;
  }

  sendAT("AT+CSCS=\"GSM\"");
  waitOK(1000);
  if (gsm) { sendAT("AT+CSMP=17,167,0,8"); waitOK(1000); }   // обратно UCS2 по умолчанию

  smsBusy = false;
  if (!smsSuccessFlag) {
    Serial.print(F("[SMS] Send FAILED to "));
    Serial.println(number);
  }
  return smsSuccessFlag;
}

// =====================================================
// АВАРИЙНЫЕ SMS С ПОВТОРНОЙ ОТПРАВКОЙ
// =====================================================
bool sendAlert(const String& number, const String& body) {
  if (sendSMS(number, body)) return true;
  Serial.print(F("[SMS] Send failed, will retry: "));
  Serial.println(number);
  retryAdd(number, body);
  return false;
}

void sendToList(char list[][16], int count, const String& body) {
  for (int i = 0; i < count; i++) {
    sendAlert(toSmsNumber(String(list[i])), body);
    safeDelay(1500);
  }
}

void sendSMSToAll(const String& body) {
  sendToList(smsList, smsCount, body);
}

void retryAdd(const String& number, const String& body) {
  for (int i = 0; i < SMS_RETRY_SLOTS; i++) {
    if (!smsRetry[i].used) {
      smsRetry[i].used = true;
      strncpy(smsRetry[i].number, number.c_str(), sizeof(smsRetry[i].number) - 1);
      smsRetry[i].number[sizeof(smsRetry[i].number) - 1] = 0;
      smsRetry[i].body = body;
      smsRetry[i].attempts = 0;
      smsRetry[i].nextTry = millis() + SMS_RETRY_DELAYS[0];
      return;
    }
  }
  Serial.println(F("[SMS] Retry queue full, message dropped"));
}

// Одна повторная попытка за вызов, только когда модем свободен
void retryService() {
  if (callActive || smsBusy || otaBusy || reinitPending || cfunPending) return;
  unsigned long now = millis();
  for (int i = 0; i < SMS_RETRY_SLOTS; i++) {
    if (!smsRetry[i].used) continue;
    if ((long)(now - smsRetry[i].nextTry) < 0) continue;
    String num = String(smsRetry[i].number);
    Serial.print(F("[SMS] Retry #"));
    Serial.print(smsRetry[i].attempts + 1);
    Serial.print(F(" -> "));
    Serial.println(num);
    if (sendSMS(num, smsRetry[i].body)) {
      smsRetry[i].used = false;
      smsRetry[i].body = "";
    } else {
      smsRetry[i].attempts++;
      if (smsRetry[i].attempts >= SMS_RETRY_MAX) {
        Serial.println(F("[SMS] Retry limit reached, message dropped"));
        smsRetry[i].used = false;
        smsRetry[i].body = "";
      } else {
        smsRetry[i].nextTry = millis() + SMS_RETRY_DELAYS[smsRetry[i].attempts];
      }
    }
    return;
  }
}

// =====================================================
// ИСХОДЯЩИЕ УВЕДОМЛЕНИЯ (из loop, когда модем свободен)
// =====================================================
void outboxAdd(const String& body, byte target) {
  if (outboxCount >= OUTBOX_LEN) {
    Serial.println(F("[SMS] Outbox full, message dropped"));
    return;
  }
  outboxBody[outboxCount] = body;
  outboxTarget[outboxCount] = target;
  outboxCount++;
}

void outboxService() {
  if (outboxCount == 0) return;
  if (callActive || smsBusy || otaBusy || reinitPending || cfunPending) return;
  String body = outboxBody[0];
  byte target = outboxTarget[0];
  for (int i = 1; i < outboxCount; i++) {
    outboxBody[i - 1] = outboxBody[i];
    outboxTarget[i - 1] = outboxTarget[i];
  }
  outboxCount--;
  outboxBody[outboxCount] = "";
  if (target == 1 && hbCount > 0) sendToList(hbList, hbCount, body);
  else                            sendToList(smsList, smsCount, body);
}

// =====================================================
// КОНТРОЛЬ СИГНАЛА GSM И SIM-КАРТЫ
// =====================================================
void checkDiagnostics() {
  if (cpinAwait && millis() - cpinAskT > 5000) cpinAwait = false;

#if !DIAG_ENABLED
  // Контроль выключен: только сбрасываем флаги, без тревог и перезапусков
  csqFresh = false;
  cpinFresh = false;
  return;
#endif

  if (csqFresh) {
    csqFresh = false;
    bool bad = (lastCsq < CSQ_LOW || lastCsq == 99);
    if (bad) { if (csqBadCount < 255) csqBadCount++; }
    else csqBadCount = 0;

    if (!csqAlarm && csqBadCount >= CSQ_BAD_CHECKS) {
      csqAlarm = true;
      outboxAdd(deviceAddress + "\nСЛАБЫЙ СИГНАЛ GSM: " +
                (lastCsq == 99 ? String("нет") : String(lastCsq) + "/31"), 0);
    } else if (csqAlarm && lastCsq >= CSQ_OK && lastCsq != 99) {
      csqAlarm = false;
      outboxAdd(deviceAddress + "\nСИГНАЛ GSM В НОРМЕ: " + String(lastCsq) + "/31", 0);
    }
  }

  if (cpinFresh) {
    cpinFresh = false;
    if (!simReady) {
      simBadCount++;
      Serial.println(F("[SIM] SIM not ready!"));
      if (simBadCount >= SIM_BAD_CHECKS && !reinitPending) {
        Serial.println(F("[SIM] -> Scheduling modem reinit"));
        simLost = true;
        simBadCount = 0;
        reinitPending = true;
      }
    } else {
      simBadCount = 0;
      if (simLost) {
        simLost = false;
        outboxAdd(deviceAddress + "\nSIM-КАРТА БЫЛА НЕДОСТУПНА\nСВЯЗЬ ВОССТАНОВЛЕНА", 0);
      }
    }
  }
}

// =====================================================
// SMS "НА СВЯЗИ" РАЗ В НЕДЕЛЮ
// =====================================================
void heartbeatService() {
  if (!hbEnabled) return;
  if (millis() - lastHbT < HB_INTERVAL_MS) return;
  lastHbT = millis();
  outboxAdd(buildStatus("НА СВЯЗИ"), 1);
}

String uptimeStr() {
  uint64_t sec = (uint64_t)esp_timer_get_time() / 1000000ULL;
  unsigned long d = (unsigned long)(sec / 86400ULL);
  unsigned long h = (unsigned long)((sec % 86400ULL) / 3600ULL);
  unsigned long m = (unsigned long)((sec % 3600ULL) / 60ULL);
  return String(d) + "д " + String(h) + "ч " + String(m) + "м";
}

String resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "ВКЛ ПИТАНИЯ";
    case ESP_RST_EXT:       return "ВНЕШНИЙ СБРОС";
    case ESP_RST_SW:        return "ПРОГРАММНЫЙ/OTA";
    case ESP_RST_PANIC:     return "СБОЙ ПРОГРАММЫ";
    case ESP_RST_INT_WDT:   return "WATCHDOG INT";
    case ESP_RST_TASK_WDT:  return "WATCHDOG";
    case ESP_RST_WDT:       return "WATCHDOG";
    case ESP_RST_BROWNOUT:  return "ПРОСАДКА ПИТАНИЯ";
    case ESP_RST_DEEPSLEEP: return "ПРОБУЖДЕНИЕ";
    default:                return "НЕИЗВЕСТНО";
  }
}

// Короткий статус (помещается в одну SMS при адресе до ~20 символов):
//   Блюхера-Дарвина НА СВЯЗИ
//   A:ОК B:ОК C:ОК
//   GSM:18 SIM:ОК
//   РАБОТА 7д 0ч
String buildStatus(const String& title) {
  String s = deviceAddress;
  if (title.length()) s += " " + title;
  s += "\n";
  s += "A:" + String(stablePhaseA ? "ОК" : "НЕТ");
  s += " B:" + String(stablePhaseB ? "ОК" : "НЕТ");
  s += " C:" + String(stablePhaseC ? "ОК" : "НЕТ") + "\n";
  s += "GSM:" + (lastCsq == 99 ? String("нет") : String(lastCsq));
  s += " SIM:" + String(simReady ? "ОК" : "НЕТ") + "\n";
  String act = "";
  const char* names[RELAY_COUNT] = { "ПК1", "ПК2", "Э1", "Э2" };
  for (int i = 0; i < RELAY_COUNT; i++) {
    if (pinState[i] || (i < 2 && pcRebootStage[i])) {
      if (act.length()) act += " ";
      act += names[i];
    }
  }
  if (act.length()) s += "РЕЛЕ: " + act + "\n";
  uint64_t sec = (uint64_t)esp_timer_get_time() / 1000000ULL;
  s += "РАБОТА " + String((unsigned long)(sec / 86400ULL)) + "д " +
       String((unsigned long)((sec % 86400ULL) / 3600ULL)) + "ч";
  return s;
}

void configureModem() {
  sendAT("ATE0");               waitOK(1000);
  sendAT("AT+CLIP=1");          waitOK(1000);
  sendAT("AT+DDET=1,100,0,0"); waitOK(1000);
  sendAT("AT+DTAM=2");          waitOK(1000);
  sendAT("AT+CMGF=1");          waitOK(1000);
  sendAT("AT+CNMI=2,1,0,0,0");  waitOK(1000);
  sendAT("AT+CPMS=\"SM\",\"SM\",\"SM\""); waitOK(1000);
  sendAT("AT+CRSL=100");        waitOK(1000);
  sendAT("AT+CLVL=100");        waitOK(1000);
  sendAT("AT+CSMP=17,167,0,8");  waitOK(1000);
  cfgAppliedOnce = true;
  cfgReapplyPending = false;
}

void initSIM() {
  Serial.println(F("[SIM] Hardware Resetting via Pin 4..."));
  digitalWrite(SIM800_RST_PIN, LOW);
  safeDelay(200);
  digitalWrite(SIM800_RST_PIN, HIGH);
  safeDelay(5000);

  bool alive = false;
  for (int i = 0; i < 12; i++) {
    sim800.println("AT");
    if (waitOK(500)) { alive = true; break; }
    safeDelay(200);
  }

  Serial.println(F("[SIM] Waiting for network registration..."));
  bool registered = false;
  for (int i = 0; i < 45; i++) {
    if (queryCREG()) { registered = true; break; }
    safeDelay(1000);
  }

  configureModem();
  lastModemOK = millis();
  lastRegOK   = cregRegistered ? millis() : (millis() - WDG_REG_DEAD / 2);

  sendAT("AT+CSQ");
  waitOK(1000);
}

void runWatchdog() {
  unsigned long now = millis();
  if (otaBusy) return;

  if (!smsBusy) {
    if (now - lastPingT > WDG_PING_EVERY) {
      lastPingT = now;
      sendAT("AT");
    }
    if (now - lastCregT > WDG_CREG_EVERY) {
      lastCregT = now;
      sendAT("AT+CREG?");
    }
    // Диагностика: сначала CSQ, через 3 сек — SIM
    if (now - lastDiagT > DIAG_EVERY) {
      lastDiagT = now;
      sendAT("AT+CSQ");
      diagCpinAt = now + 3000;
      if (diagCpinAt == 0) diagCpinAt = 1;
    }
    if (diagCpinAt && (long)(now - diagCpinAt) >= 0) {
      diagCpinAt = 0;
      cpinAwait = true;
      cpinAskT = now;
      sendAT("AT+CPIN?");
    }
  }

  if (now - lastModemOK > WDG_UART_DEAD && !reinitPending) {
    Serial.println(F("[Watchdog] UART Freeze detected -> Scheduling hardware reinit"));
    reinitPending = true;
    return;
  }

  if (now - lastRegOK > WDG_REG_DEAD && !callActive && !cfunPending && !reinitPending && !smsBusy) {
    Serial.println(F("[Watchdog] Network Lost > 4m -> Scheduling CFUN restart"));
    cfunPending = true;
  }
}

void resetSIMNetwork() {
  sendAT("AT+CFUN=0");
  waitOK(3000);
  safeDelay(1000);
  sendAT("AT+CFUN=1");
  waitOK(3000);
  lastRegOK = millis() - WDG_REG_DEAD + 120000UL;
}

bool anyOutputBusy() {
  for (int i = 0; i < RELAY_COUNT; i++) if (pinState[i]) return true;
  if (pcRebootStage[0] || pcRebootStage[1]) return true;
  return false;
}

void checkPhases() {
  bool curA = (digitalRead(PHASE_A) == LOW);
  bool curB = (digitalRead(PHASE_B) == LOW);
  bool curC = (digitalRead(PHASE_C) == LOW);

  if (curA != lastReadA || curB != lastReadB || curC != lastReadC) {
    phaseDebounceTimer = millis();
    lastReadA = curA;
    lastReadB = curB;
    lastReadC = curC;
  }
  else if ((millis() - phaseDebounceTimer) > PHASE_DEBOUNCE) {
    if (curA != stablePhaseA || curB != stablePhaseB || curC != stablePhaseC) {
      stablePhaseA = curA;
      stablePhaseB = curB;
      stablePhaseC = curC;

      if (callActive || anyOutputBusy() || smsBusy) {
        phaseSMSPending = true;
      } else {
        sendSMSToAll(buildPhaseStatus());
      }
    }
  }
}

void tryFlushPendingSMS() {
  if (phaseSMSPending && !callActive && !anyOutputBusy() && !smsBusy) {
    phaseSMSPending = false;
    sendSMSToAll(buildPhaseStatus());
  }
}

String buildPhaseStatus() {
  // Коротко, чтобы помещалось в одну SMS (<= 70 символов)
  bool allOk = stablePhaseA && stablePhaseB && stablePhaseC;
  String smsBody = deviceAddress + "\n";
  smsBody += allOk ? "ФАЗЫ ОК\n" : "АВАРИЯ ФАЗ!\n";
  smsBody += "A:" + String(stablePhaseA ? "ОК" : "НЕТ");
  smsBody += " B:" + String(stablePhaseB ? "ОК" : "НЕТ");
  smsBody += " C:" + String(stablePhaseC ? "ОК" : "НЕТ");
  return smsBody;
}

bool isNumberAllowed(String incoming) {
  int q1 = incoming.indexOf('"');
  int q2 = incoming.indexOf('"', q1 + 1);
  if (q1 < 0 || q2 < 0) return false;

  String raw = incoming.substring(q1 + 1, q2);
  raw = maybeUcs2HexNumber(raw); // Декодируем, только если это действительно UCS2 HEX
  String clean = "";
  for (unsigned int i = 0; i < raw.length(); i++) {
    if (isDigit(raw.charAt(i))) clean += raw.charAt(i);
  }
  if (clean.length() == 11 && clean.startsWith("8")) {
    clean = "7" + clean.substring(1);
  }

  return isPhoneAllowed(clean);
}

// =====================================================
// ГОЛОСОВОЕ МЕНЮ
//  Главное меню (0001.amr):
//    1 - Экран 01 (подменю, 0002.amr)
//    2 - Экран 02 (подменю, 0003.amr)
//  Подменю экрана:
//    1 - отключить сеть 380 В на 60 секунд
//    2 - включить или выключить компьютер (короткий импульс Power)
//    3 - перезагрузить компьютер (длинный импульс -> пауза -> короткий импульс)
//    0 - возврат в главное меню
//  * - завершить звонок (в любом меню)
// =====================================================

// screen: 0 = Экран 01, 1 = Экран 02
// Реле ПК: индекс screen (0/1), реле сети экрана: индекс screen + 2 (2/3)
bool pcBusy(int screen) {
  return pinState[screen] || pcRebootStage[screen] != 0;
}

// ----- Действия с реле (общие для голосового меню и SMS) -----
// Возвращают false, если для этого ПК/экрана уже выполняется команда
bool pcPower(int screen) {                          // короткий импульс: вкл/выкл
  if (pcBusy(screen)) return false;
  pulseOutput(screen, PULSE_PC_SHORT);
  return true;
}

bool pcReboot(int screen) {                         // длинный -> пауза -> короткий
  if (pcBusy(screen)) return false;
  pcRebootStage[screen] = 1;
  pulseOutput(screen, pulsePcLong);
  return true;
}

bool pcForceOff(int screen) {                       // только длинный импульс
  if (pcBusy(screen)) return false;
  pulseOutput(screen, pulsePcLong);
  return true;
}

bool screenPowerCycle(int screen) {                 // сеть экрана выкл на 60 сек
  int r = screen + 2;
  if (pinState[r]) return false;
  pulseOutput(r, TIMEOUT_SCREEN_PWR);
  return true;
}

void handleScreenMenu(int screen, char d) {
  if (d == '1') {                                   // сеть 380 В — выкл на 60 сек
    screenPowerCycle(screen);
    playAmr(4);
    return;
  }
  if (d == '2') {                                   // вкл/выкл компьютер
    if (pcPower(screen)) playAmr(4);
    return;
  }
  if (d == '3') {                                   // перезагрузка компьютера
    if (pcReboot(screen)) playAmr(4);
    return;
  }
  if (d == '0') { menu = ROOT_MENU; playAmr(1); }
}

void handleDTMF(char d) {
  if (!callActive) return;
  if (millis() - lastDTMF < DTMF_GUARD) return;
  lastDTMF = millis();

  if (d == '*') {
    sendAT("ATH");
    callActive = false;
    stopAmr();
    return;
  }

  if (menu == ROOT_MENU) {
    if (d == '1') { menu = MENU_A; playAmr(2); return; }

#if SCREEN_COUNT >= 2
    if (d == '2') { menu = MENU_B; playAmr(3); return; }
#else
    if (d == '2') { playAmr(6); return; }
#endif
    return;
  }

  if (menu == MENU_A) { handleScreenMenu(0, d); return; }

#if SCREEN_COUNT >= 2
  if (menu == MENU_B) { handleScreenMenu(1, d); return; }
#endif
}

void pulseOutput(int index, unsigned long duration) {
  relayOn(index);
  pinState[index]    = true;
  pinTimer[index]    = millis();
  pinDuration[index] = duration;
}

void outputHandler() {
  for (int i = 0; i < RELAY_COUNT; i++) {
    if (!pinState[i]) continue;
    if (millis() - pinTimer[i] > pinDuration[i]) {
      relayOff(i);
      pinState[i] = false;
      pinDuration[i] = 0;
      // Перезагрузка ПК: длинный импульс закончился -> ждём паузу
      if (i < 2 && pcRebootStage[i] == 1) {
        pcRebootStage[i] = 2;
        pcRebootTimer[i] = millis();
      }
    }
  }
  // Перезагрузка ПК: после паузы — короткий импульс (включение)
  for (int i = 0; i < 2; i++) {
    if (pcRebootStage[i] == 2 && millis() - pcRebootTimer[i] > PC_REBOOT_PAUSE) {
      pcRebootStage[i] = 0;
      pulseOutput(i, PULSE_PC_SHORT);
    }
  }
}

void checkTimeout() {
  if (callActive && millis() - callStart > CALL_TIMEOUT) {
    sendAT("ATH");
    callActive = false;
    pendingGreeting = false;
    stopAmr();
  }
}

void sendAT(const char* cmd) {
  sim800.println(cmd);
}

void stopAmr() {
  sendAT("AT+CMEDPLAY=0");
}

void playAmr(int fileNum) {
  stopAmr();
  safeDelay(100);
  String cmd = "AT+CMEDPLAY=1,\"C:\\User\\000" + String(fileNum) + ".amr\",0,100";
  sendAT(cmd.c_str());
}

String extractQuoted(const String& s, int nth) {
  int from = 0;
  for (int n = 0; n <= nth; n++) {
    int q1 = s.indexOf('"', from);
    if (q1 < 0) return "";
    int q2 = s.indexOf('"', q1 + 1);
    if (q2 < 0) return "";
    if (n == nth) return s.substring(q1 + 1, q2);
    from = q2 + 1;
  }
  return "";
}

String digitsOnly(const String& raw) {
  String clean = "";
  for (unsigned int i = 0; i < raw.length(); i++) {
    if (isDigit(raw.charAt(i))) clean += raw.charAt(i);
  }
  if (clean.length() == 11 && clean.startsWith("8")) {
    clean = "7" + clean.substring(1);
  }
  return clean;
}

bool isPhoneAllowed(const String& digits) {
  for (int i = 0; i < callCount; i++) {
    if (digits == callList[i]) return true;
  }
  return false;
}

String toSmsNumber(const String& digits) {
  if (digits.length() == 0) return "";
  if (digits.charAt(0) == '+') return digits;
  if (digits.length() == 11 && digits.startsWith("7")) return "+" + digits;
  return "+" + digits;
}

String maybeUcs2HexToText(const String& s) {
  String t = s;
  t.trim();
  if (t.length() < 8 || (t.length() % 4) != 0) return s;
  for (unsigned int i = 0; i < t.length(); i++) {
    char c = t.charAt(i);
    bool hex = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    if (!hex) return s;
  }
  String out = "";
  for (unsigned int i = 0; i + 3 < t.length(); i += 4) {
    int cp = (int)strtol(t.substring(i, i + 4).c_str(), nullptr, 16);
    if (cp == 0) continue;
    if (cp < 128) {
      out += (char)cp;
    } else if (cp < 0x800) {
      out += (char)(0xC0 | (cp >> 6));
      out += (char)(0x80 | (cp & 0x3F));
    } else {
      out += (char)(0xE0 | (cp >> 12));
      out += (char)(0x80 | ((cp >> 6) & 0x3F));
      out += (char)(0x80 | (cp & 0x3F));
    }
  }
  return out.length() ? out : s;
}

// Для номеров: UCS2 HEX номера всегда выглядит как группы "00xx" (002B0037...).
// Обычный номер из цифр (например, 12-значный) так не выглядит и не декодируется.
String maybeUcs2HexNumber(const String& s) {
  String t = s;
  t.trim();
  if (t.length() < 8 || (t.length() % 4) != 0) return s;
  for (unsigned int i = 0; i < t.length(); i += 4) {
    if (t.charAt(i) != '0' || t.charAt(i + 1) != '0') return s;
  }
  return maybeUcs2HexToText(t);
}

bool waitModemToken(const char* token, uint32_t tmo) {
  unsigned long t = millis();
  String acc;
  while (millis() - t < tmo) {
    serviceBackground();
    while (sim800.available()) {
      char c = (char)sim800.read();
      if (c == '\r') continue;
      if (c == '\n') {
        acc.trim();
        if (acc.length() > 0) {
          Serial.println(acc);
          if (acc.indexOf(token) >= 0) return true;
          if (acc.indexOf("ERROR") >= 0) return false;
          if (acc.indexOf("FAIL") >= 0) return false;
        }
        acc = "";
      } else {
        acc += c;
      }
    }
  }
  return false;
}

bool readExactly(uint8_t* buf, int n, uint32_t tmo) {
  int got = 0;
  unsigned long t = millis();
  while (got < n && (millis() - t) < tmo) {
    serviceBackground();
    if (sim800.available()) {
      buf[got++] = (uint8_t)sim800.read();
    }
  }
  return got == n;
}

bool parseHttpUrl(const String& url, String& host, int& port, String& path) {
  String u = url;
  u.trim();
  if (u.startsWith("https://") || u.startsWith("HTTPS://")) return false;
  if (u.startsWith("HTTP://")) u = "http://" + u.substring(7);
  if (!u.startsWith("http://")) return false;
  u = u.substring(7);
  int slash = u.indexOf('/');
  String hp;
  if (slash < 0) {
    hp = u;
    path = "/";
  } else {
    hp = u.substring(0, slash);
    path = u.substring(slash);
  }
  int colon = hp.indexOf(':');
  if (colon >= 0) {
    host = hp.substring(0, colon);
    port = hp.substring(colon + 1).toInt();
    if (port <= 0) port = 80;
  } else {
    host = hp;
    port = 80;
  }
  return host.length() > 0;
}

bool gprsAttach() {
  sendAT("AT+CIPSHUT");
  waitModemToken("SHUT OK", 8000);

  sendAT("AT+CIPMUX=0");
  if (!waitOK(2000)) return false;

  sendAT("AT+CIPRXGET=1");
  if (!waitOK(2000)) return false;

  sendAT("AT+CGATT=1");
  waitOK(8000);

  String cstt = String("AT+CSTT=\"") + GPRS_APN + "\",\"" + GPRS_USER + "\",\"" + GPRS_PASS + "\"";
  sendAT(cstt.c_str());
  waitOK(5000);

  sendAT("AT+CIICR");
  if (!waitOK(20000)) {
    Serial.println(F("[OTA] CIICR failed"));
    return false;
  }

  sim800.println("AT+CIFSR");
  unsigned long t = millis();
  bool gotIp = false;
  String acc;
  while (millis() - t < 5000) {
    serviceBackground();
    while (sim800.available()) {
      char c = (char)sim800.read();
      if (c == '\r') continue;
      if (c == '\n') {
        acc.trim();
        if (acc.length() > 0) {
          Serial.println(acc);
          if (acc.indexOf('.') > 0 && acc.indexOf("ERROR") < 0) gotIp = true;
        }
        acc = "";
      } else {
        acc += c;
      }
    }
    if (gotIp) break;
  }
  return gotIp;
}

void gprsShutdown() {
  sendAT("AT+CIPCLOSE");
  waitModemToken("CLOSE OK", 3000);
  sendAT("AT+CIPSHUT");
  waitModemToken("SHUT OK", 5000);
}

bool tcpOpen(const String& host, int port) {
  String cmd = "AT+CIPSTART=\"TCP\",\"" + host + "\",\"" + String(port) + "\"";
  sendAT(cmd.c_str());
  unsigned long t = millis();
  String acc;
  while (millis() - t < 40000) {
    serviceBackground();
    while (sim800.available()) {
      char c = (char)sim800.read();
      if (c == '\r') continue;
      if (c == '\n') {
        acc.trim();
        if (acc.length() > 0) {
          Serial.println(acc);
          if (acc.indexOf("CONNECT OK") >= 0) return true;
          if (acc.indexOf("ALREADY CONNECT") >= 0) return true;
          if (acc.indexOf("CONNECT FAIL") >= 0) return false;
          if (acc.indexOf("ERROR") >= 0) return false;
        }
        acc = "";
      } else {
        acc += c;
      }
    }
  }
  return false;
}

int cipRead(uint8_t* buf, int maxN, int& remaining) {
  remaining = 0;
  sim800.print(F("AT+CIPRXGET=2,"));
  sim800.println(maxN);

  unsigned long t = millis();
  String acc;
  int actual = -1;
  while (millis() - t < 8000) {
    serviceBackground();
    while (sim800.available()) {
      char c = (char)sim800.read();
      if (c == '\r') continue;
      if (c == '\n') {
        acc.trim();
        if (acc.startsWith("+CIPRXGET: 2,")) {
          int c1 = acc.indexOf(',');
          int c2 = acc.indexOf(',', c1 + 1);
          if (c1 >= 0) {
            if (c2 > c1) {
              actual = acc.substring(c1 + 1, c2).toInt();
              remaining = acc.substring(c2 + 1).toInt();
            } else {
              actual = acc.substring(c1 + 1).toInt();
              remaining = 0;
            }
          }
          goto gotHeader;
        }
        if (acc.indexOf("ERROR") >= 0) return -1;
        if (acc.indexOf("CLOSED") >= 0) return -2;
        acc = "";
      } else {
        acc += c;
      }
    }
  }
  return -1;

gotHeader:
  if (actual <= 0) {
    waitOK(1500);
    return 0;
  }
  if (actual > maxN) actual = maxN;
  if (!readExactly(buf, actual, 8000)) return -1;
  waitOK(2000);
  return actual;
}

void normalizePhone(char* dst, const char* src) {
  char tmp[16];
  int n = 0;
  for (int i = 0; src[i] && n < 15; i++) {
    if (src[i] >= '0' && src[i] <= '9') tmp[n++] = src[i];
  }
  tmp[n] = 0;
  if (n == 11 && tmp[0] == '8') tmp[0] = '7';
  strncpy(dst, tmp, 15);
  dst[15] = 0;
}

int findPhone(char list[][16], int count, const char* digits) {
  for (int i = 0; i < count; i++) {
    if (strcmp(list[i], digits) == 0) return i;
  }
  return -1;
}

bool phonesSave() {
  if (!phonePrefs.begin("phones", false)) return false;
  phonePrefs.putUChar("smsN", (uint8_t)smsCount);
  phonePrefs.putUChar("callN", (uint8_t)callCount);
  phonePrefs.putBytes("sms", smsList, sizeof(smsList));
  phonePrefs.putBytes("call", callList, sizeof(callList));
  phonePrefs.putUChar("hbN", (uint8_t)hbCount);
  phonePrefs.putBytes("hb", hbList, sizeof(hbList));
  phonePrefs.putUChar("init", 1);
  phonePrefs.end();
  return true;
}

void phonesSeedFactory() {
  smsCount = 0;
  callCount = 0;
  memset(smsList, 0, sizeof(smsList));
  memset(callList, 0, sizeof(callList));
  for (int i = 0; i < TARGET_COUNT && smsCount < PHONE_MAX; i++) {
    normalizePhone(smsList[smsCount], TARGET_PHONES[i]);
    if (smsList[smsCount][0]) smsCount++;
  }
  for (int i = 0; i < ALLOWED_COUNT && callCount < PHONE_MAX; i++) {
    normalizePhone(callList[callCount], ALLOWED[i]);
    if (callList[callCount][0]) callCount++;
  }
}

void phonesLoad() {
  smsCount = 0;
  callCount = 0;
  hbCount = 0;
  memset(smsList, 0, sizeof(smsList));
  memset(callList, 0, sizeof(callList));
  memset(hbList, 0, sizeof(hbList));
  if (!phonePrefs.begin("phones", true)) {
    phonesSeedFactory();
    phonesSave();
    return;
  }
  uint8_t inited = phonePrefs.getUChar("init", 0);
  if (!inited) {
    phonePrefs.end();
    phonesSeedFactory();
    phonesSave();
    return;
  }
  smsCount = phonePrefs.getUChar("smsN", 0);
  callCount = phonePrefs.getUChar("callN", 0);
  if (smsCount > PHONE_MAX) smsCount = PHONE_MAX;
  if (callCount > PHONE_MAX) callCount = PHONE_MAX;
  phonePrefs.getBytes("sms", smsList, sizeof(smsList));
  phonePrefs.getBytes("call", callList, sizeof(callList));
  hbCount = phonePrefs.getUChar("hbN", 0);
  if (hbCount > PHONE_MAX) hbCount = PHONE_MAX;
  if (hbCount > 0) phonePrefs.getBytes("hb", hbList, sizeof(hbList));
  phonePrefs.end();
  if (smsCount == 0 && callCount == 0) {
    phonesSeedFactory();
    phonesSave();
  }
  Serial.print(F("[PHONES] SMS="));
  Serial.print(smsCount);
  Serial.print(F(" CALL="));
  Serial.print(callCount);
  Serial.print(F(" HB="));
  Serial.println(hbCount);
}

bool phonesAdd(char list[][16], int& count, const char* digits) {
  if (!digits || !digits[0]) return false;
  if (findPhone(list, count, digits) >= 0) return true;
  if (count >= PHONE_MAX) return false;
  strncpy(list[count], digits, 15);
  list[count][15] = 0;
  count++;
  return phonesSave();
}

bool phonesDel(char list[][16], int& count, const char* digits, bool keepLast) {
  int idx = findPhone(list, count, digits);
  if (idx < 0) return false;
  if (keepLast && count <= 1) return false;
  for (int i = idx; i < count - 1; i++) {
    strncpy(list[i], list[i + 1], 16);
  }
  memset(list[count - 1], 0, 16);
  count--;
  return phonesSave();
}

String listPhones(char list[][16], int count, bool plus) {
  if (count == 0) return "(пусто)";
  String s;
  for (int i = 0; i < count; i++) {
    if (i) s += "\n";
    if (plus) s += "+";
    s += list[i];
  }
  return s;
}

void cfgLoad() {
  pulsePcLong = PULSE_PC_LONG_DEF;
  deviceAddress = DEVICE_ADDRESS;
  hbEnabled = HB_DEFAULT_ON;
  if (!cfgPrefs.begin("cfg", true)) {
    Serial.println(F("[CFG] NVS empty, defaults"));
    return;
  }
  unsigned long v = cfgPrefs.getULong("longMs", PULSE_PC_LONG_DEF);
  String a = cfgPrefs.getString("addr", DEVICE_ADDRESS);
  hbEnabled = cfgPrefs.getUChar("hbOn", HB_DEFAULT_ON) != 0;
  cfgPrefs.end();
  a.trim();
  if (a.length() > 0) deviceAddress = a;
  Serial.print(F("[CFG] addr="));
  Serial.println(deviceAddress);
  if (v < PULSE_PC_LONG_MIN) v = PULSE_PC_LONG_MIN;
  if (v > PULSE_PC_LONG_MAX) v = PULSE_PC_LONG_MAX;
  pulsePcLong = v;
  Serial.print(F("[CFG] pulsePcLong="));
  Serial.println(pulsePcLong);
}

bool cfgSave() {
  if (!cfgPrefs.begin("cfg", false)) return false;
  cfgPrefs.putULong("longMs", pulsePcLong);
  cfgPrefs.putString("addr", deviceAddress);
  cfgPrefs.putUChar("hbOn", hbEnabled ? 1 : 0);
  cfgPrefs.end();
  return true;
}

// Количество символов в UTF-8 строке (кириллица = 1 символ)
int utf8Chars(const String& s) {
  int n = 0;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (((uint8_t)s[i] & 0xC0) != 0x80) n++;
  }
  return n;
}

String handleCfgCommand(const String& upper, const String& cmdLine) {
  // ----- Адрес устройства -----
  if (upper == "GET ADDR" || upper == "ADDR") {
    return "АДРЕС: " + deviceAddress;
  }

  if (upper == "RESET ADDR") {
    deviceAddress = DEVICE_ADDRESS;
    if (!cfgSave()) return "Ошибка записи NVS";
    return "Адрес сброшен: " + deviceAddress;
  }

  if (upper.startsWith("SET ADDR")) {
    String a = cmdLine.substring(8);   // регистр текста сохраняется как есть
    a.trim();
    if (a.length() == 0) return "SET ADDR <адрес>, например SET ADDR Ленина 5";
    if (utf8Chars(a) > ADDR_MAX_CHARS) {
      return "Адрес слишком длинный (макс " + String(ADDR_MAX_CHARS) + " символов)";
    }
    deviceAddress = a;
    if (!cfgSave()) return "Ошибка записи NVS";
    return "АДРЕС сохранён: " + deviceAddress;
  }

  if (upper == "GET LONG" || upper == "LONG") {
    return "LONG=" + String(pulsePcLong) + " мс\n(короткий=" + String(PULSE_PC_SHORT) + " мс)";
  }

  if (upper == "RESET LONG") {
    pulsePcLong = PULSE_PC_LONG_DEF;
    cfgSave();
    return "LONG сброшен: " + String(pulsePcLong) + " мс";
  }

  if (upper.startsWith("SET LONG")) {
    int p = upper.indexOf("LONG");
    String numStr = "";
    if (p >= 0) {
      numStr = cmdLine.substring(p + 4);
      numStr.trim();
    }
    if (numStr.length() == 0) return "SET LONG <мс>, например SET LONG 5000";
    long v = numStr.toInt();
    if (v < (long)PULSE_PC_LONG_MIN || v > (long)PULSE_PC_LONG_MAX) {
      return "Диапазон " + String(PULSE_PC_LONG_MIN) + ".." + String(PULSE_PC_LONG_MAX) + " мс";
    }
    pulsePcLong = (unsigned long)v;
    if (!cfgSave()) return "Ошибка записи NVS";
    return "LONG=" + String(pulsePcLong) + " мс (сохранено)";
  }

  return "";
}

// =====================================================
// SMS: УПРАВЛЕНИЕ РЕЛЕ
//   PC1 POWER  / PC2 POWER   — вкл/выкл ПК (короткое нажатие)
//   PC1 REBOOT / PC2 REBOOT  — перезагрузка ПК (длинное -> пауза -> короткое)
//   PC1 OFF    / PC2 OFF     — принудительно выключить ПК (длинное нажатие)
//   SCR1 RESET / SCR2 RESET  — отключить сеть экрана на 60 сек
// =====================================================
String handleRelayCommand(const String& upper) {
  int screen = -1;
  bool isPc = false;
  String act;
  if      (upper.startsWith("PC1 "))   { screen = 0; isPc = true;  act = upper.substring(4); }
  else if (upper.startsWith("PC2 "))   { screen = 1; isPc = true;  act = upper.substring(4); }
  else if (upper.startsWith("SCR1 "))  { screen = 0; isPc = false; act = upper.substring(5); }
  else if (upper.startsWith("SCR2 "))  { screen = 1; isPc = false; act = upper.substring(5); }
  else return "";
  act.trim();

  if (screen >= SCREEN_COUNT) return "Экран " + String(screen + 1) + " не используется";

  String n = String(screen + 1);
  const char* busyMsg = "уже выполняется команда, подождите";

  if (isPc) {
    if (act == "POWER") return pcPower(screen)    ? "ПК" + n + ": нажата Power (вкл/выкл)" : "ПК" + n + ": " + busyMsg;
    if (act == "REBOOT")return pcReboot(screen)   ? "ПК" + n + ": перезагрузка запущена"   : "ПК" + n + ": " + busyMsg;
    if (act == "OFF")   return pcForceOff(screen) ? "ПК" + n + ": принудительное выключение" : "ПК" + n + ": " + busyMsg;
    return "PC" + n + " POWER / REBOOT / OFF";
  }
  if (act == "RESET") {
    return screenPowerCycle(screen) ? "Экран " + n + ": сеть отключена на 60 сек"
                                    : "Экран " + n + ": " + busyMsg;
  }
  return "SCR" + n + " RESET";
}

// =====================================================
// SMS: "НА СВЯЗИ" (HB)
//   HB ON / HB OFF        — включить / выключить
//   HB ADD +79... / HB DEL +79...
//   HB LIST               — состояние и список номеров
//   HB NOW                — отправить прямо сейчас
// =====================================================
String handleHbCommand(const String& upper, const String& cmdLine) {
  if (!(upper == "HB" || upper.startsWith("HB "))) return "";
  String sub = upper.length() > 2 ? upper.substring(3) : String("LIST");
  sub.trim();

  if (sub == "ON") {
    hbEnabled = true;
    lastHbT = millis();
    if (!cfgSave()) return "Ошибка записи NVS";
    return "НА СВЯЗИ: включено, раз в " + String(HB_INTERVAL_DAYS) + " дн";
  }
  if (sub == "OFF") {
    hbEnabled = false;
    if (!cfgSave()) return "Ошибка записи NVS";
    return "НА СВЯЗИ: выключено";
  }
  if (sub == "NOW") {
    lastHbT = millis();
    outboxAdd(buildStatus("НА СВЯЗИ"), 1);
    return "";   // ответ — само сообщение "НА СВЯЗИ"
  }
  if (sub == "LIST" || sub.length() == 0) {
    String out = "НА СВЯЗИ: " + String(hbEnabled ? "ВКЛ" : "ВЫКЛ");
    if (hbEnabled) {
      unsigned long left = HB_INTERVAL_MS - (millis() - lastHbT);
      unsigned long hLeft = left / 3600000UL;
      out += "\nследующая через " + String(hLeft / 24) + "д " + String(hLeft % 24) + "ч";
    }
    out += "\nНомера (" + String(hbCount) + "):\n";
    out += hbCount ? listPhones(hbList, hbCount, true) : String("(пусто - шлём на список SMS)");
    return out;
  }
  if (sub.startsWith("ADD ") || sub.startsWith("DEL ")) {
    String numRaw = sub.substring(4);   // в номере только цифры, регистр не важен
    numRaw.trim();
    char digits[16];
    normalizePhone(digits, numRaw.c_str());
    if (strlen(digits) < 10) return "Некорректный номер";
    if (sub.startsWith("ADD ")) {
      if (!phonesAdd(hbList, hbCount, digits)) return "Не добавлен (лимит " + String(PHONE_MAX) + ")";
      return String("Добавлен в НА СВЯЗИ: +") + digits;
    }
    if (!phonesDel(hbList, hbCount, digits, false)) return "Номер не найден в НА СВЯЗИ";
    return String("Удалён из НА СВЯЗИ: +") + digits;
  }
  return "HB ON / OFF / NOW / LIST\nHB ADD +79..\nHB DEL +79..";
}

String handlePhoneCommand(const String& upper, const String& cmdLine) {
  if (upper == "HELP") {
    return String("Формат: ") + (strlen(CMD_PIN) ? "PIN " : "") + "КОМАНДА\n"
           "ADD/DEL SMS +79..\nADD/DEL CALL +79..\nLIST\nRESET PHONES\n"
           "GET LONG\nSET LONG 5000\nRESET LONG\n"
           "GET ADDR\nSET ADDR текст\nRESET ADDR\n"
           "STATUS\nPC1 POWER/REBOOT/OFF\nPC2 POWER/REBOOT/OFF\nSCR1 RESET\nSCR2 RESET\n"
           "HB ON/OFF/NOW/LIST\nHB ADD/DEL +79..\nVER\nOTA http://..";
  }

  if (upper == "LIST" || upper == "LIST SMS" || upper == "LIST CALL") {
    String out;
    if (upper != "LIST CALL") {
      out += "SMS (" + String(smsCount) + "):\n" + listPhones(smsList, smsCount, true);
    }
    if (upper == "LIST") out += "\f";   // списки SMS и CALL — отдельными SMS
    if (upper != "LIST SMS") {
      out += "CALL (" + String(callCount) + "):\n" + listPhones(callList, callCount, true);
    }
    return out;
  }

  if (upper == "RESET PHONES") {
    phonesSeedFactory();
    phonesSave();
    return "Списки сброшены к заводским\nSMS:" + String(smsCount) + " CALL:" + String(callCount);
  }

  int sp = cmdLine.indexOf(' ');
  if (sp < 0) return "";
  String rest = cmdLine.substring(sp + 1);
  rest.trim();
  int sp2 = rest.indexOf(' ');
  String which = rest;
  String numRaw = "";
  if (sp2 >= 0) {
    which = rest.substring(0, sp2);
    numRaw = rest.substring(sp2 + 1);
    numRaw.trim();
  }
  which.toUpperCase();

  char digits[16];
  normalizePhone(digits, numRaw.c_str());
  bool isSms = (which == "SMS");
  bool isCall = (which == "CALL" || which == "WL");
  if (!isSms && !isCall) return "";
  if (upper.startsWith("ADD ") || upper.startsWith("DEL ")) {
    if (strlen(digits) < 10) return "Некорректный номер";
  }

  if (upper.startsWith("ADD ")) {
    bool ok = isSms
      ? phonesAdd(smsList, smsCount, digits)
      : phonesAdd(callList, callCount, digits);
    if (!ok) return "Не добавлен (лимит " + String(PHONE_MAX) + ")";
    return String("Добавлен в ") + (isSms ? "SMS" : "CALL") + ": +" + digits;
  }

  if (upper.startsWith("DEL ")) {
    if (isCall) {
      if (callCount <= 1) return "Нельзя удалить последний CALL";
      if (!phonesDel(callList, callCount, digits, true)) return "Номер не найден в CALL";
      return String("Удалён из CALL: +") + digits;
    }
    if (smsCount <= 1) return "Нельзя удалить последний SMS";
    if (!phonesDel(smsList, smsCount, digits, true)) return "Номер не найден в SMS";
    return String("Удалён из SMS: +") + digits;
  }

  return "";
}

// =====================================================
// ОБРАБОТКА ВХОДЯЩИХ SMS (С ИСПРАВЛЕНИЕМ UCS2/HEX В СЕНДЕРЕ)
// =====================================================
void processIncomingSms(int index) {
  if (index < 0) return;
  smsBusy = true;

  sim800.print(F("AT+CMGR="));
  sim800.println(index);

  String sender;
  String body;
  bool gotHeader = false;
  bool done = false;
  unsigned long t = millis();
  String acc;

  while (!done && (millis() - t < 8000)) {
    serviceBackground();
    while (sim800.available()) {
      char c = (char)sim800.read();
      if (c == '\r') continue;
      if (c == '\n') {
        acc.trim();
        if (acc.length() == 0) {
          acc = "";
          continue;
        }
        Serial.println(acc);
        if (!gotHeader && acc.startsWith("+CMGR:")) {
          gotHeader = true;
          sender = extractQuoted(acc, 1);
          sender = maybeUcs2HexNumber(sender); // Декодируем номер отправителя, если он в UCS2 HEX
          acc = "";
          continue;
        }
        if (gotHeader && (acc == "OK" || acc == "ERROR")) {
          done = true;
          acc = "";
          break;
        }
        if (gotHeader) {
          if (body.length()) body += "\n";
          body += acc;
        } else {
          handleLine(acc);
        }
        acc = "";
      } else {
        acc += c;
      }
    }
  }

  sim800.print(F("AT+CMGD="));
  sim800.println(index);
  waitOK(2000);
  smsBusy = false;

  if (!gotHeader) return;

  String digits = digitsOnly(sender);
  if (!isPhoneAllowed(digits)) {
    Serial.println(F("[SMS] Command ignored: sender not allowed"));
    return;
  }

  body = maybeUcs2HexToText(body);
  body.trim();
  int nl = body.indexOf('\n');
  String cmdLine = (nl >= 0) ? body.substring(0, nl) : body;
  cmdLine.trim();

  // Проверка PIN: "1234 КОМАНДА"
  if (strlen(CMD_PIN) > 0) {
    String pin = String(CMD_PIN);
    if (!cmdLine.startsWith(pin + " ")) {
      Serial.println(F("[SMS] Command ignored: wrong or missing PIN"));
      return;
    }
    cmdLine = cmdLine.substring(pin.length() + 1);
    cmdLine.trim();
  }

  String upper = cmdLine;
  upper.toUpperCase();

  String replyTo = toSmsNumber(digits);

  if (upper == "VER" || upper == "VERSION") {
    sendSMS(replyTo, String("ВЕРСИЯ: ") + FW_VERSION + "\n" + deviceAddress +
            "\nLONG=" + String(pulsePcLong) + " мс" +
            "\nРАБОТАЕТ: " + uptimeStr() +
            "\nСБРОС: " + resetReasonStr(bootReason) +
            "\nRAM: " + String(ESP.getFreeHeap() / 1024) + " КБ (мин " +
            String(ESP.getMinFreeHeap() / 1024) + ")");
    return;
  }

  if (upper == "STATUS" || upper == "STATE") {
    sendSMS(replyTo, buildStatus());
    return;
  }

  {
    String relayReply = handleRelayCommand(upper);
    if (relayReply.length()) {
      sendSMS(replyTo, relayReply);
      return;
    }
  }

  if (upper == "HB" || upper.startsWith("HB ")) {
    String hbReply = handleHbCommand(upper, cmdLine);
    if (hbReply.length()) sendSMS(replyTo, hbReply);
    return;
  }

  {
    String cfgReply = handleCfgCommand(upper, cmdLine);
    if (cfgReply.length()) {
      sendSMS(replyTo, cfgReply);
      return;
    }
  }

  {
    String phoneReply = handlePhoneCommand(upper, cmdLine);
    if (phoneReply.length()) {
      sendSMS(replyTo, phoneReply);
      return;
    }
  }

  if (upper.startsWith("OTA")) {
    int sp = cmdLine.indexOf(' ');
    if (sp < 0) {
      sendSMS(replyTo, "OTA: укажите URL, например OTA http://host/fw.bin");
      return;
    }
    String args = cmdLine.substring(sp + 1);
    args.trim();
    String url = args;
    String md5 = "";
    int sp2 = args.indexOf(' ');
    if (sp2 > 0) {
      url = args.substring(0, sp2);
      md5 = args.substring(sp2 + 1);
      md5.trim();
      md5.toLowerCase();
    }
    bool md5Ok = (md5.length() == 32);
    for (unsigned int i = 0; md5Ok && i < md5.length(); i++) {
      if (!isHexadecimalDigit(md5.charAt(i))) md5Ok = false;
    }
    if (md5.length() > 0 && !md5Ok) {
      sendSMS(replyTo, "OTA: неверный MD5 (нужно 32 hex-символа)");
      return;
    }
    if (OTA_REQUIRE_MD5 && !md5Ok) {
      sendSMS(replyTo, "OTA: нужен MD5\nOTA http://host/fw.bin <md5>");
      return;
    }
    String host, path;
    int port = 80;
    if (!parseHttpUrl(url, host, port, path)) {
      sendSMS(replyTo, "OTA: нужен HTTP URL (не HTTPS)");
      return;
    }
    otaUrl = url;
    otaMd5 = md5Ok ? md5 : "";
    otaRequester = replyTo;
    otaPending = true;
    sendSMS(replyTo, "OTA: начинаю загрузку\n" + url);
    return;
  }
}

void performOta() {
  if (otaUrl.length() == 0) return;

  String host, path;
  int port = 80;
  if (!parseHttpUrl(otaUrl, host, port, path)) {
    sendSMS(otaRequester, "OTA: некорректный URL");
    return;
  }

  otaBusy = true;
  Serial.print(F("[OTA] GET "));
  Serial.print(host);
  Serial.print(':');
  Serial.print(port);
  Serial.println(path);

  String result = "OTA: ошибка";
  bool ok = false;
  uint8_t chunk[OTA_CHUNK_SIZE];
  size_t bodyGot = 0;
  int contentLength = -1;
  int httpCode = 0;
  bool headerDone = false;
  String header;
  unsigned long started = millis();
  int emptyReads = 0;

  if (!gprsAttach()) {
    result = "OTA: нет GPRS (проверьте APN)";
    goto ota_done;
  }
  if (!tcpOpen(host, port)) {
    result = "OTA: не удалось открыть TCP";
    goto ota_done;
  }

  sendAT("AT+CIPSEND");
  if (!waitPrompt(8000)) {
    result = "OTA: нет приглашения CIPSEND";
    goto ota_done;
  }

  {
    String req = "GET " + path + " HTTP/1.0\r\nHost: " + host + "\r\nConnection: close\r\nUser-Agent: ESP32-SIM800-OTA\r\n\r\n";
    sim800.print(req);
    sim800.write(26);
  }
  if (!waitModemToken("SEND OK", 20000)) {
    result = "OTA: HTTP-запрос не отправлен";
    goto ota_done;
  }

  while (!headerDone) {
    if (millis() - started > OTA_TOTAL_TMO) {
      result = "OTA: таймаут заголовка";
      goto ota_done;
    }
    int remaining = 0;
    int n = cipRead(chunk, OTA_CHUNK_SIZE, remaining);
    if (n == -2) {
      result = "OTA: TCP закрыт до заголовка";
      goto ota_done;
    }
    if (n < 0) {
      result = "OTA: ошибка чтения заголовка";
      goto ota_done;
    }
    if (n == 0) {
      emptyReads++;
      if (emptyReads > 80) {
        result = "OTA: пустой ответ сервера";
        goto ota_done;
      }
      safeDelay(200);
      yield();
      continue;
    }
    emptyReads = 0;
    for (int i = 0; i < n; i++) header += (char)chunk[i];
    int pos = header.indexOf("\r\n\r\n");
    if (pos < 0) {
      if (header.length() > OTA_HDR_MAX) {
        result = "OTA: слишком большой HTTP-заголовок";
        goto ota_done;
      }
      continue;
    }
    headerDone = true;
    String hdr = header.substring(0, pos);
    int sp1 = hdr.indexOf(' ');
    if (sp1 > 0) httpCode = hdr.substring(sp1 + 1, sp1 + 4).toInt();
    String hdrLower = hdr;
    hdrLower.toLowerCase();
    int clPos = hdrLower.indexOf("content-length:");
    if (clPos >= 0) {
      int eol = hdr.indexOf("\r\n", clPos);
      String clStr = hdr.substring(clPos + 15, eol < 0 ? hdr.length() : eol);
      clStr.trim();
      contentLength = clStr.toInt();
    }
    if (httpCode != 200) {
      result = "OTA: HTTP " + String(httpCode);
      goto ota_done;
    }
    if (contentLength <= 0 || (unsigned long)contentLength > OTA_MAX_BYTES) {
      result = "OTA: некорректный размер " + String(contentLength);
      goto ota_done;
    }
    if (!Update.begin((size_t)contentLength)) {
      result = "OTA: Update.begin() failed";
      goto ota_done;
    }
    if (otaMd5.length() == 32) {
      if (!Update.setMD5(otaMd5.c_str())) {
        result = "OTA: ошибка установки MD5";
        Update.abort();
        goto ota_done;
      }
    }
    {
      int bodyStart = pos + 4;
      int bodyN = (int)header.length() - bodyStart;
      if (bodyN > 0) {
        uint8_t* p = (uint8_t*)(header.c_str() + bodyStart);
        if (Update.write(p, (size_t)bodyN) != (size_t)bodyN) {
          result = "OTA: ошибка записи flash";
          Update.abort();
          goto ota_done;
        }
        bodyGot += (size_t)bodyN;
      }
    }
    header = "";
  }

  while (bodyGot < (size_t)contentLength) {
    if (millis() - started > OTA_TOTAL_TMO) {
      result = "OTA: таймаут загрузки";
      Update.abort();
      goto ota_done;
    }
    int remaining = 0;
    int n = cipRead(chunk, OTA_CHUNK_SIZE, remaining);
    if (n == -2) break;
    if (n < 0) {
      result = "OTA: обрыв на " + String(bodyGot) + "/" + String(contentLength);
      Update.abort();
      goto ota_done;
    }
    if (n == 0) {
      emptyReads++;
      if (emptyReads > 80) {
        result = "OTA: обрыв на " + String(bodyGot) + "/" + String(contentLength);
        Update.abort();
        goto ota_done;
      }
      safeDelay(200);
      yield();
      continue;
    }
    yield();
    emptyReads = 0;
    size_t take = (size_t)n;
    if (bodyGot + take > (size_t)contentLength) take = (size_t)contentLength - bodyGot;
    if (Update.write(chunk, take) != take) {
      result = "OTA: ошибка записи flash";
      Update.abort();
      goto ota_done;
    }
    bodyGot += take;
    if ((bodyGot & 0x7FFF) < take) {
      Serial.print(F("[OTA] "));
      Serial.print(bodyGot);
      Serial.print('/');
      Serial.println(contentLength);
    }
  }

  if (bodyGot != (size_t)contentLength) {
    result = "OTA: неполный файл " + String(bodyGot) + "/" + String(contentLength);
    Update.abort();
    goto ota_done;
  }
  if (!Update.end(true)) {
    result = "OTA: ошибка проверки (MD5?) " + String(Update.errorString());
    goto ota_done;
  }

  ok = true;
  result = String("OTA: ок, ") + String(bodyGot) + " байт, перезагрузка";

ota_done:
  gprsShutdown();
  otaBusy = false;
  Serial.println(result);
  if (otaRequester.length()) sendSMS(otaRequester, result);
  otaUrl = "";
  otaMd5 = "";
  if (ok) {
    safeDelay(2000);
    ESP.restart();
  }
}