#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ✅ Variável Global para calibração
#define TEMPO_POR_ML 700 // Tempo de acionamento da bomba para cada 1ml (em ms)

RTC_DS3231 rtc;

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define TIME_CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"
#define CONFIG_CHARACTERISTIC_UUID "dcba4321-8765-4321-abcd-0987654321ef"
#define TEST_CHARACTERISTIC_UUID "efab4321-8765-4321-abcd-0987654321ff"
#define LOG_CHARACTERISTIC_UUID "fedcba98-7654-3210-fedc-ba9876543210"

BLEServer *pServer = NULL;
BLECharacteristic *timeCharacteristic = NULL;
BLECharacteristic *configCharacteristic = NULL;
BLECharacteristic *testCharacteristic = NULL;
BLECharacteristic *logCharacteristic = NULL;
bool deviceConnected = false;

// ✅ Pinos das bombas (ajuste conforme seu circuito)
#define BOMBA1_PIN 5
#define BOMBA2_PIN 19
#define BOMBA3_PIN 18

// ✅ Estrutura das Bombas
struct Bomb {
  int hour;
  int minute;
  float dosagem;
  bool diasSemana[7];
  float calibrCoef = 1.0; // Coeficiente de calibração
  bool status;
};

Bomb bombas[3];
bool bombaJaAcionada[3] = { false, false, false };

Preferences preferences; // Para salvar as configurações na memória flash

// ===================
// Função de Log de Acionamento
// ===================

void logAcionamento(int bombaIndex, float dosagem, String origem) {
  DateTime now = rtc.now();
  char buffer[50];
  sprintf(buffer, "%02d/%02d - %02d:%02d - Bomba %d - %.1fmls - %s", 
          now.day(), now.month(), now.hour(), now.minute(), bombaIndex + 1, dosagem, origem.c_str());
  String newEntry = String(buffer);
  
  // Recupera o log atual da memória flash
  String log = preferences.getString("pump_log", "");
  if (log.length() > 0) {
    log += "\n";
  }
  log += newEntry;
  
  // Salva o log atualizado na flash
  preferences.putString("pump_log", log);
  
  Serial.println("✅ [logAcionamento] Registro salvo:");
  Serial.println(newEntry);
}

// Função para carregar configuração das bombas salvas na memória flash
void loadBombasConfig() {
  String configJson = preferences.getString("bombas", "");
  if (configJson == "") {
    Serial.println("ℹ️ [loadBombasConfig] Nenhuma configuração armazenada encontrada.");
    return;
  }
  Serial.println("🔄 [loadBombasConfig] Carregando configuração armazenada:");
  Serial.println(configJson);

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, configJson);
  if (error) {
    Serial.println("❌ [loadBombasConfig] Erro ao desserializar JSON: " + String(error.c_str()));
    return;
  }
  for (int i = 0; i < 3; i++) {
    String bombaKey = "bomb" + String(i + 1);
    if (doc[bombaKey].isNull()) {
      Serial.println("⚠️ [loadBombasConfig] Configuração da bomba " + String(i + 1) + " ausente no JSON!");
      continue;
    }
    JsonObject bomba = doc[bombaKey].as<JsonObject>();
    bombas[i].hour = bomba["time"]["hour"];
    bombas[i].minute = bomba["time"]["minute"];
    bombas[i].dosagem = bomba["dosagem"];
    bombas[i].status = bomba["status"];
    bombas[i].calibrCoef = bomba["calibrCoef"];
    if (!bomba["diasSemanaSelecionados"].isNull()) {
      JsonArray dias = bomba["diasSemanaSelecionados"].as<JsonArray>();
      for (int j = 0; j < 7; j++) {
        bombas[i].diasSemana[j] = dias[j];
      }
    }
    Serial.println("✅ [loadBombasConfig] Bomba " + String(i + 1) + " configurada:");
    Serial.print("   Hora: "); Serial.println(bombas[i].hour);
    Serial.print("   Minuto: "); Serial.println(bombas[i].minute);
    Serial.print("   Dosagem: "); Serial.println(bombas[i].dosagem);
    Serial.print("   Calib. Coef.: "); Serial.println(bombas[i].calibrCoef);
  }
}

// ===================
// Métodos de Acionamento
// ===================

// Agora a função recebe um parâmetro a mais "origem" para identificar o tipo de acionamento
void acionarBomba(int bombaIndex, float dosagem, String origem) {
  if (bombaIndex < 0 || bombaIndex >= 3) {
    Serial.println("❌ [acionarBomba] Índice de bomba inválido!");
    return;
  }
  int tempoAtivacao = dosagem * TEMPO_POR_ML * bombas[bombaIndex].calibrCoef;
  int pinoBomba;
  switch (bombaIndex) {
    case 0: pinoBomba = BOMBA1_PIN; break;
    case 1: pinoBomba = BOMBA2_PIN; break;
    case 2: pinoBomba = BOMBA3_PIN; break;
    default: return;
  }
  Serial.println("🚰 [acionarBomba] Acionando Bomba " + String(bombaIndex + 1) + " por " + String(tempoAtivacao) + "ms");
  digitalWrite(pinoBomba, HIGH);
  delay(tempoAtivacao);
  digitalWrite(pinoBomba, LOW);
  Serial.println("✅ [acionarBomba] Bomba " + String(bombaIndex + 1) + " desligada!");
  
  // Registra o acionamento no log
  logAcionamento(bombaIndex, dosagem, origem);
}

void testarBomba(int bombIndex, float dosagem) {
  Serial.println("🔎 [testarBomba] Teste de Bomba acionado!");
  if (bombIndex < 0 || bombIndex >= 3) {
    Serial.println("❌ [testarBomba] Índice da bomba inválido!");
    return;
  }
  if (dosagem <= 0) {
    Serial.println("❌ [testarBomba] Dosagem inválida! Deve ser maior que zero.");
    return;
  }
  Serial.println("🚰 [testarBomba] Testando Bomba " + String(bombIndex + 1) + " com dosagem: " + String(dosagem));
  // Aqui chamamos a função de acionamento indicando que se trata de um teste
  acionarBomba(bombIndex, dosagem, "Teste");
}

// ===================
// Callbacks BLE
// ===================

class TimeCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    Serial.println("📥 [TimeCharacteristic] onWrite acionado.");
    
    std::string value = pCharacteristic->getValue();
    if (value.empty()) {
      Serial.println("❌ [TimeCharacteristic] Nenhum dado recebido!");
      return;
    }

    Serial.println("📥 [TimeCharacteristic] Hora recebida: " + String(value.c_str()));

    int hora, minuto, segundo;
    if (sscanf(value.c_str(), "%d:%d:%d", &hora, &minuto, &segundo) == 3) {
      rtc.adjust(DateTime(2025, 3, 11, hora, minuto, segundo));
      Serial.printf("✅ [TimeCharacteristic] Hora ajustada para: %02d:%02d:%02d\n", hora, minuto, segundo);
    } else {
      Serial.println("❌ [TimeCharacteristic] Formato de hora inválido! Esperado: HH:MM:SS");
    }
  }
};

class LogCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) override {
    String log = preferences.getString("pump_log", "");
    pCharacteristic->setValue(log.c_str());
    Serial.println("📤 [LogCharacteristic] Leitura solicitada, enviando log:");
    Serial.println(log);
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("✅ [MyServerCallbacks] Dispositivo conectado via Bluetooth!");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("⚠️ [MyServerCallbacks] Dispositivo desconectado!");
    BLEDevice::startAdvertising();
  }
};

class TestCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    Serial.println("📥 [TestCharacteristic] onWrite acionado.");
    
    std::string value = pCharacteristic->getValue();
    if (value.empty()) {
      Serial.println("❌ [TestCharacteristic] Nenhum dado recebido!");
      return;
    }
    
    Serial.println("📥 [TestCharacteristic] Comando recebido:");
    Serial.println(value.c_str());
    
    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, value);
    if (error) {
      Serial.println("❌ [TestCharacteristic] Erro ao desserializar JSON de teste: " + String(error.c_str()));
      return;
    }
    
    if (doc["bomb"].isNull() || doc["dosagem"].isNull()) {
      Serial.println("❌ [TestCharacteristic] JSON incompleto! Esperado: {\"bomb\": <int>, \"dosagem\": <float>}");
      return;
    }
    
    int bombIndex = doc["bomb"];
    float dosagem = doc["dosagem"];
    
    if (bombIndex < 0 || bombIndex >= 3) {
      Serial.println("❌ [TestCharacteristic] Índice da bomba inválido!");
      return;
    }
    if (dosagem <= 0) {
      Serial.println("❌ [TestCharacteristic] Dosagem inválida!");
      return;
    }
    
    Serial.println("🚰 [TestCharacteristic] Acionando teste de bomba...");
    testarBomba(bombIndex, dosagem);
    
    std::string response = "OK, teste da bomba " + std::to_string(bombIndex + 1) + " realizado!";
    pCharacteristic->setValue(response);
    
    Serial.println("✅ [TestCharacteristic] Resposta enviada ao cliente!");
  }
};

class ConfigCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    Serial.println("📥 [ConfigCharacteristic] onWrite acionado.");
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) {
      Serial.println("❌ [ConfigCharacteristic] Nenhum valor recebido para configuração!");
      return;
    }
    Serial.println("📥 [ConfigCharacteristic] JSON recebido:");
    Serial.println(value.c_str());

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, value);
    if (error) {
      Serial.println("❌ [ConfigCharacteristic] Erro ao desserializar JSON: " + String(error.c_str()));
      return;
    }

    for (int i = 0; i < 3; i++) {
      String bombaKey = "bomb" + String(i + 1);
      if (doc[bombaKey].isNull()) {
        Serial.println("⚠️ [ConfigCharacteristic] Configuração da bomba " + String(i + 1) + " ausente no JSON!");
        continue;
      }
      JsonObject bomba = doc[bombaKey].as<JsonObject>();
      bombas[i].hour = bomba["time"]["hour"];
      bombas[i].minute = bomba["time"]["minute"];
      bombas[i].dosagem = bomba["dosagem"];
      bombas[i].status = bomba["status"];
      bombas[i].calibrCoef = bomba["calibrCoef"];
      if (!bomba["diasSemanaSelecionados"].isNull()) {
        JsonArray dias = bomba["diasSemanaSelecionados"].as<JsonArray>();
        for (int j = 0; j < 7; j++) {
          bombas[i].diasSemana[j] = dias[j];
        }
      }
      Serial.println("✅ [ConfigCharacteristic] Bomba " + String(i + 1) + " atualizada!");
      Serial.print("   Hora: "); Serial.println(bombas[i].hour);
      Serial.print("   Minuto: "); Serial.println(bombas[i].minute);
      Serial.print("   Dosagem: "); Serial.println(bombas[i].dosagem);
      Serial.print("   Calib. Coef.: "); Serial.println(bombas[i].calibrCoef);
    }
    Serial.println("✅ [ConfigCharacteristic] Configuração das bombas atualizada no ESP32!");

    // Salva a configuração na memória não volátil
    preferences.putString("bombas", String(value.c_str()));
    Serial.println("✅ [ConfigCharacteristic] Configuração salva na memória flash!");

    // Atualiza o valor da característica BLE com a nova configuração para que o app possa lê-la posteriormente
    pCharacteristic->setValue(value.c_str());
  }

  // Callback de leitura que retorna a configuração salva na memória flash
  void onRead(BLECharacteristic *pCharacteristic) override {
    String configJson = preferences.getString("bombas", "");
    pCharacteristic->setValue(configJson.c_str());
    Serial.println("📤 [ConfigCharacteristic] Leitura solicitada, enviando configuração:");
    Serial.println(configJson);
  }
};

// ===================
// Inicialização
// ===================

void inicializarBombas() {
  pinMode(BOMBA1_PIN, OUTPUT);
  pinMode(BOMBA2_PIN, OUTPUT);
  pinMode(BOMBA3_PIN, OUTPUT);
  digitalWrite(BOMBA1_PIN, LOW);
  digitalWrite(BOMBA2_PIN, LOW);
  digitalWrite(BOMBA3_PIN, LOW);
  Serial.println("✅ [inicializarBombas] Bombas inicializadas!");
}

void setup() {
  Serial.begin(115200);
  Serial.println("🚀 [setup] Iniciando ESP32 como Servidor BLE...");
  
  if (!rtc.begin()) {
    Serial.println("❌ [setup] Erro ao iniciar o RTC!");
  }
  if (rtc.lostPower()) {
    Serial.println("⚠️ [setup] RTC perdeu a alimentação, ajustando data e hora...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  inicializarBombas();
  
  // Inicializa a memória não volátil
  preferences.begin("bomb-config", false);
  // Carrega configurações salvas (se houver)
  loadBombasConfig();
  
  BLEDevice::init("ESP32_Aquario");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // ✅ Característica de Hora (Leitura, Notificação e Escrita)
  timeCharacteristic = pService->createCharacteristic(
    TIME_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
  );
  timeCharacteristic->setCallbacks(new TimeCharacteristicCallbacks());
  timeCharacteristic->addDescriptor(new BLE2902());
  
  // ✅ Característica de Teste (Leitura e Escrita)
  testCharacteristic = pService->createCharacteristic(
    TEST_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );
  testCharacteristic->setCallbacks(new TestCharacteristicCallbacks());
  testCharacteristic->addDescriptor(new BLE2902());
  
  // ✅ Característica de Configuração (Leitura e Escrita)
  configCharacteristic = pService->createCharacteristic(
    CONFIG_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  configCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());

  // ✅ Nova Característica de Log (Apenas Leitura)
  logCharacteristic = pService->createCharacteristic(
    LOG_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ
  );
  logCharacteristic->setCallbacks(new LogCharacteristicCallbacks());
  
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ [setup] Servidor BLE pronto!");
}
  
void loop() {
  static int ultimoMinuto = -1; // Variável para detectar mudança de minuto
  DateTime now = rtc.now();

  Serial.printf("🕒 Hora atual: %02d:%02d:%02d\n", now.hour(), now.minute(), now.second());

  // Atualiza a característica BLE com o horário atual
  char timeString[9];
  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  timeCharacteristic->setValue(timeString);
  timeCharacteristic->notify();

  // Se o minuto mudou, reinicia a variável de controle das bombas
  if (now.minute() != ultimoMinuto) {
    Serial.println("🔄 Reiniciando status das bombas para novo minuto.");
    for (int i = 0; i < 3; i++) {
      bombaJaAcionada[i] = false;
    }
    ultimoMinuto = now.minute();
  }

  // Obtemos o dia da semana atual (0 = Domingo, 1 = Segunda, ... 6 = Sábado)
  int diaSemana = now.dayOfTheWeek();

  // Verifica se alguma bomba precisa ser acionada, considerando o dia da semana
  for (int i = 0; i < 3; i++) {
    if (bombas[i].status && !bombaJaAcionada[i]) {
      if (bombas[i].hour == now.hour() && bombas[i].minute == now.minute()) {
        if (bombas[i].diasSemana[diaSemana]) {
          Serial.printf("⏳ Acionando bomba %d às %02d:%02d no dia %d\n", i + 1, now.hour(), now.minute(), diaSemana);
          // Aqui indicamos que o acionamento é "Programado"
          acionarBomba(i, bombas[i].dosagem, "Programado");
          bombaJaAcionada[i] = true;
        } else {
          Serial.printf("ℹ️ Bomba %d não acionada: dia %d não habilitado\n", i + 1, diaSemana);
        }
      }
    }
  }

  delay(1000); // Aguarda 1 segundo
}
