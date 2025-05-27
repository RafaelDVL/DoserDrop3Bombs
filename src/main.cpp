#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>


// ✅ Variável Global para calibração
#define TEMPO_POR_ML 700 // Tempo de acionamento da bomba para cada 1ml (em ms)

RTC_DS3231 rtc;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long lastFirebaseReconnectAttempt = 0;
const unsigned long firebaseReconnectInterval = 600000;

const char* ssid = "Wifi.Rodrigues";
const char* password = "@Tearsofthedragon26";

#define MAX_PENDING_LOGS 10
#define LOG_INTERVAL      500
struct PendingLog {
  int      bombaIndex;
  float    dosagem;
  String   origem;
  DateTime timestamp;
};

PendingLog pendingLogs[MAX_PENDING_LOGS];
volatile int pendingHead = 0;
volatile int pendingTail = 0;
unsigned long lastLogAttempt = 0;

#define API_KEY "AIzaSyDnUQ4Y12V4R7YKjRJtrWI61FmR5-HGSZU"
#define DATABASE_URL "https://firedoser-default-rtdb.firebaseio.com/"

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define TIME_CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"
#define CONFIG_CHARACTERISTIC_UUID "dcba4321-8765-4321-abcd-0987654321ef"
#define TEST_CHARACTERISTIC_UUID "efab4321-8765-4321-abcd-0987654321ff"

BLEServer *pServer = NULL;
BLECharacteristic *timeCharacteristic = NULL;
BLECharacteristic *configCharacteristic = NULL;
BLECharacteristic *testCharacteristic = NULL;
bool deviceConnected = false;

// ✅ Pinos das bombas (ajuste conforme seu circuito)
#define BOMBA1_PIN 5
#define BOMBA2_PIN 18
#define BOMBA3_PIN 19

// ✅ Estrutura das Bombas
struct Bomb
{
  int hour;
  int minute;
  float dosagem;
  bool diasSemana[7];
  float calibrCoef;
  bool status;
  String name;
  float quantidadeEstoque;

  // Construtor para inicializar corretamente
  Bomb()
  {
    hour = 0;
    minute = 0;
    dosagem = 0;
    calibrCoef = 1.0;
    status = false;
    name = "";
    quantidadeEstoque = 0;
    for (int i = 0; i < 7; i++)
    {
      diasSemana[i] = false;
    }
  }
};

// Inicialmente, os campos novos serão padrão (nome vazio e estoque 0)
Bomb bombas[3];

bool bombaJaAcionada[3] = {false, false, false};

Preferences preferences; // Para salvar as configurações na memória flash

String getTokenType(TokenInfo info) {
  switch (info.type) {
    case token_type_undefined:
      return "undefined";
    case token_type_legacy_token:
      return "legacy token";
    case token_type_id_token:
      return "id token";
    case token_type_custom_token:
      return "custom token";
    case token_type_oauth2_access_token:
      return "OAuth2.0 access token";
    default:
      return "unknown";
  }
}

String getTokenStatus(TokenInfo info) {
  switch (info.status) {
    case token_status_uninitialized:
      return "uninitialized";
    case token_status_on_signing:
      return "on signing";
    case token_status_on_request:
      return "on request";
    case token_status_on_refresh:
      return "on refreshing";
    case token_status_ready:
      return "ready";
    case token_status_error:
      return "error";
    default:
      return "unknown";
  }
}


bool conectarWiFi() {
  Serial.print("🔗 Conectando ao Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi conectado!");
    Serial.print("📡 IP Local: ");
    Serial.println(WiFi.localIP());

    int rssi = WiFi.RSSI();
    Serial.print("📶 Intensidade do sinal Wi-Fi (RSSI): ");
    Serial.print(rssi);
    Serial.println(" dBm");

    // 🔥 Se quiser, pode transformar em texto:
    if (rssi >= -50) {
      Serial.println("🔋 Sinal Excelente");
    } else if (rssi >= -60) {
      Serial.println("🔋 Sinal Muito Bom");
    } else if (rssi >= -70) {
      Serial.println("🔋 Sinal Regular");
    } else {
      Serial.println("⚠️ Sinal Fraco");
    }

    return true;
  } else {
    Serial.println("\n❌ Falha ao conectar no Wi-Fi.");
    return false;
  }
}

void tokenStatusCallback(TokenInfo info) {
  Serial.printf("🛡️ Token info: type = %s, status = %s\n",
                getTokenType(info).c_str(),
                getTokenStatus(info).c_str());
}

void initFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  config.token_status_callback = tokenStatusCallback;

  auth.user.email = "rafaelrodrigues.dsg3d@gmail.com";
  auth.user.password = "@FireDoser123";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  if (Firebase.ready()) {
    Serial.println("✅ Firebase conectado e pronto!");
  } else {
    Serial.println("❌ Firebase não conectado.");
  }
}

// Função para carregar configuração das bombas salvas na memória flash
void loadBombasConfig()
{
  String configJson = preferences.getString("bombas", "");

  if (configJson == "")
  {
    Serial.println("ℹ️ [loadBombasConfig] Nenhuma configuração armazenada encontrada.");
    return;
  }
  Serial.println("🔄 [loadBombasConfig] Carregando configuração armazenada:");
  Serial.println(configJson);

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, configJson);

  if (error)
  {
    Serial.println("❌ [loadBombasConfig] Erro ao desserializar JSON: " + String(error.c_str()));
    return;
  }

  for (int i = 0; i < 3; i++)
  {
    String bombaKey = "bomb" + String(i + 1);
    if (doc[bombaKey].isNull())
    {
      Serial.println("⚠️ [loadBombasConfig] Configuração da bomba " + String(i + 1) + " ausente no JSON!");
      continue;
    }

    JsonObject bomba = doc[bombaKey].as<JsonObject>();

    bombas[i].hour = bomba["time"]["hour"] | 0;
    bombas[i].minute = bomba["time"]["minute"] | 0;
    bombas[i].dosagem = bomba["dosagem"] | 0.0;
    bombas[i].status = bomba["status"] | false;
    bombas[i].calibrCoef = bomba["calibrCoef"] | 1.0;
    bombas[i].name = bomba.containsKey("name") ? String(bomba["name"].as<const char *>()) : "Sem Nome";
    bombas[i].quantidadeEstoque = bomba["quantidadeEstoque"] | 0.0;

    if (bomba.containsKey("diasSemanaSelecionados"))
    {
      JsonArray dias = bomba["diasSemanaSelecionados"].as<JsonArray>();
      for (int j = 0; j < 7; j++)
      {
        bombas[i].diasSemana[j] = dias[j] | false;
      }
    }
    else
    {
      for (int j = 0; j < 7; j++)
      {
        bombas[i].diasSemana[j] = false;
      }
    }

    Serial.println("✅ [loadBombasConfig] Bomba " + String(i + 1) + " carregada:");
    Serial.print("   Nome: ");
    Serial.println(bombas[i].name);
    Serial.print("   Estoque: ");
    Serial.println(bombas[i].quantidadeEstoque);
  }
}

void adicionarLogFirebase(int bombaIndex, float dosagem, String origem) {
  DateTime now = rtc.now();
  String path = "/logs/";
  path += String(now.unixtime());

  FirebaseJson json;
  String timestamp = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()) + " " + String(now.hour()) + ":" + String(now.minute());

  json.set("timestamp", timestamp);
  json.set("bomba", bombas[bombaIndex].name);
  json.set("dosagem", dosagem);
  json.set("origem", origem);

  Serial.print("🔄 Tentando enviar para Firebase em ");
  Serial.println(path);

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("✅ Log enviado para Firebase.");
  } else {
    Serial.print("❌ Erro ao enviar log: ");
    Serial.println(fbdo.errorReason());
  }
}


// Função para acionar a bomba sem registrar log
void acionarBomba(int bombaIndex, float dosagem, String origem)
{

  if (bombaIndex < 0 || bombaIndex >= 3)
  {
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

  Serial.println("🚰 [acionarBomba] Acionando Bomba " + String(bombaIndex + 1) + " (" + bombas[bombaIndex].name + ") por " + String(tempoAtivacao) + "ms");
  digitalWrite(pinoBomba, HIGH);

  unsigned long tempoInicio = millis();

    while (millis() - tempoInicio < tempoAtivacao) {
      delay(10);
    }

    
    digitalWrite(pinoBomba, LOW);
    Serial.println("✅ [acionarBomba] Bomba " + String(bombaIndex + 1) + " desligada!");

  // ✅ Atualiza o estoque: desconta a dosagem, garantindo que não fique negativo
  if (bombas[bombaIndex].quantidadeEstoque > 0)
  {
    bombas[bombaIndex].quantidadeEstoque -= dosagem;
    if (bombas[bombaIndex].quantidadeEstoque < 0)
    {
      bombas[bombaIndex].quantidadeEstoque = 0;
    }
    Serial.print("🔄 [acionarBomba] Novo estoque da bomba ");
    Serial.print(bombaIndex + 1);
    Serial.print(" (");
    Serial.print(bombas[bombaIndex].name);
    Serial.print("): ");
    Serial.println(bombas[bombaIndex].quantidadeEstoque);

    // ✅ Agora salvamos a nova configuração na memória flash para persistência
    DynamicJsonDocument doc(512);
    String configJson = preferences.getString("bombas", "");
    if (!configJson.isEmpty())
    {
      DeserializationError error = deserializeJson(doc, configJson);
      if (error)
      {
        Serial.println("❌ [acionarBomba] Erro ao desserializar JSON antes de salvar estoque!");
      }
    }

    String bombaKey = "bomb" + String(bombaIndex + 1);
    doc[bombaKey]["quantidadeEstoque"] = bombas[bombaIndex].quantidadeEstoque;

    String newConfigJson;
    serializeJson(doc, newConfigJson);
    preferences.putString("bombas", newConfigJson);
    Serial.println("✅ [acionarBomba] Estoque atualizado na memória flash!");
  }


      int nextTail = (pendingTail + 1) % MAX_PENDING_LOGS;
      if (nextTail != pendingHead) {

        DateTime now = rtc.now();
        pendingLogs[pendingTail++] = { bombaIndex, dosagem, origem, now };
        pendingTail = nextTail;
        Serial.println("🔖 Log enfileirado para envio futuro.");
      } else {
        Serial.println("⚠️ Fila de logs cheia, descartando log.");
      }
}


void testarBomba(int bombIndex, float dosagem)
{
  Serial.println("🔎 [testarBomba] Teste de Bomba acionado!");
  if (bombIndex < 0 || bombIndex >= 3)
  {
    Serial.println("❌ [testarBomba] Índice da bomba inválido!");
    return;
  }
  if (dosagem <= 0)
  {
    Serial.println("❌ [testarBomba] Dosagem inválida! Deve ser maior que zero.");
    return;
  }
  Serial.println("🚰 [testarBomba] Testando Bomba " + String(bombIndex + 1) + " (" + bombas[bombIndex].name + ") com dosagem: " + String(dosagem));
  // Aqui chamamos a função de acionamento indicando que se trata de um teste
  acionarBomba(bombIndex, dosagem, "Teste");
}

class TimeCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    Serial.println("📥 [TimeCharacteristic] onWrite acionado.");

    std::string value = pCharacteristic->getValue();
    if (value.empty())
    {
      Serial.println("❌ [TimeCharacteristic] Nenhum dado recebido!");
      return;
    }

    Serial.println("📥 [TimeCharacteristic] Hora recebida: " + String(value.c_str()));

    int dia, mes, ano, hora, minuto, segundo;
    // tenta ler com segundos
    int parsed = sscanf(value.c_str(), "%d/%d/%d %d:%d:%d",
                        &dia, &mes, &ano, &hora, &minuto, &segundo);
    if (parsed == 6)
    {
      // tudo ok, já temos segundos
    }
    else
    {
      // tenta ler apenas dia/mês/ano hora:minuto
      parsed = sscanf(value.c_str(), "%d/%d/%d %d:%d",
                      &dia, &mes, &ano, &hora, &minuto);
      if (parsed == 5)
      {
        segundo = 0; // assume zero
        Serial.println("ℹ️ [TimeCharacteristic] Sem segundos, assumindo 00");
      }
      else
      {
        Serial.println("❌ [TimeCharacteristic] Formato inválido! Esperado: DD/MM/YYYY HH:MM[:SS]");
        return;
      }
    }

    // Se chegou aqui, temos dia, mês, ano, hora, minuto e segundo (talvez =0)
    rtc.adjust(DateTime(ano, mes, dia, hora, minuto, segundo));
    Serial.printf("✅ [TimeCharacteristic] Data e hora ajustadas para: %02d/%02d/%04d %02d:%02d:%02d\n",
                  dia, mes, ano, hora, minuto, segundo);
  }
};


class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("✅ [MyServerCallbacks] Dispositivo conectado via Bluetooth!");
  }
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("⚠️ [MyServerCallbacks] Dispositivo desconectado!");
    BLEDevice::startAdvertising();
  }
};

class TestCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    Serial.println("📥 [TestCharacteristic] onWrite acionado.");

    std::string value = pCharacteristic->getValue();
    if (value.empty())
    {
      Serial.println("❌ [TestCharacteristic] Nenhum dado recebido!");
      return;
    }

    Serial.println("📥 [TestCharacteristic] Comando recebido:");
    Serial.println(value.c_str());

    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, value);
    if (error)
    {
      Serial.println("❌ [TestCharacteristic] Erro ao desserializar JSON de teste: " + String(error.c_str()));
      return;
    }

    if (doc["bomb"].isNull() || doc["dosagem"].isNull())
    {
      Serial.println("❌ [TestCharacteristic] JSON incompleto! Esperado: {\"bomb\": <int>, \"dosagem\": <float>}");
      return;
    }

    int bombIndex = doc["bomb"];
    float dosagem = doc["dosagem"];

    if (bombIndex < 0 || bombIndex >= 3)
    {
      Serial.println("❌ [TestCharacteristic] Índice da bomba inválido!");
      return;
    }
    if (dosagem <= 0)
    {
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

class ConfigCharacteristicCallbacks : public BLECharacteristicCallbacks
{

  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    Serial.println("📥 [ConfigCharacteristic] onWrite acionado.");

    std::string value = pCharacteristic->getValue();
    if (value.length() == 0)
    {
      Serial.println("❌ [ConfigCharacteristic] Nenhum valor recebido para configuração!");
      return;
    }

    Serial.println("📥 [ConfigCharacteristic] JSON recebido:");
    Serial.println(value.c_str());

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, value);
    if (error)
    {
      Serial.println("❌ [ConfigCharacteristic] Erro ao desserializar JSON: " + String(error.c_str()));
      return;
    }

    for (int i = 0; i < 3; i++)
    {
      String bombaKey = "bomb" + String(i + 1);
      if (!doc.containsKey(bombaKey))
        continue;

      JsonObject bomba = doc[bombaKey];

      bombas[i].name = bomba.containsKey("name") ? String(bomba["name"].as<const char *>()) : "Sem Nome";
      bombas[i].hour = bomba["time"]["hour"] | 0;
      bombas[i].minute = bomba["time"]["minute"] | 0;
      bombas[i].dosagem = bomba["dosagem"] | 0.0;
      bombas[i].status = bomba["status"] | false;
      bombas[i].calibrCoef = bomba["calibrCoef"] | 1.0;
      bombas[i].quantidadeEstoque = bomba["quantidadeEstoque"] | 0.0;

      if (!bomba["diasSemanaSelecionados"].isNull())
      {
        JsonArray dias = bomba["diasSemanaSelecionados"].as<JsonArray>();
        for (int j = 0; j < 7; j++)
        {
          bombas[i].diasSemana[j] = dias[j].as<bool>();
        }
      }

      Serial.println("✅ [ConfigCharacteristic] Bomba " + String(i + 1) + " atualizada!");
      Serial.print("   Nome: ");
      Serial.println(bombas[i].name);
      Serial.print("   Hora: ");
      Serial.println(bombas[i].hour);
      Serial.print("   Minuto: ");
      Serial.println(bombas[i].minute);
      Serial.print("   Dosagem: ");
      Serial.println(bombas[i].dosagem);
      Serial.print("   Calib. Coef.: ");
      Serial.println(bombas[i].calibrCoef);
      Serial.print("   Quantidade Estoque: ");
      Serial.println(bombas[i].quantidadeEstoque);
    }

    Serial.println("✅ [ConfigCharacteristic] Configuração das bombas atualizada no ESP32!");

    // Salva a configuração na memória não volátil
    preferences.putString("bombas", String(value.c_str()));
    Serial.println("✅ [ConfigCharacteristic] Configuração salva na memória flash!");
  }

  // Callback de leitura que retorna a configuração salva na memória flash
  void onRead(BLECharacteristic *pCharacteristic) override
  {
    // Cria um JSON dinâmico com tamanho suficiente
    DynamicJsonDocument doc(512);

    // Para cada bomba, monta o objeto JSON
    for (int i = 0; i < 3; i++)
    {
      String bombaKey = "bomb" + String(i + 1);
      JsonObject bomba = doc.createNestedObject(bombaKey);

      // Adiciona o horário
      JsonObject timeObj = bomba.createNestedObject("time");
      timeObj["hour"] = bombas[i].hour;
      timeObj["minute"] = bombas[i].minute;

      // Adiciona os demais campos
      bomba["dosagem"] = bombas[i].dosagem;
      bomba["status"] = bombas[i].status;
      bomba["calibrCoef"] = bombas[i].calibrCoef;
      bomba["name"] = bombas[i].name;
      bomba["quantidadeEstoque"] = bombas[i].quantidadeEstoque;

      // Adiciona o array de dias da semana
      JsonArray dias = bomba.createNestedArray("diasSemanaSelecionados");
      for (int j = 0; j < 7; j++)
      {
        dias.add(bombas[i].diasSemana[j]);
      }
    }

    // Serializa o JSON para uma string
    String configJson;
    serializeJson(doc, configJson);

    // Atualiza a característica BLE com o JSON construído
    pCharacteristic->setValue(configJson.c_str());

    Serial.println("📤 [ConfigCharacteristic] Enviando configuração atualizada via BLE:");
    Serial.println(configJson);
  }
};


// Inicialização

void inicializarBombas()
{
  pinMode(BOMBA1_PIN, OUTPUT);
  pinMode(BOMBA2_PIN, OUTPUT);
  pinMode(BOMBA3_PIN, OUTPUT);
  digitalWrite(BOMBA1_PIN, LOW);
  digitalWrite(BOMBA2_PIN, LOW);
  digitalWrite(BOMBA3_PIN, LOW);
  Serial.println("✅ [inicializarBombas] Bombas inicializadas!");
}

void setup()
{
  Serial.begin(115200);
  conectarWiFi();
  initFirebase(); 
  Serial.println("🚀 [setup] Iniciando ESP32 com os servidores...");

  if (!rtc.begin())
  {
    Serial.println("❌ [setup] Erro ao iniciar o RTC!");
  }

  if (rtc.lostPower())
{
  Serial.println("⚠️ [setup] RTC perdeu a alimentação!");
  Serial.println("⏳ Aguardando ajuste de data e hora via BLE...");
}
  inicializarBombas();

  // Inicializa a memória não volátil
  preferences.begin("bomb-config", false);
  // Carrega configurações salvas (se houver)
  loadBombasConfig();

  BLEDevice::init("ESP32_Aquario2");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // ✅ Característica de Hora (Leitura, Notificação e Escrita)
  timeCharacteristic = pService->createCharacteristic(
      TIME_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  timeCharacteristic->setCallbacks(new TimeCharacteristicCallbacks());
  timeCharacteristic->addDescriptor(new BLE2902());

  // ✅ Característica de Teste (Leitura e Escrita)
  testCharacteristic = pService->createCharacteristic(
      TEST_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  testCharacteristic->setCallbacks(new TestCharacteristicCallbacks());
  testCharacteristic->addDescriptor(new BLE2902());

  // ✅ Característica de Configuração (Leitura e Escrita)
  configCharacteristic = pService->createCharacteristic(
      CONFIG_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  configCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());


  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("✅ [setup] Servidor BLE pronto!");
}

void loop() {
  // ─── 0) Garantir Wi-Fi conectado ───
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Wi-Fi caiu, reconectando...");
    conectarWiFi();
  }

  // ─── 1) Envio de 1 log pendente a cada LOG_INTERVAL ───
  if (pendingHead != pendingTail && millis() - lastLogAttempt >= LOG_INTERVAL) {
    lastLogAttempt = millis();
    PendingLog &lg = pendingLogs[pendingHead];

    // Prepara caminho e JSON
    String path = "/logs/" + String(lg.timestamp.unixtime());
    FirebaseJson json;
    String ts = String(lg.timestamp.day()) + "/"
              + String(lg.timestamp.month()) + "/"
              + String(lg.timestamp.year()) + " "
              + String(lg.timestamp.hour()) + ":"
              + String(lg.timestamp.minute());
    json.set("timestamp", ts);
    json.set("bomba",   bombas[lg.bombaIndex].name);
    json.set("dosagem", lg.dosagem);
    json.set("origem",  lg.origem);

    Serial.print("🔄 Tentando enviar log pendente em ");
    Serial.println(path);

    // 1.a) Forçar refresh do token se necessário
    if (!Firebase.ready() &&
        millis() - lastFirebaseReconnectAttempt > firebaseReconnectInterval) {
      lastFirebaseReconnectAttempt = millis();
      Serial.println("🔄 Firebase não está pronto. Tentando renovar token...");
      Firebase.refreshToken(&config);
      Firebase.begin(&config, &auth);
      if (Firebase.ready()) {
        Serial.println("✅ Firebase reconectado com sucesso!");
      } else {
        Serial.print("❌ Erro ao reconectar Firebase: ");
        Serial.println(fbdo.errorReason());
      }
    }

    // 1.b) Tentar enviar, renovado ou não
    if (Firebase.ready() && Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
      Serial.println("✅ Log pendente enviado!");
      pendingHead = (pendingHead + 1) % MAX_PENDING_LOGS;
    } else {
      Serial.print("❌ Falha ao enviar log: ");
      Serial.println(fbdo.errorReason());
    }
  }

  // ─── 2) Lógica de 1s: atualização de hora BLE e acionamento programado ───
  static int ultimoMinuto = -1;
  static unsigned long lastCheckTime = 0;

  if (millis() - lastCheckTime >= 1000) {
    lastCheckTime = millis();
    DateTime now = rtc.now();
    int diaSemana = now.dayOfTheWeek();
    const char *diasSemanaNomes[] = {
      "Domingo","Segunda","Terça","Quarta","Quinta","Sexta","Sábado"
    };

    // 2.a) Notificar BLE só quando o minuto muda
    if (now.minute() != ultimoMinuto) {
      ultimoMinuto = now.minute();

      // ─── Monta JSON de status ───
      DynamicJsonDocument statusDoc(256);

      // 1) Hora formatada
      char buf[20];
      snprintf(buf, sizeof(buf),
               "%02d/%02d/%04d %02d:%02d",
               now.day(), now.month(), now.year(),
               now.hour(), now.minute());
      statusDoc["time"] = buf;

      // 2) Status do Wi-Fi
      bool wifiOk = (WiFi.status() == WL_CONNECTED);
      statusDoc["wifi"]["connected"] = wifiOk;
      statusDoc["wifi"]["rssi"]      = wifiOk ? WiFi.RSSI() : 0;

      // 3) Status do Firebase
      statusDoc["firebase"]["ready"] = Firebase.ready();

      // Serializa e notifica via BLE
      String payload;
      serializeJson(statusDoc, payload);
      timeCharacteristic->setValue(payload.c_str());
      timeCharacteristic->notify();

      // Também log no Serial para debug
      Serial.printf("📅 %02d/%02d/%04d (%s)\n",
                    now.day(), now.month(), now.year(),
                    diasSemanaNomes[diaSemana]);
      Serial.printf("🕒 %02d:%02d\n", now.hour(), now.minute());
      Serial.println("🔄 Resetando flags de acionamento");
      for (int i = 0; i < 3; i++) {
        bombaJaAcionada[i] = false;
      }
    }

    // 2.b) Acionamento programado das bombas
    for (int i = 0; i < 3; i++) {
      if (bombas[i].status &&
          !bombaJaAcionada[i] &&
          bombas[i].hour   == now.hour()   &&
          bombas[i].minute == now.minute() &&
          bombas[i].diasSemana[diaSemana]) {
        Serial.printf("⏳ Programado: bomba %d (%s) às %02d:%02d\n",
                      i+1, bombas[i].name.c_str(),
                      now.hour(), now.minute());
        acionarBomba(i, bombas[i].dosagem, "Programado");
        bombaJaAcionada[i] = true;
      }
    }
  }
}




