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

BLEServer *pServer = NULL;
BLECharacteristic *timeCharacteristic = NULL;
BLECharacteristic *configCharacteristic = NULL;
BLECharacteristic *testCharacteristic = NULL;
bool deviceConnected = false;

// ✅ Pinos das bombas (ajuste conforme seu circuito)
#define BOMBA1_PIN 5
#define BOMBA2_PIN 19
#define BOMBA3_PIN 18

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
    while (millis() - tempoInicio < tempoAtivacao) {}
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

// ===================
// Callbacks BLE
// ===================

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
    if (sscanf(value.c_str(), "%d/%d/%d %d:%d:%d", &dia, &mes, &ano, &hora, &minuto, &segundo) == 6)
    {
      rtc.adjust(DateTime(ano, mes, dia, hora, minuto, segundo));
      Serial.printf("✅ [TimeCharacteristic] Data e hora ajustadas para: %02d/%02d/%04d %02d:%02d:%02d\n", dia, mes, ano, hora, minuto, segundo);
    }
    else
    {
      Serial.println("❌ [TimeCharacteristic] Formato inválido! Esperado: DD/MM/YYYY HH:MM:SS");
    }
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
  Serial.println("🚀 [setup] Iniciando ESP32 como Servidor BLE...");

  if (!rtc.begin())
  {
    Serial.println("❌ [setup] Erro ao iniciar o RTC!");
  }
  if (rtc.lostPower())
  {
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

void loop()
{
    static int ultimoMinuto = -1;
    static unsigned long lastCheckTime = 0; // Controle para rodar o loop a cada 1s

    // ✅ Executa apenas se passou 1 segundo desde a última execução
    if (millis() - lastCheckTime >= 1000)
    {
        lastCheckTime = millis(); // Atualiza o tempo da última checagem

        DateTime now = rtc.now();
        int diaSemana = now.dayOfTheWeek();
        const char *diasSemanaNomes[] = {"Domingo", "Segunda", "Terça", "Quarta", "Quinta", "Sexta", "Sábado"};

        // ✅ Só imprime a hora quando o minuto muda
        if (now.minute() != ultimoMinuto)
        {
            ultimoMinuto = now.minute();  // Atualiza o minuto salvo

            // Atualiza a característica BLE com a hora
            char timeString[20];
            snprintf(timeString, sizeof(timeString), "%02d/%02d/%04d %02d:%02d", 
                     now.day(), now.month(), now.year(), now.hour(), now.minute());
            timeCharacteristic->setValue(timeString);
            timeCharacteristic->notify();

            // ✅ Printa a hora apenas uma vez por minuto
            Serial.printf("📅 Data atual: %02d/%02d/%04d (%s)\n", now.day(), now.month(), now.year(), diasSemanaNomes[diaSemana]);
            Serial.printf("🕒 Hora atual: %02d:%02d\n", now.hour(), now.minute());
            Serial.println("🔄 Reiniciando status das bombas para novo minuto.");

            // ✅ Reseta a flag de acionamento das bombas
            for (int i = 0; i < 3; i++)
            {
                bombaJaAcionada[i] = false;
            }
        }

        // ✅ Verifica se alguma bomba precisa ser acionada
        for (int i = 0; i < 3; i++)
        {
            if (bombas[i].status && !bombaJaAcionada[i] &&
                bombas[i].hour == now.hour() && bombas[i].minute == now.minute() &&
                bombas[i].diasSemana[diaSemana])
            {
                Serial.printf("⏳ Acionando bomba %d (%s) às %02d:%02d no dia %s\n",
                              i + 1, bombas[i].name.c_str(), now.hour(), now.minute(), diasSemanaNomes[diaSemana]);

                acionarBomba(i, bombas[i].dosagem, "Programado");
                bombaJaAcionada[i] = true;
            }
        }
    }
}
