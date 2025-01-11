#include <ESP8266WiFi.h>

// Configuração Wi-Fi
const char* ssid = "iPhone de Werbert";      // Nome da sua rede Wi-Fi
const char* password = "Vento&mistral";     // Senha da sua rede Wi-Fi

// Configuração do servidor web
WiFiServer server(80);

// Função para ler dados da Serial com timeout
String lerDadosSerialComTimeout(unsigned long timeoutMs) {
  String dado = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial.available()) {  // Lê os dados recebidos do Mega via Serial
      dado = Serial.readStringUntil('\n'); // Lê até o caractere de nova linha
      break;
    }
  }
  return dado.isEmpty() ? "Sem dados disponíveis." : dado;
}

// Função para enviar resposta HTTP ao cliente conectado
void enviarRespostaWiFi(WiFiClient client, String dadoSerial) { // Removido "const"
    String resposta = "<!DOCTYPE html><html>";
    resposta += "<head><title>Monitoramento</title></head><meta http-equiv=\"refresh\" content=\"5\"></head>";
    resposta += "<body>";
    resposta += "<h1>Monitoramento dos Sensores</h1>";
    resposta += "<p>Temperatura: " + dadoSerial.substring(0, dadoSerial.indexOf(';')) + " C</p>";
    dadoSerial.remove(0, dadoSerial.indexOf(';') + 1); // Remove o valor processado

    resposta += "<p>Umidade: " + dadoSerial.substring(0, dadoSerial.indexOf(';')) + " %</p>";
    dadoSerial.remove(0, dadoSerial.indexOf(';') + 1);

    resposta += "<p>Distância 1: " + dadoSerial.substring(0, dadoSerial.indexOf(';')) + " cm</p>";
    dadoSerial.remove(0, dadoSerial.indexOf(';') + 1);

    resposta += "<p>Distância 2: " + dadoSerial.substring(0, dadoSerial.indexOf(';')) + " cm</p>";
    dadoSerial.remove(0, dadoSerial.indexOf(';') + 1);

    resposta += "<p>Distância 3: " + dadoSerial.substring(0, dadoSerial.indexOf(';')) + " cm</p>";
    dadoSerial.remove(0, dadoSerial.indexOf(';') + 1);

    resposta += "<p>Distância 4: " + dadoSerial + " cm</p>";
    resposta += "</body></html>";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println(resposta);

    client.stop();
    Serial.println("Dado enviado ao cliente.");
}

void setup() {
  // Configurações iniciais
  Serial.begin(115200);  // Comunicação serial com o Mega
  Serial.println("Iniciando ESP8266...");

  // Conexão à rede Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Conectando ao Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConectado ao Wi-Fi!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  // Inicia o servidor web
  server.begin();
  Serial.println("Servidor iniciado.");
}

void loop() {

  // Lê dados da porta serial com timeout de 1000ms
  String dadoSerial = lerDadosSerialComTimeout(1000);

  // Verifica se há clientes conectados
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Cliente conectado.");
    enviarRespostaWiFi(client, dadoSerial);
  }

  delay(100);  // Pequeno atraso para evitar travamentos
}
