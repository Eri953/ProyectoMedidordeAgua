#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// Definir pines y parámetros
#define DHT_PIN 25           // Pin de datos del sensor DHT11
#define DHT_TYPE DHT11       // Tipo de sensor DHT
#define WATER_SENSOR_PIN 26  // Pin del sensor de agua (digital)
#define LED_PIN 33           // Pin del LED

// Parámetros Wi-Fi y MQTT
const char* ssid = "S20+ de Erick";         // Nombre de tu red Wi-Fi
const char* password = "erick1234";         // Contraseña de la red Wi-Fi
const char* mqtt_server = "mqtt.eclipseprojects.io"; // Dirección del servidor MQTT
const int mqtt_port = 1883;                 // Puerto MQTT (por defecto 1883)

// Temas MQTT para el grupo 05
const char* topic_temp = "st/grupo05/temperatura";
const char* topic_hum = "st/grupo05/humedad";
const char* topic_agua = "st/grupo05/agua";

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);  // Inicializar el sensor DHT11

// Declaraciones de funciones
void reconnect();
void publicarTemperatura(float temperatura);
void publicarHumedad(float humedad);
void publicarEstadoAgua(const char* estadoAgua);

void setup() {
  // Iniciar comunicación serie
  Serial.begin(115200);

  // Conectar a Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");

  // Configurar MQTT
  client.setServer(mqtt_server, mqtt_port);

  // Inicializar el sensor DHT
  dht.begin();

  // Configurar los pines
  pinMode(LED_PIN, OUTPUT);          // Configurar el LED como salida
  digitalWrite(LED_PIN, LOW);        // Apagar el LED inicialmente
  pinMode(WATER_SENSOR_PIN, INPUT);  // Configurar el sensor de agua como entrada
}

void loop() {
  // Reconectar al servidor MQTT si se pierde la conexión
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer valores del sensor DHT11
  float humedad = dht.readHumidity();
  float temperatura = dht.readTemperature();

  // Verificar si las lecturas son válidas
  if (isnan(humedad) || isnan(temperatura)) {
    Serial.println("Error al leer el sensor DHT11");
    delay(2000);
    return;  // Saltar el resto del loop si hay error
  }

  // Leer valor del sensor de agua (0 = mojado, 1 = seco)
  int waterStatus = digitalRead(WATER_SENSOR_PIN);

  // Controlar el LED en función del estado del sensor de agua
  if (waterStatus == 1) {  // Si el sensor está mojado (LOW)
    digitalWrite(LED_PIN, LOW);  // Apagar el LED
  } else {  // Si el sensor no está mojado (HIGH)
    digitalWrite(LED_PIN, HIGH); // Encender el LED
  }

  // Publicar el estado del sensor de agua en MQTT
  const char* estadoAgua = (waterStatus == LOW) ? "0" : "1";
  publicarEstadoAgua(estadoAgua);

  // Publicar valores de temperatura y humedad en MQTT
  publicarTemperatura(temperatura);
  publicarHumedad(humedad);

  // Mostrar datos por consola
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.print(" °C, Humedad: ");
  Serial.print(humedad);
  Serial.print(" %, Estado del agua: ");
  Serial.println(estadoAgua);

  // Esperar 5 segundos antes de leer nuevamente
  delay(5000);
}

void reconnect() {
  // Intentar conectar al broker MQTT
  while (!client.connected()) {
    Serial.print("Intentando conectar al MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Conectado al broker MQTT");
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando nuevamente en 5 segundos...");
      delay(5000);
    }
  }
}

// Función para publicar la temperatura
void publicarTemperatura(float temperatura) {
  String mensaje = String(temperatura);
  client.publish(topic_temp, mensaje.c_str());
  Serial.println("Temperatura publicada en MQTT: " + mensaje);
}

// Función para publicar la humedad
void publicarHumedad(float humedad) {
  String mensaje = String(humedad);
  client.publish(topic_hum, mensaje.c_str());
  Serial.println("Humedad publicada en MQTT: " + mensaje);
}

// Función para publicar el estado del agua
void publicarEstadoAgua(const char* estadoAgua) {
  client.publish(topic_agua, estadoAgua);
  Serial.println("Estado del agua publicado en MQTT: " + String(estadoAgua));
}
