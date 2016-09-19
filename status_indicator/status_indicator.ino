#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//static const uint8_t aD0   = 16;
//static const uint8_t aD1   = 5;
//static const uint8_t aD2   = 4;
//static const uint8_t aD3   = 0;
//static const uint8_t aD4   = 2;
//static const uint8_t aD5   = 14;
//static const uint8_t aD6   = 12;
//static const uint8_t aD7   = 13;
//static const uint8_t aD8   = 15;
//static const uint8_t aD9   = 3;
//static const uint8_t aD10  = 1;

#define RED_ALERT_PIN 2
#define GREEN_FIELD_PIN 15
#define BUZZER_PIN 14
#define MOTOR_PIN 12
#define MOTOR_ROT_TIME_PIN A0

// in file sensitive.h put your
// const char* ssid = "SSID";
// and
// const char* password = "PASSWORD_TO_WIFI";
#include "sensitive.h"

const char* mqtt_server = "192.168.1.111";
const int mqtt_port = 1883;

const char* mqtt_username = "";
const char* mqtt_password = "";
const char* mqtt_topic = "statusIndicator";

static const int CONNECTING = 0;
static const int WAITING_FOR_MESSAGE = 1;
static const int OK_STATUS = 2;
static const int ERROR_STATUS = 4;
static const int UNKNOWN_STATUS = 8;

static const int MAX_ERROR_BEEPS = 3;

int status = CONNECTING;
int soundCouter = MAX_ERROR_BEEPS;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(RED_ALERT_PIN, HIGH);
    digitalWrite(GREEN_FIELD_PIN, HIGH);
    delay(250);
    digitalWrite(RED_ALERT_PIN, LOW);
    digitalWrite(GREEN_FIELD_PIN, LOW);
    delay(250);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    char statusArrived = (char)payload[i];
    Serial.print(statusArrived);
    switch (statusArrived) {
      case '0':
        status = OK_STATUS;
        break;
      case '1':
        status = ERROR_STATUS;
        break;
      default:
        status = UNKNOWN_STATUS;
        break;
    }

  }
  Serial.println();
}

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(RED_ALERT_PIN, OUTPUT);
  pinMode(GREEN_FIELD_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  pinMode(MOTOR_ROT_TIME_PIN, INPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void reconnect() {
  while (!client.connected()) {
    status = CONNECTING;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
      status = WAITING_FOR_MESSAGE;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 4 seconds");
      digitalWrite(RED_ALERT_PIN, HIGH);
      digitalWrite(GREEN_FIELD_PIN, HIGH);
      delay(2000);
      digitalWrite(RED_ALERT_PIN, LOW);
      digitalWrite(GREEN_FIELD_PIN, LOW);
      delay(2000);
    }
  }
}


void light (int pin) {
  digitalWrite(pin, HIGH);   
  delay(250);              
  digitalWrite(pin, LOW); 
  delay(250);
}


void sound(int pin)
{
  int freq = 7500;
  analogWriteFreq(freq);  
  for (int i = 0; i < 2; i++) {
    analogWrite(pin, HIGH);
    delay(250);
    analogWrite(pin, 0); 
    delay (250);
  }
}


void turn(int pin)
{
  int rotTime = analogRead(MOTOR_ROT_TIME_PIN);
  // scale
  rotTime = 300 * ( rotTime / 1024.0); 
  Serial.print("  ROT_TIME:");
  Serial.println(rotTime);
  analogWrite(pin, HIGH);
  delay(rotTime);
  analogWrite(pin, LOW);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  
  client.loop();
  switch (status) {
    case WAITING_FOR_MESSAGE:
      light(RED_ALERT_PIN);
      light(GREEN_FIELD_PIN);
      soundCouter = MAX_ERROR_BEEPS;
      break;
    case ERROR_STATUS:
      digitalWrite(RED_ALERT_PIN, HIGH);
      if (soundCouter > 0) {
        sound(BUZZER_PIN);
        soundCouter--;
      } else {
        delay (1000);
      }
      turn(MOTOR_PIN); 
      digitalWrite(RED_ALERT_PIN, LOW);   
      break;
    case OK_STATUS:
      soundCouter = MAX_ERROR_BEEPS;
      digitalWrite(GREEN_FIELD_PIN, HIGH);
      delay(1000);
      digitalWrite(GREEN_FIELD_PIN, LOW);
      break;
    case UNKNOWN_STATUS:
      light(RED_ALERT_PIN);
      light(RED_ALERT_PIN);
      light(GREEN_FIELD_PIN);
      light(GREEN_FIELD_PIN);      
      soundCouter = MAX_ERROR_BEEPS;    
      break;
    default:
       delay(1000);
       soundCouter = MAX_ERROR_BEEPS;
       break;
  }

}
