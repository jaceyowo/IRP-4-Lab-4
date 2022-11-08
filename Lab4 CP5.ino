/****************************************
 * Activity4: IoT
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#define Builtin_LED 13

WiFiClient ubidots;
PubSubClient client(ubidots);

char mqttBroker[]  = "industrial.api.ubidots.com";                // MQTT broker domain name for Ubidots
char payload[100];
char topic_pub[150];
char topic_sub[150];
char str_sensor[10];
unsigned long timer = 0;                                          // initial timer value
float sub_payload;
float sensor;


/****************************************
 * WIFI and Ubidots Credential
 ****************************************/
#define WIFISSID "LAPTOP-GCAD0KR7 1308"                                        // Line1: Wifi SSID
#define PASSWORD "0$o6U800"                                        // Line2: Wifi password
#define TOKEN "BBFF-jg5OfeuyEvgWfIGiQFNO6rDFMksKQS"                     // Line3: Ubidots TOKEN

/****************************************
 * Define Constants and assign Pins
 ****************************************/
#define MQTT_CLIENT_NAME "S10223377"                          // Line4: Set unique device client Name; 8-12 alphanumeric string 
#define VARIABLE_LABEL "lux"                                // Line5: Set variable label; lowercase only
#define DEVICE_LABEL "lightsensor"                            // Line6: Set device label; lowercase only                                                  

#define SENSOR 32                                             // Line7: Assign SENSOR to pin32
#define LED 33                                                // Line8: Assign LED to pin33
#define PB 25                                                 // Line9: Assign Pushbutton to pin25
#define RELAY 27                                              // Line10: Assign Relay control to pin27

float Threshold = 120;                                          // Line11: Set Threshold intensity(in Volts)
unsigned long interval= 10000;                                // Line12: Set interval between publish(in ms)

/************ Main Functions*************/
void setup() {
  Serial.begin(115200);
  pinMode(Builtin_LED, OUTPUT);   
  pinMode(SENSOR, INPUT);                                     // Line13: Set SENSOR as INPUT
  pinMode(LED, OUTPUT);                                       // Line14: Set LED as OUTPUT
  pinMode(PB, INPUT);                                         // Line15: Set PB as INPUT
  pinMode(RELAY, OUTPUT);                                     // Line16: Set RELAY as OUTPUT
  digitalWrite(Builtin_LED, HIGH);                            // initialise Builtin_LED pin HIGH; LED off
  digitalWrite(RELAY, HIGH);                                  // initialise RELAY pin HIGH; relay not energised
  digitalWrite(LED, HIGH);                                    // initialise LED pin HIGH; LED off
  iot_setup();                                                // Line17: Setup wifi and mqtt broker
} 

void loop() {
  if (millis() > timer){                                          // if uC clock exceed timer, read sensor and publish
  if (!client.connected()) {                                // Line18: Check connection to broker
    mqttconnect();                                          // Line19: Connect to broker
    client.subscribe(topic_sub);                            // Line20: Subscribe to "topic_sub"
  }                                                         // Line21:
                                                    // Line27: blank; add instruction if required
  }
  client.loop();                                            // Line31: mqtt real time function
}

/****************************************
 * Functions
 ****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  sub_payload = atof(p);
  Serial.print(sub_payload);
  Serial.println(topic);
  if (sub_payload < Threshold) {                              // Line32: if received subscribe payload < Threshold, do line 33
    digitalWrite(RELAY, HIGH);                           // Line33: turn on Builtin_LED
  } else{
    digitalWrite(RELAY, LOW);                          // Line34: else turn off Builtin LED
  }
}
 
/* Connect to MQTT broker */
void mqttconnect() {
    while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
   }
}

/* Brink builtin_LED twice */
void blink_LED(){
  digitalWrite(Builtin_LED, LOW);                                        
  delay(500);
  digitalWrite(Builtin_LED, HIGH);
  delay(500);
  digitalWrite(Builtin_LED, LOW);  
  delay(500);
  digitalWrite(Builtin_LED, HIGH);
  delay(1000);
}

/* setup WIFI, MQTT, topics and subscribe callback function */
void iot_setup(){ 
  WiFi.begin(WIFISSID, PASSWORD);
  digitalWrite(Builtin_LED, HIGH); 
  Serial.print("Wait for WiFi...");
  timer = millis() + 3000;

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() > timer){
      ESP.restart();
    }
    Serial.print(".");
    delay(500);
  }
  Serial.println("WiFi Connected");
  blink_LED();
  
  sprintf(topic_pub, "%s%s", "/v1.6/devices/", DEVICE_LABEL);                       // publish topic /v1.6/devices/'DEVICE_LABEL'
  sprintf(topic_sub, "%s%s/%s/lv", "/v1.6/devices/", DEVICE_LABEL,VARIABLE_LABEL);  // subscribe topic /v1.6/devices/'DEVICE_LABEL'/'VARIABLE_LABEL'/lv  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback); 
}

/* setup payload in json format */
void convert_json(){
  dtostrf(sensor, 4, 2, str_sensor);                            
  sprintf(payload, "%s", "");                                   
  sprintf(payload, "{\"%s\":{\"value\": %s}}", VARIABLE_LABEL,str_sensor );   
}

/* set timer for next publish base on preset interval, minimum 10sec */
void set_timer(){
  if (interval < 9999){
    interval = 10000;
  }
  timer = millis() + interval;
}
