#include <SoftwareSerial.h>
#include <SerialCommand.h>

#include <esp_now.h>
#include <WiFi.h>

#define led1 4
#define led2 2
#define pinPing 5


//CEU
SerialCommand sCmd;
void pingHandler();
////////////////////////////////////////////////

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xCC, 0xE1, 0x64};

// Define variables to store BME280 readings to be sent
float distance;


// Define variables to store incoming readings
float incomingDist;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float dist;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message SHarpIRReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingDist = incomingReadings.dist;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(pinPing, OUTPUT);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  while (!Serial);
  sCmd.addCommand("PING", pingHandler);
  //sCmd.addCommand("Capt",CaptorHandler);
    /*sCmd.addCommand("BACKWARD", backwardHandler);
    sCmd.addCommand("FORWARD", forwardHandler);*/
}
 
void loop() {
  if (Serial.available() > 0)
    sCmd.readSerial(); //Read strings from serial port and invoke right handler
}


void CaptorHandler(){
if (incomingDist < 12.0f ) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
    } 
 
    if (incomingDist > 13.0f ) {
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
  }

}
void pingHandler()
{
    CaptorHandler();
    Serial.println("PONG");  
    Serial.println(incomingDist);
    digitalWrite(pinPing, HIGH);
   
  
}
void errorHandler (const char *command)
{
    Serial.println("WRONG PONG");
    /*digitalWrite(pinError, HIGH);
    delay(1000);
    digitalWrite(pinError, LOW);*/
}

void echoHandler () {
  char *arg;
  arg = sCmd.next();
  if (arg != NULL)
    Serial.println(arg);
  else
    Serial.println("nothing to echo");
}




