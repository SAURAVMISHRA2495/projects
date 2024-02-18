/*

The Can Bus (a.k.a Two-Wire Automobile Interface, TWAI on ESP32) Sender, Receiver,Serial Writer on EPS32 S3.

*/

#include <driver/twai.h>
#include <WiFi.h>
#include <PubSubClient.h>  
#include <esp_wifi.h>  
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
#include <Arduino_GFX_Library.h>


// GPIOs utilized to connect to the CAN bus transceived
#define RX_GPIO_PIN               GPIO_NUM_4
#define TX_GPIO_PIN               GPIO_NUM_5
#define TFT_SCK    18
#define TFT_MOSI   23
#define TFT_MISO   19
#define TFT_CS     22
#define TFT_DC     21
#define TFT_RESET  17
/*Variable battery */
float Bat_Temp;
float Bat_Vtg;
float Bat_Crnt;
float Bat_Lvl;
// Interval
#define DEFULT_INTERVAL           1000
// Set the custom MAC address in case your ESP32 is not regsitered with the acts network - wifi spoofing
uint8_t newMACAddress[] = {0xf4, 0x96, 0x34, 0x9d, 0xe1, 0xdf};  // a8:6d:aa:0e:61:f9  //f4:96:34:9d:e1:df
DynamicJsonDocument sensor_data_payload(1024);

char sensor_data_format_for_mqtt_publish[1024];

const char* ssid =   "Gk@120";                          //ssid - service set Identifier (Replace it with your ssid name)

const char* password =  "999999999";                     // replace with ssid paasword

const char* mqttBroker = "demo.thingsboard.io";                  // broker address - replace it with your broker address/cloud broker - test.mosquitto.org192.168.43.75

const int   mqttPort = 1883;                            // broker port number

const char* clientID = "a2259230-ca5b-11ee-bac2-0f1d08d252b5";                   // client-id --> replace it in case willing to connect with same broker

const char* username = "Batteryparametrs123";// access token
const char* mqtt_topic_for_publish = "v1/devices/me/telemetry"; // topic namescdac/room/data

WiFiClient MQTTclient;

PubSubClient client(MQTTclient);
boolean reconnect()
{
  //boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession])
  if (client.connect(clientID,username,password)){

    Serial.println("Attempting to connect broker");
    
  }
  return client.connected();
}
long lastReconnectAttempt = 0;

static bool driver_installed      = false;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup1();

  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_PIN, RX_GPIO_PIN, TWAI_MODE_LISTEN_ONLY);

  general_config.tx_queue_len = 0;
  general_config.rx_queue_len = 1000;

  // Depends on the baud rate of can bus on the vehicle
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();

  // Transparent transmission
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install the TWAI driver
  if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK)
  {
    Serial.println("Driver installed");

    // Start the TWAI driver
    if (twai_start() == ESP_OK)
    {
      Serial.println("Driver started");

      // Reconfigure the alerts to detect the error of frames received, Bus-Off error and RX queue full error
      /*
      TWAI_ALERT_RX_DATA        0x00000004    Alert(4)    : A frame has been received and added to the RX queue
      TWAI_ALERT_ERR_PASS       0x00001000    Alert(4096) : TWAI controller has become error passive
      TWAI_ALERT_BUS_ERROR      0x00000200    Alert(512)  : A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
      TWAI_ALERT_RX_QUEUE_FULL  0x00000800    Alert(2048) : The RX queue is full causing a frame to be lost
      */
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      // uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;

      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
      {
        Serial.println("CAN alerts reconfigured");

        // TWAI driver is installed and started
        driver_installed = true;
      }
      else
        Serial.println("Failed to reconfigure alerts");
    }
    else
      Serial.println("Failed to start driver");
  }
  else
   Serial.println("Failed to install driver");
   
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (driver_installed)
  {
    uint32_t alerts_triggered;
    twai_status_info_t status_info;

    // Check if alert triggered
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(DEFULT_INTERVAL));
    twai_get_status_info(&status_info);

    // Handle the alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
      Serial.println("Alert: TWAI controller has become error passive.");
    else if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %d\n", status_info.bus_error_count);
    }
    else if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
    {
      Serial.println("Alert: the RX queue is full causing received frames to be lost.");
      Serial.printf("RX buffered: %d\t", status_info.msgs_to_rx);
      Serial.printf("RX miussed: %d\n", status_info.rx_missed_count);
      Serial.printf("RX overrun %d\n", status_info.rx_overrun_count);
    }
    else if (alerts_triggered & TWAI_ALERT_RX_DATA)
    {
       // New message(s) received
       // One or more message received, handle all
       twai_message_t message;

       while (twai_receive(&message, 0) == ESP_OK)
       {
        handle_rx_message(message); 
        loop1(message);
       }    
    }
  }
  
}

void setup1() {

  Serial.begin(115200);//baud rate bits per sec
  Serial.println("Attempting to connect...");
  WiFi.mode(WIFI_STA);      
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]); // for wifi spoofing
  WiFi.begin(ssid, password); // Connect to WiFi.

  
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Couldn't connect to WiFi.");
  }
  Serial.print("ESP32 IP ADDRESS : ");
  Serial.println(WiFi.localIP());
  //Add details for MQTT Broker
  client.setServer(mqttBroker, mqttPort); // Connect to broker
  lastReconnectAttempt = 0;
   /*TFT Display Initialization */
  // Arduino_ESP32SPI bus = Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
  // Arduino_ILI9341 display = Arduino_ILI9341(&bus, TFT_RESET);
  // display.begin();
  // display.fillScreen(WHITE);
  // display.setCursor(20, 20);
  // display.setTextSize(2);
  // display.setTextColor(BLUE);
  // display.print(" WELCOME TO SRBMS \n");
  // display.print("\n Bat_Temp=");
  // display.print(String(Bat_Temp));
  // display.print("\n");
  // display.print("\n Bat_Vtg=");
  // display.print(String(Bat_Vtg));
  // display.print("\n");
  // display.print("\n Bat_Crnt=");
  // display.print(String(Bat_Crnt));
  // display.print("\n");
}
void loop1(twai_message_t message) {

 Arduino_ESP32SPI bus = Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
  Arduino_ILI9341 display = Arduino_ILI9341(&bus, TFT_RESET);
  display.begin();
  display.fillScreen(WHITE);
  display.setCursor(20, 20);
  display.setTextSize(2);
  display.setTextColor(BLUE);
  if (!client.connected()) //if device is not connected to broker
  {
    long now = millis();  // Returns the number of milliseconds passed since the Arduino board began running the current program
    if (now - lastReconnectAttempt > 5000) { // Try to reconnect.
      lastReconnectAttempt = now;
      if (reconnect())
      { 
        lastReconnectAttempt = 0;

      }
 
    }

  }
  else 
  { 
    Serial.println("Connected to Broker --- !!");
    client.loop();
    sensor_data_payload["Bat_Temp"] = message.data[0];
    sensor_data_payload["Bat_Vtg"] = message.data[1];
    sensor_data_payload["Bat_Crnt"] = message.data[2];
    sensor_data_payload["Bat_Levta"] = message.data[3]; 
     Bat_Temp= message.data[0];
     Bat_Vtg= message.data[1];
     Bat_Crnt= message.data[2];
     Bat_Lvl= message.data[3];

    serializeJson(sensor_data_payload, sensor_data_format_for_mqtt_publish);   
    delay(200);  
    client.publish(mqtt_topic_for_publish,sensor_data_format_for_mqtt_publish);  //(topicname, payload)
    Serial.println("sensor data sent to broker");
    /********************************************************************************************************/ 
  // Arduino_ESP32SPI bus = Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
  // Arduino_ILI9341 display = Arduino_ILI9341(&bus, TFT_RESET);
  // display.begin();
  // display.fillScreen(WHITE);
  // display.setCursor(20, 20);
  // display.setTextSize(2);
  // display.setTextColor(BLUE);
 // display.fillRect(20, 40, 300, 80, WHITE);
  display.print(" WELCOME TO SRBMS \n\n");
  display.print("\n  Bat_Temp=");
  display.print(String(Bat_Temp));
  display.print("\n\n");
  display.print("\n  Bat_Vtg=");
  display.print(String(Bat_Vtg));
  display.print("\n\n");
  display.print("\n  Bat_Crnt=");
  display.print(String(Bat_Crnt));
  display.print("\n");
  delay(3000);
  delay(3000);

  }
}


