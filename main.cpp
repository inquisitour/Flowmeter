#define ROLE "WiFi"
#define LOCATION "M5"
#define VERSION "v1.0.1"

#include <Arduino.h>
#include "painlessMesh.h"
#include "FS.h"
#include <SPIFFS.h>
#include <ModbusRTU.h>
#include <Arduino_JSON.h>
#include "PubSubClient.h"
#include <SPI.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "RTClib.h"
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <AsyncElegantOTA.h>

const char* ssid = "Hetadatain_GF";
const char* password = "hetadatain@123";

const char* server = "50.62.22.142";


//AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);
ModbusRTU mb;
Modbus::ResultCode err;
Scheduler userScheduler; // to control your personal task
RTC_DS1307 RTC;

#define relayPin 5
#define MAX485_DE_RE 5
#define sendLed 26
#define connLed 26
#define BUTTON_PIN 0

#define LED_BUILTIN 2
#define SENSOR  35 

unsigned long currentMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 0.45;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
float DeltaT;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long counter ;
#define DS18B20PIN 4
#define DS18B20 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(DS18B20PIN);
OneWire oneWire1(DS18B20);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors11(&oneWire);
DallasTemperature sensors1(&oneWire1);


//#define MESH_PORT 5555 // Mesh Port should be same for all nodes in Mesh Network

//volatile bool need_restart = false;
//volatile int interrupts;

//variables
int nitya;
unsigned long TimeDifference;
float TR;
uint8_t mfd_read_pos = 0, sdQueue;
uint16_t hregs2[96];
float mfdValues[25];
uint32_t mfdVals[25];
int device_count;
uint16_t mfd_dev_id[10];
String id;
int rebootTime;
int pos;
uint32_t root;
unsigned long previousMillis = 0;
int mfd = 0;
int first_Reg, second_Reg;
int time_to_print;
bool MCP_Sent = false;
String msgMfd_payload[5];
String tempread;
uint32_t lastReconnectAttempt = 0;
xSemaphoreHandle xMutex;
unsigned long ts_epoch;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String bweh;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
   // bweh += (char)payload[i];
  }
 // Serial.println(bweh);
 // ts_epoch = bweh.toInt();
 // Serial.println(ts_epoch);
}

//String payload;
//int wdt = 0;

// User stub
//void total_counter();
void IRAM_ATTR pulseCounter();
void RTC_update();
void read_temp();
void read_flow_sensor();
void mfdConfig();
void updateTime();
void sendMFD();
void vApplicationIdleHook(void);
void writeTimeToCard();
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration);
bool dataStream(uint16_t address, uint16_t deviceId);
boolean read_Mfd_Task();
void updateTime();
void sendPayload(String &payload , String &tempread);
void saveToCard(String &payload, String &tempread);
void multi_mfd_read();
void convertMfdFloats();
void sendLog();
void schedulerUpdate(void *random);
void IRAM_ATTR onTime();
//void printDateTime(const RtcDateTime& dt);
//void checkAlive(void *bwehh);
//void meshUpdate(void *random);
/*
// ************************************
// ** RTC Setup
// ************************************
void RTC_Update(){
  // Do udp NTP lookup, epoch time is unix time - subtract the 30 extra yrs (946684800UL) library expects 2000
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime()-946684800UL;

  ts_epoch = timeClient.getEpochTime();

  Rtc.SetDateTime(epochTime);
  //Serial.println(ts_epoch);
}
bool RTC_Valid(){
  bool boolCheck = true;
  if (!Rtc.IsDateTimeValid()){
    Serial.println("RTC lost confidence in the DateTime!  Updating DateTime");
    boolCheck = false;
    RTC_Update();    
  }
  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now.  Updating Date Time");
    Rtc.SetIsRunning(true);
    boolCheck = false;
    RTC_Update();
  }
}

// Utility print function
#define countof(a) (sizeof(a) / sizeof(a[0]))
void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];
    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    //Serial.println(datestring);
}*/

void updateTime()
{
  //will update time && also watchdog
  //wdt++;
  ts_epoch++;
  rebootTime++;
  nitya++;

}

Task taskSendLog(TASK_SECOND * 8, TASK_FOREVER, &sendLog); // Set task second to send msg in a time interval
void sendLog()
{
  String logs;
  String topic = "test/lolFlow";
if(WiFi.isConnected()){
  SPIFFS.begin();
  File file = SPIFFS.open("/offlinelog.txt", "r"); // FILE_READ is default so not realy needed but if you like to use this technique for e.g. write you need FILE_WRITE

  if(!file)
  {
    Serial.println("No File Found ");
    taskSendLog.disable();
    pos = 0;
    return;
  }
  String buffer;
  uint8_t i = 0;
  //for (i = 0; i < 1; i++)
 // {
    file.seek(pos);
    buffer = file.readStringUntil('\n');
    Serial.println(buffer); //Printing for debugging purpose
    logs = buffer;
    if (buffer != "")
    {
      client.publish(topic.c_str(), logs.c_str());
     // Serial.println("logs:");
      Serial.println(logs);
      pos = file.position();
    }

    file.close();
    Serial.println("DONE Reading");
  //}

  if (buffer == "")
  {
    Serial.print("done dumping");
    if(SPIFFS.exists("/offlinelog.txt")){
    SPIFFS.remove("/offlinelog.txt");
    }
  }
  SPIFFS.end();
}
}
//Declarations for tasks scheduling
Task taskUpdateTime(TASK_SECOND * 1, TASK_FOREVER, &updateTime);        // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskReadMfd(TASK_MINUTE * 2, TASK_FOREVER, &read_Mfd_Task);        // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskMultiMfdRead(TASK_SECOND * 10, TASK_FOREVER, &multi_mfd_read); // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskReadFlowSensor(TASK_MINUTE * 2, TASK_FOREVER, &read_flow_sensor);
Task taskReadTemp(TASK_MINUTE * 2, TASK_FOREVER, &read_temp);
Task taskRTCupdate(TASK_MINUTE * 1, TASK_FOREVER, &RTC_update);
//Task taskTotalCounter(TASK_MINUTE * 2, TASK_FOREVER, &total_counter);
boolean mqttConnect() {
  Serial.print("Connecting to MQTT Server ");
  Serial.print(server);

  // Connect to MQTT Broker
  //boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
   boolean status = client.connect("WiFiMQTTClient", "username", "password");

  if (status == false) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" success");
  client.publish("test/lolFlow", " MQTT Connected through WiFi");

  client.subscribe("test/roflFlow");
  return client.connected();
}


void setup(void) {
  Serial.begin(115200);
  // sensors.begin();
  Wire.begin();
  RTC.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
 pinMode(SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  counter =0;
  totalMilliLitres = 0;
  previousMillis = 0;
  // Wait for connection
  /*
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }*/
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(server, 1883);
  client.setCallback(callback);

  Serial2.begin(9600, SERIAL_8E1);
  mb.begin(&Serial2);
  mb.master();

  SPIFFS.begin();

  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile)
  {
    Serial.println("Failed to open config file");
    //lcd.println("Failed to open config file");
  }
  size_t size = configFile.size();
  if (size > 1024)
  {
    Serial.println("Config file size is too large");
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<1024> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error)
  {
    Serial.println("Failed to parse config file");
  }
  const char *ID = doc["id"];

  const char *ROOT = doc["root"];
  const char *MFD = doc["mfd"];
  const char *DELAY = doc["delay"];

  Serial.print("Loaded id: ");

  id = ID;
 
  Serial.println(ID);
  // Serial.print("Loaded root id: ");
  root = 2137584946;
  Serial.println(ROOT);

   mfd = atoi(MFD);
  Serial.println(mfd);
  configFile.close();

  Serial.print("Loaded MCP pins: ");
  Serial.print(" loaded Send Delay: ");
  // maintain time in case of wdt reset
 /* File timeFile = SPIFFS.open("/time.txt", "r");
  if (timeFile)
  {
    String timeTemp = timeFile.readStringUntil('\n');
    ts_epoch = timeTemp.toInt();
    timeFile.close();
  }
  if (SPIFFS.exists("/pos.txt"))
  {
    File posFile = SPIFFS.open("/pos.txt", "r");
    String timeTemp = timeFile.readStringUntil('\n');
    pos = timeTemp.toInt();

    posFile.close();
  }
  Serial.println("position");
  Serial.print(pos);*/
  mfdConfig();
  mqttConnect();
  read_temp();
  //start the mesh
  //declarations for scheduler UwU
  userScheduler.addTask(taskMultiMfdRead);
 
  userScheduler.addTask(taskUpdateTime);
  taskUpdateTime.enable();
  userScheduler.addTask(taskReadMfd);
  taskReadMfd.enable();
  userScheduler.addTask(taskSendLog);
  taskSendLog.enable();
 userScheduler.addTask(taskReadFlowSensor);
 taskReadFlowSensor.enable();
 userScheduler.addTask(taskReadTemp);
 taskReadTemp.enable();
 userScheduler.addTask(taskRTCupdate);
 taskRTCupdate.enable();
 //userScheduler.addTask(taskTotalCounter);
 //taskTotalCounter.enable();
  xTaskCreatePinnedToCore(schedulerUpdate, "schedulerUpdate", 32000, NULL, 2, NULL, 1);

 //userScheduler.addTask(taskReadCounter);
 //taskReadCounter.enable();
  SPIFFS.end();
  pinMode(connLed, OUTPUT);

  /*
  // Start NTP Time Client
  timeClient.begin();
  delay(2000);
  timeClient.update();
  Rtc.Begin();
  RTC_Update();
*/

  /*server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");*/
}


void schedulerUpdate(void *random)
{

  for (;;)
  {
    userScheduler.execute();
    vTaskDelay(10/ portTICK_RATE_MS);  delay(10);
  }
}


/*
void checkAlive(void *bwehh)
{

  for (;;)
  {
    client.publish("test/lol", "ready");
    vTaskDelay(120 / portTICK_RATE_MS);
  }
}*/


void loop(void) {
   // it will run the scheduler
 // userScheduler.execute();
   read_flow_sensor();
  //userScheduler.setHighPriorityScheduler();
  /*if (!RTC_Valid()){
    RTC_Update();
  }
  RtcDateTime currTime = Rtc.GetDateTime();
  printDateTime(currTime);
 // Serial.println(timeClient.getFormattedTime());
  delay(10000);*/

if (!client.connected())
    {
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {                      // && !client.connected()
      Serial.println("=== MQTT NOT CONNECTED ===");
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
    }
   
if(rebootTime > 3600){
  writeTimeToCard();
  ESP.restart();
}
 //AsyncElegantOTA.loop();
 //Serial.flush();
 client.loop();
}

void writeTimeToCard()
{
  SPIFFS.begin();
  SPIFFS.remove("/time.txt");
  File dataFile = SPIFFS.open("/time.txt", "w");
  if (dataFile)
  {
    dataFile.println(ts_epoch);
    dataFile.close();
  }
  else
  {
    Serial.println("error opening time.txt");
  }
  SPIFFS.end();
}

boolean read_Mfd_Task()
{
  MCP_Sent = false;
  // userScheduler.disableAll();
  taskUpdateTime.enableIfNot();
  // vTaskSuspend(meshTaskHandle_t);
  time_to_print = ts_epoch;
  taskMultiMfdRead.enable();
  // xTaskCreate(multi_mfd_read,"readsMFD",20000,NULL,2,&mfdTaskHandle_t);
  return true;
}

bool resCallback(Modbus::ResultCode event, uint16_t, void *)
{
  err = event;
}

Modbus::ResultCode readSync(uint16_t Address, uint16_t start, uint16_t num, uint16_t *buf)
{
  // xSemaphoreTake(xMutex, portMAX_DELAY);
  if (mb.slave())
  {
    // xSemaphoreGive(xMutex);
    return Modbus::EX_GENERAL_FAILURE;
  }
  Serial.printf("SlaveID: %d Hreg %d\n", Address, start);
  mb.readHreg(Address, start, buf, num, resCallback);
  while (mb.slave())
  {
    mb.task();
  }
  Modbus::ResultCode res = err;
  // xSemaphoreGive(xMutex);
  return res;
}
bool dataStream(uint16_t address, uint16_t deviceId)
{

  if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
  {
    Serial.println("OK 2");
    convertMfdFloats();
    return true;
  }
  else
  {
    Serial.print("Error trying again ! ");

    // dataStream(address,deviceId);

    if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
    {
      Serial.println("OK 2");
      convertMfdFloats();
      return true;
    }
    else
    {
      uint8_t j = 0;
      // String msgMfd;
      for (uint8_t i = 0; i <= 48 ; i++)
      {
        mfdValues[i] = 0;
      }

      return false;
    }
  }
}

void convertMfdFloats()
{

  uint8_t j = 0;
  // String msgMfd;
  for (uint8_t i = 0; i <= 48; i++)
  {

    uint16_t temp1[2] = {hregs2[i], hregs2[i + 1]};
    memcpy(&mfdValues[j], temp1, 32); i++;j++;
  }
}
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration)
{
  String msgMfd;
    DateTime now = RTC.now();

  ts_epoch = now.unixtime();
  Serial.println(ts_epoch);
  time_to_print = ts_epoch;

  if (dataStream(address, devId))
  {
    if (iteration == 1 ){
    msgMfd += time_to_print;
    msgMfd.concat(",");
    msgMfd.concat(id);
    msgMfd.concat(",");
    msgMfd.concat(devId);
    msgMfd.concat(",");
    msgMfd.concat("2");
    //msgMfd.concat(",");
    //msgMfd.concat(iteration);
    msgMfd.concat(",");}

    for (uint8_t i = 0; i <= 24; i++)
    {

      msgMfd.concat(mfdValues[i]);
      if (i <= 24)
      {
        msgMfd.concat(",");
      }
    }
    delay(500);
    String msgToSend = msgMfd;

    return msgToSend;
  }
  else{
    delay(500);
    // mfd_read_pos++;
    // taskMultiMfdRead.restart();
    if(iteration == 1 ){
    msgMfd += time_to_print;
    msgMfd.concat(",");
    msgMfd.concat(id);
    msgMfd.concat(",");
    msgMfd.concat(devId);
    msgMfd.concat(",");
    msgMfd.concat("2");
    msgMfd.concat(",");
    msgMfd.concat(iteration);
    msgMfd.concat(",");
}
    //msgMfd.concat(",");

    for (uint8_t i = 0; i <= 24; i++)
    {
      msgMfd.concat(mfdValues[i]);
      if (i <= 24)
      {
        msgMfd.concat(",");
      }
    }
    return msgMfd;
  }
}
void multi_mfd_read()
{

  time_to_print++; //set the time for mfd data to be in sync
  Serial2.end();
  Serial2.begin(9600, SERIAL_8E1);

  msgMfd_payload[0] = readMfd(mfd_dev_id[mfd_read_pos], 100, 1);
  vTaskDelay(100 / portTICK_RATE_MS);
  msgMfd_payload[1] = readMfd(mfd_dev_id[mfd_read_pos], 148, 2);
  //msgMfd_payload[2] = readMfd(mfd_dev_id[mfd_read_pos], 148, 3);
  //msgMfd_payload[3] = readMfd(mfd_dev_id[mfd_read_pos],172,4);
  //msgMfd_payload[4] = readMfd(mfd_dev_id[mfd_read_pos],196,5);

  delay(10);
  sendMFD();
  Serial2.flush();
  mfd_read_pos++;
  if (mfd_read_pos >= device_count)
  {
    Serial2.end();
    mfd_read_pos = 0;
    taskMultiMfdRead.disable();
  }

}

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

 void read_flow_sensor()
 {  
   
   // currentMillis = millis();
   // if(currentMillis-previousMillis > interval){

  if (nitya == 120) {
    //Serial.println("current:");
    //Serial.print(currentMillis);
    pulse1Sec = pulseCount * nitya;
  
    //Serial.print("previous:");
    //Serial.println(previousMillis);
    TimeDifference = nitya;
    Serial.println("Interval in seconds");
    Serial.println(TimeDifference);
    
    flowRate = (1000.0 / pulse1Sec) / calibrationFactor;
    
    //previousMillis = millis();
    
   
    flowMilliLitres = (flowRate / 60) * 1000;
    //flowMilliLitres = pulse1Sec ;
    Serial.println("pulserate:");
    Serial.println(pulse1Sec);
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    
    // Print the fFlow rate for this second in litres / minute
    Serial.println("Flow rate: ");
    Serial.println(flowMilliLitres,DEC);  // Print the integer part of the variable
    Serial.print("mL/Sec");

    // Print the cumulative total of litres flowed since starting
    Serial.println("Output Liquid Quantity: ");
    Serial.println(totalMilliLitres); 
    Serial.print("mL / ");
    Serial.println(totalMilliLitres / 1000);
    Serial.print("L");

    nitya = 0;
    pulseCount = 0;
  }
 /*
 int8_t i ;
  counter = 0 ; 
  for (i=0 ; i < 5 ; i++){

   counter +=pulse1Sec;
     Serial.println("counter:");
    delay(1000);

  Serial.println(counter);
 } */
      
  }
 // }
  
  
 
 void read_temp (){

  sensors11.requestTemperatures(); 
  float tempC = sensors11.getTempCByIndex(0);
  float tempF = sensors11.getTempFByIndex(0);
  //Serial.flush();
  Serial.println("temp1:");
  Serial.print(tempC);
  Serial.print("ºC");
  Serial.print(tempF);
  Serial.print("ºF");
  Serial.println("...........");
  delay(500);

  sensors1.requestTemperatures(); 
  float temperatureCc = sensors1.getTempCByIndex(0);
  float temperatureFf = sensors1.getTempFByIndex(0);
  Serial.println("temp2:");
  Serial.print(temperatureCc);
  Serial.print("ºC");
  Serial.println("...........");
  Serial.println(temperatureFf);
  Serial.print("ºF");
  Serial.println("...........");
  delay(500);

  DeltaT= tempC-temperatureCc;
  TR = 0.33 * 0.060 * flowMilliLitres * DeltaT;
  String tempread;

  time_to_print = ts_epoch;

  tempread += time_to_print;
  tempread.concat(",");
  tempread.concat(id);
  tempread.concat(",");
  tempread.concat(tempC);
  tempread.concat(",");
  tempread.concat(temperatureCc);
  tempread.concat(",");
  tempread.concat(DeltaT);
  tempread.concat(",");
  tempread.concat(TR);
  tempread.concat(",");
  //tempread.concat(millis());
  //tempread.concat(",");
  //tempread.concat(previousMillis);
  //tempread.concat(",");
  tempread.concat(TimeDifference);
  //tempread.concat(",");
  //tempread.concat(counter);
  tempread.concat(",");
  tempread.concat(flowMilliLitres);
  Serial.println(tempread);


  client.publish("test/lolFlow", tempread.c_str());
  taskSendLog.enable();

 }


void sendMFD()
{ String tempread;
   String payload = msgMfd_payload[0] + msgMfd_payload[1] ;
   //for (uint8_t i = 0; i < 5; i++)
  //{
    sendPayload(payload, tempread);
    //vTaskDelay(300 / portTICK_RATE_MS);
    taskSendLog.enable();
 // }

}

//writing data to card
void saveToCard(String &payload, String &tempread)
{
SPIFFS.begin();
if(!WiFi.isConnected()){
  File dataFile = SPIFFS.open("/offlinelog.txt", "a");
  Serial.println("current data size : ");
  Serial.println(dataFile.size());
  dataFile.println(payload);
  Serial.println("mfd:");
  Serial.println(payload);
  dataFile.println(tempread);
  Serial.println("temp:");

  Serial.println(tempread);

  Serial.print("payload saved");
  dataFile.close();
  }
  SPIFFS.end();
}
//sending data to server
void sendPayload(String &payload , String &tempread)
{
if(WiFi.isConnected()){
  client.publish("test/lolFlow", payload.c_str());
  client.publish("test/lolFlow", tempread.c_str());

  Serial.println(payload);
  Serial.println(tempread);

  }
else{
  saveToCard(payload, tempread);
}
}

void mfdConfig()
{

  SPIFFS.begin();

  File configFile = SPIFFS.open("/mfdConf.json", "r");
  if (!configFile)
  {
    Serial.println("Failed to open config file");
  }
  size_t size = configFile.size();
  if (size > 1024)
  {
    Serial.println("Config file size is too large");
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<1024> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error)
  {
    Serial.println("Failed to parse config file");
  }
  const char *DEV_COUNT = doc["device_count"];
  device_count = atoi(DEV_COUNT);
  Serial.println(device_count);

  const char *MFD_SERIAL_ID_1 = doc["mfd_dev_id_1"];
  mfd_dev_id[0] = atoi(MFD_SERIAL_ID_1);
  Serial.println(mfd_dev_id[0]);

  const char *MFD_SERIAL_ID_2 = doc["mfd_dev_id_2"];
  mfd_dev_id[1] = atoi(MFD_SERIAL_ID_2);
  Serial.println(mfd_dev_id[1]);

  const char *MFD_SERIAL_ID_3 = doc["mfd_dev_id_3"];
  mfd_dev_id[2] = atoi(MFD_SERIAL_ID_3);
  Serial.println(mfd_dev_id[2]);

  const char *MFD_SERIAL_ID_4 = doc["mfd_dev_id_4"];
  mfd_dev_id[3] = atoi(MFD_SERIAL_ID_4);
  Serial.println(mfd_dev_id[3]);

  const char *MFD_SERIAL_ID_5 = doc["mfd_dev_id_5"];
  mfd_dev_id[4] = atoi(MFD_SERIAL_ID_5);
  Serial.println(mfd_dev_id[4]);

  const char *MFD_SERIAL_ID_6 = doc["mfd_dev_id_6"];
  mfd_dev_id[5] = atoi(MFD_SERIAL_ID_6);
  Serial.println(mfd_dev_id[5]);

  const char *MFD_SERIAL_ID_7 = doc["mfd_dev_id_7"];
  mfd_dev_id[6] = atoi(MFD_SERIAL_ID_7);
  Serial.println(mfd_dev_id[6]);

  const char *MFD_SERIAL_ID_8 = doc["mfd_dev_id_8"];
  mfd_dev_id[7] = atoi(MFD_SERIAL_ID_8);
  Serial.println(mfd_dev_id[7]);
}


void IRAM_ATTR onTime() {

    taskReadMfd.forceNextIteration();
   
} 


 void RTC_update () {
 // char row[128];
  DateTime now = RTC.now();
  Serial.print("timestamp=");
  Serial.println(now.unixtime());
  ts_epoch = now.unixtime();
  Serial.println(ts_epoch);
  //snprintf(row,sizeof(row),"printf=%lu",now.unixtime());
  //Serial.println(row);
  

}/*
 void total_counter(){
 uint8_t i ;
  counter = 0 ; 
  for (i=0 ; i < 120 ; i++){

   counter +=pulse1Sec;
     Serial.println("counter:");
    delay(1000);

  Serial.println(counter);
 } 
 }*/