#include <Arduino.h>
#include <ArduinoJson.h>
#include "sdkconfig.h"
#include "DHT.h"
#include "pinName-board.h"
#include "config.h"
// Load Wi-Fi and OTA library
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>

TaskHandle_t TaskMeasure_h;
TaskHandle_t TaskServer_h;
// Network credentials
const char* ssid = "Dry_Age";
const char* password = "Tamblay31415";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");

DHT dht(DHTPIN, DHTTYPE);
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 1, 0, 0.2, DIRECT);

// Tasks functions
void TaskServer( void * pvParameters );
void TaskMeasure( void * pvParameters );


void setup() {
    Serial.begin(115200);
    Serial.println("DHTxx test!");
    xTaskCreatePinnedToCore(
        TaskServer,   /* Task function. */
        "TaskServer",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        0,           /* priority of the task */
        &TaskServer_h,      /* Task handle to keep track of created task */
        1);          /* pin task to core 1 */
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreatePinnedToCore(
        TaskMeasure,   /* Task function. */
        "TaskMeasure",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task (high priority) */
        &TaskMeasure_h,      /* Task handle to keep track of created task */
        0);          /* pin task to core 0 */
    vTaskDelay(pdMS_TO_TICKS(500));
}

void TaskServer( void * pvParameters ){
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    events.onConnect([](AsyncEventSourceClient *client){
    client->send("", "connection", millis(), 500);
    });
    server.addHandler(&events);
    server.begin();
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskMeasure( void * pvParameters ){
    int WindowSize = 60000;
    unsigned long windowStartTime, lastRead;
    lastRead = millis()-11000;
    windowStartTime = millis();
    Setpoint = 10;
    myPID.SetOutputLimits(0, WindowSize);
    myPID.SetMode(AUTOMATIC);
    DynamicJsonDocument output(1024);
    pinMode(IN1, OUTPUT);
    digitalWrite(IN1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    dht.begin();
    float prev_t = 20.0;
    float prev_h = 60.0;
    float h = 60.0;
    float t = 20.0;
    bool on = true;
    Serial.println("Init complete");
    for(;;)
    {
        unsigned long now = millis();
        if(now - lastRead > 10000){
            h = dht.readHumidity();
            t = dht.readTemperature();
            if (isnan(h) || isnan(t)) {
                Serial.println("Failed to read from DHT sensor!");
                h = prev_h;
                t = prev_t;
            }
            else{
                prev_h = h;
                prev_t = t;
                output["temp"] = t;
                output["hr"] = h;
                output["time"] = millis();
                output["on"] = on;
                output["output"] = Output;
                String to_server;
                serializeJson(output, to_server);
                events.send(to_server.c_str(),"status",millis()); // Envia notificacion push
                Serial.println(to_server);
            }
            lastRead = millis();
        }
        Input = t;
        myPID.Compute();
        Serial.println(Output);
        if (now - windowStartTime > WindowSize)
        { //time to shift the Relay Window
            windowStartTime += WindowSize;
        }
        if (Output > now - windowStartTime){
            digitalWrite(IN1, HIGH);
            on = false;
        }
        else{
            digitalWrite(IN1, LOW);
            on = true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
