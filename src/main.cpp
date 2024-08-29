#include <Arduino.h>
#include <ArduinoJson.h>

#include <LittleFS.h>
// #include <WiFiUdp.h>

#include <TaskScheduler.h>

// #include <DNSClient.h>


#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESPAsyncUDP.h>
#elif ESP32
#include <AsyncUDP.h>
#define LED_BUILTIN 4
#endif

// #include <pins_arduino.h>
// #include "gpioMng.hpp"

#include "config.hpp"
#include "packet.hpp"
#include "tonkey.hpp"

// #include "esfos_drv.hpp"

/*
양궁장 패드 전용 버전
센서 타운팅 방식 (T1D1)
*/
AsyncUDP udp;

const byte version = 200;
const String firmWareType = "T1D1";
const u32 system_Header = 20200519;

uint16_t bc_port = 7204;
uint16_t udp_port_data = 8284;

const int sensor_pin = 14; // D5;
// const int relay_pin = D6; //replay control
const int relay_pin = 5; // D1;

u16 g_nFire_count = 0;

int nFSM = 0;
u32 g_chipId;
String g_chipIdStr = "egcs_";


int deviceIndex = -1;
uint32_t trigger_delay = 150;

volatile boolean enableTriger = true;
boolean connectioWasAlive = false;

IPAddress g_remoteIp;
uint16_t g_remotePort = 0;

IRAM_ATTR void detect_sensor()
{
    if (enableTriger)
    {
        enableTriger = false;
    }
}

static byte packetBuffer[1024];

void udp_loop_res()
{
}

// Ticker g_tickerUdpReq;
// Ticker g_tickerUdpRes;

// void dumpConfig()
// {
//     File f = LittleFS.open(configFileName, "r");
//     String line = f.readString();
//     Serial.println(line);
//     f.close();
// }

Scheduler runner;
tonkey g_MainParser;

Config g_config;

String ParseCmd(String _strLine)
{
    g_MainParser.parse(_strLine);

    if (g_MainParser.getTokenCount() > 0)
    {
        String cmd = g_MainParser.getToken(0);
        JsonDocument _res_doc;
        if (cmd == "about")
        {
            /* code */
            _res_doc["result"] = "ok";
            _res_doc["os"] = "cronos-v1";
            _res_doc["app"] = "egcsUnit_v2";
            _res_doc["version"] = "A2_" + String(g_config.version);
            _res_doc["author"] = "gbox3d";
// esp8266 chip id
#ifdef ESP8266
            _res_doc["chipid"] = ESP.getChipId();
#elif ESP32
            _res_doc["chipid"] = ESP.getEfuseMac();
#endif
        }
        else if (cmd == "reboot")
        {
#ifdef ESP8266
            ESP.reset();
#elif ESP32
            ESP.restart();
#endif
        }
        else if (cmd == "config")
        {
            if (g_MainParser.getTokenCount() > 1)
            {
                String subCmd = g_MainParser.getToken(1);
                if (subCmd == "load")
                {
                    g_config.load();
                    _res_doc["result"] = "ok";
                    _res_doc["ms"] = "config loaded";
                }
                else if (subCmd == "save")
                {
                    g_config.save();
                    _res_doc["result"] = "ok";
                    _res_doc["ms"] = "config saved";
                }
                else if (subCmd == "dump")
                {

                    // parse json g_config.dump()
                    String jsonStr = g_config.dump();
                    DeserializationError error = deserializeJson(_res_doc["ms"], jsonStr);
                    if (error)
                    {
                        // Serial.print(F("deserializeJson() failed: "));
                        // Serial.println(error.f_str());
                        // return;
                        _res_doc["result"] = "fail";
                        _res_doc["ms"] = "json parse error";
                    }
                    else
                    {
                        _res_doc["result"] = "ok";
                    }
                }
                else if (subCmd == "clear")
                {
                    g_config.clear();
                    _res_doc["result"] = "ok";
                    _res_doc["ms"] = "config cleared";
                }
                else if (subCmd == "set")
                {
                    if (g_MainParser.getTokenCount() > 2)
                    {
                        String key = g_MainParser.getToken(2);
                        String value = g_MainParser.getToken(3);
                        g_config.set(key.c_str(), value);
                        _res_doc["result"] = "ok";
                        _res_doc["ms"] = "config set";
                    }
                    else
                    {
                        _res_doc["result"] = "fail";
                        _res_doc["ms"] = "need key and value";
                    }
                }
                else if (subCmd == "setA")
                { // set json array
                    if (g_MainParser.getTokenCount() > 2)
                    {
                        String key = g_MainParser.getToken(2);
                        String value = g_MainParser.getToken(3);
                        // parse json value
                        //  JSON 문자열 파싱을 위한 임시 객체
                        JsonDocument tempDoc; // 임시 JSON 문서

                        // JSON 문자열 파싱
                        DeserializationError error = deserializeJson(tempDoc, value);
                        // DeserializationError error = deserializeJson(g_config[key.c_str()], value);
                        if (error)
                        {
                            // Serial.print(F("deserializeJson() failed: "));
                            // Serial.println(error.f_str());
                            // return;
                            _res_doc["result"] = "fail";
                            _res_doc["ms"] = "json parse error";
                        }
                        else
                        {
                            // JsonArray array = tempDoc[key].as<JsonArray>();

                            g_config.set(key.c_str(), tempDoc);
                            _res_doc["result"] = "ok";
                            _res_doc["ms"] = tempDoc;
                        }
                        // g_config.set(key.c_str(), value);
                        // _res_doc["result"] = "ok";
                        // _res_doc["ms"] = "config set";
                    }
                    else
                    {
                        _res_doc["result"] = "fail";
                        _res_doc["ms"] = "need key and value";
                    }
                }
                else if (subCmd == "get")
                {
                    if (g_MainParser.getTokenCount() > 2)
                    {
                        String key = g_MainParser.getToken(2);

                        // check key exist
                        if (!g_config.hasKey(key.c_str()))
                        {
                            _res_doc["result"] = "fail";
                            _res_doc["ms"] = "key not exist";
                            // serializeJson(_res_doc, Serial);
                            // Serial.println();
                            // return;
                        }
                        else
                        {
                            _res_doc["result"] = "ok";
                            _res_doc["value"] = g_config.get<String>(key.c_str());
                        }
                    }
                    else
                    {
                        _res_doc["result"] = "fail";
                        _res_doc["ms"] = "need key";
                    }
                }
                else
                {
                    _res_doc["result"] = "fail";
                    _res_doc["ms"] = "unknown sub command";
                }
            }
            else
            {
                _res_doc["result"] = "fail";
                _res_doc["ms"] = "need sub command";
            }
        }
        else
        {
            _res_doc["result"] = "fail";
            _res_doc["ms"] = "unknown command " + _strLine;
        }

        // serializeJson(_res_doc, Serial);
        // Serial.println();

        return _res_doc.as<String>();
        // String
    }
    else
    {
        return "";
    }
}

Task task_Cmd(100, TASK_FOREVER, []()
              {
    if (Serial.available() > 0)
    {
        String _strLine = Serial.readStringUntil('\n');

        _strLine.trim();

        Serial.println(ParseCmd(_strLine));
    } });

// Task task_udp(100, TASK_FOREVER, []()
//               { udp_loop_res(); });

// Task task_req(100, TASK_FOREVER, []()
//               { udp_loop_req(); });

Task task_req(5000, TASK_FOREVER, []()
{
    S_REQ_PACKET _req;
    _req.header = system_Header;
    _req.chipId = g_chipId;
    _req.index = deviceIndex;
    _req.code = 0x01;
    _req.status = nFSM;
    _req.pkt_size = sizeof(S_REQ_PACKET);
    _req.count = g_nFire_count;
    _req.extra[0] = version + g_config.version;
    _req.extra[1] = WiFi.localIP()[3]; // last ip address

    // Serial.printf("send req : %d\n", g_nFire_count);

    // Serial.printf("send req %s : %d \n",g_remoteIp.toString().c_str(),g_remotePort);

    udp.writeTo((byte *)&_req, sizeof(S_REQ_PACKET), g_remoteIp, g_remotePort);
    
});

Task task_Blink(150, TASK_FOREVER, []()
                { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); });

// udp broadcast task
Task task_Broadcast(3000, TASK_FOREVER, []()
                    {

  if (WiFi.status() == WL_CONNECTED) {
    JsonDocument broadcastDoc;
    broadcastDoc["chipid"] = g_chipIdStr;
    broadcastDoc["type"] = "broadcast";
    
    String broadcastMessage;
    serializeJson(broadcastDoc, broadcastMessage);

    udp.broadcastTo(broadcastMessage.c_str(), bc_port);

  } });


bool dnsResolved = false;
String remoteHostName;


WiFiEventHandler staConnectedHandler;
WiFiEventHandler staGotIPHandler;
WiFiEventHandler staDisconnectedHandler;

// UDP 수신 콜백 함수
void onPacketReceived(AsyncUDPPacket packet)
{
    if(packet.isBroadcast()) {
     
    }
    else if(packet.isMulticast()) {
        
    }
    else {
        S_RES_PACKET *pResPkt;

    // S_RES_PACKET temp;

    //함부러 타입캐스팅 하면 분제 발생...
    pResPkt = (S_RES_PACKET *)packet.data();  

    // g_remoteIp = packet.remoteIP();
    // g_remotePort = packet.remotePort();

    // 응답처리
    // temp = *(S_RES_PACKET *)packet.data();
    // Serial.printf("From %s, packet size : %d\n", packet.remoteIP().toString().c_str(), packet.length());

        if (packet.length() > 0)
        {
            byte _code = pResPkt->code;

            if (_code == 0x11)
            { // ready to fire
                g_nFire_count = (pResPkt->param[0] | (pResPkt->param[1] << 8));
            }
            else if (pResPkt->code == 0x12)
            { // stop file
                
                digitalWrite(relay_pin, LOW);
                enableTriger = false;
                g_nFire_count = 0;
                nFSM = 0;
            }
            else if (pResPkt->code == 0x13)
            { // system info

                S_RES_SYS_PACKET _res_packet;
                // S_RES_SYS_PACKET *pSysPkt = &_res;

                _res_packet.header = system_Header;
                _res_packet.chipId = g_chipId;
                _res_packet.code = pResPkt->code;
                _res_packet.version = version;

                _res_packet.ipAddress[0] = WiFi.localIP()[0];
                _res_packet.ipAddress[1] = WiFi.localIP()[1];
                _res_packet.ipAddress[2] = WiFi.localIP()[2];
                _res_packet.ipAddress[3] = WiFi.localIP()[3];

                udp.writeTo((byte *)&_res_packet, sizeof(S_RES_SYS_PACKET), packet.remoteIP(), packet.remotePort());

                // 응답
                // Udp.beginPacket(Udp.remoteIP().toString().c_str(), Udp.remotePort());
                // Udp.write((byte *)&_res_packet, sizeof(S_RES_SYS_PACKET));
                // Udp.endPacket();
            }
            else if (pResPkt->code == 0x14)
            { // res status
                S_REQ_PACKET _res_packet;
                _res_packet.header = system_Header;
                _res_packet.chipId = g_chipId;
                _res_packet.index = deviceIndex;
                _res_packet.code = pResPkt->code;
                _res_packet.status = nFSM;
                _res_packet.pkt_size = sizeof(S_REQ_PACKET);
                _res_packet.count = g_nFire_count;
                _res_packet.extra[0] = version;
                _res_packet.extra[1] = WiFi.localIP()[3]; // last ip address

                // Serial.printf("send res : %d\n", g_nFire_count);

                udp.writeTo((byte *)&_res_packet, sizeof(S_REQ_PACKET), packet.remoteIP(), packet.remotePort());
            }
            else if (pResPkt->code == 0x21) // read config
            {
                String _s = g_config.dump();

                S_REQ_PACKET *pPkt = (S_REQ_PACKET *)packetBuffer;
                pPkt->header = system_Header;
                pPkt->chipId = g_chipId;
                pPkt->code = 0x21;
                pPkt->status = nFSM;
                pPkt->index = deviceIndex;
                pPkt->pkt_size = sizeof(S_REQ_PACKET);
                pPkt->count = _s.length();

                memcpy((char *)pPkt + sizeof(S_REQ_PACKET), _s.c_str(), _s.length());

                udp.writeTo(packetBuffer, sizeof(S_REQ_PACKET) + _s.length(), packet.remoteIP(), packet.remotePort());
            }
            else if (pResPkt->code == 0x22) // write config
            {
                //to do 
                g_config.save();
            }
            else if (pResPkt->code == 0x23) // format fifs
            {
                g_config.clear();
            }
            else if(pResPkt->code == 0x24) { // set triger delay

                trigger_delay = (pResPkt->param[0] | (pResPkt->param[1] << 8));

                Serial.printf("set trigger delay : %d\n", trigger_delay);

                g_config.set("trigger_delay", trigger_delay);
            }
            else if(pResPkt->code ==0x25) { //get trigger delay
                S_RES_PACKET *pPkt = (S_RES_PACKET *)packetBuffer;

                pPkt->header = system_Header;
                pPkt->code = 0x25;
                *((u16_t *)pPkt->param) = trigger_delay;
                pPkt->extra = 0;
                udp.writeTo(packetBuffer, sizeof(S_RES_PACKET), packet.remoteIP(), packet.remotePort());
                
            }
            else if (pResPkt->code == 0x1)
            { // pass
            }
            else if (pResPkt->code == 0x00)
            {
                ESP.restart();
            }
            else
            {
                Serial.printf("unknown code : %d \n", pResPkt->code);
            }
        }
    }


    
}

void procEGCS()
{
    static u32 _workTick = 0;
    
    // main control logic
    switch (nFSM)
    {
        case 0:
        {
            //ready
            if (g_nFire_count > 0)
            {
                nFSM = 9;
                // task_req.enable();
            }
        }
        break;
        case 7: {
            if(millis() - _workTick > 3000) {
                nFSM = 10;
                enableTriger = true;
                _workTick = millis();
                Serial.print("start game : ");
                Serial.println( String(g_nFire_count));
                
            }

        }
        break;
        case 8: {
            if(millis() - _workTick > 500) {
                nFSM = 7;
                digitalWrite(relay_pin, HIGH);
                Serial.println("wait system ready");
                
                _workTick = millis();

            }

        }
        break;
        case 9: { //start game
            _workTick = millis();
            nFSM = 8;
        }
        break;
        case 10:
        {
            if (!enableTriger) // 발사 신호 감지
            {
                g_nFire_count--;
                {
                    S_REQ_PACKET _req;
                    _req.header = system_Header;
                    _req.chipId = g_chipId;
                    _req.index = deviceIndex;
                    _req.code = 0x01;
                    _req.status = nFSM;
                    _req.pkt_size = sizeof(S_REQ_PACKET);
                    _req.count = g_nFire_count;
                    _req.extra[0] = version;
                    
                    udp.writeTo((byte *)&_req, sizeof(S_REQ_PACKET), g_remoteIp, g_remotePort);
                }

                nFSM = 11;
                _workTick = millis();
            }

        }
        break;

        case 11: {
            if(millis() - _workTick > trigger_delay) {
                

                if (g_nFire_count <= 0)
                {
                    nFSM = 12; // game over
                    // delay(100);
                    _workTick = millis();
                }
                else
                {
                    //격발완료시 다음 단계로 
                    if(digitalRead(sensor_pin) == HIGH) {
                        nFSM = 10; // next fire
                        enableTriger = true;
                    }
                }
            }
            else {
                
            }

        }
        break;
        case 12: {
            // if(millis() - _workTick > 100) {
            if(digitalRead(sensor_pin) == HIGH) {
                nFSM = 0;
                digitalWrite(relay_pin, LOW);
            }
        }
        break;
    }
   
}


void setupUDP() {
  if (udp.listen(udp_port_data)) {
    Serial.println("UDP Listening on port " + String(udp_port_data));
    udp.onPacket(onPacketReceived);

    Serial.println("UDP broadcast port : " + String(bc_port));
    Serial.printf("UDP Remote : %s:%d\n", g_remoteIp.toString().c_str(), g_remotePort);

    task_req.enable();
    task_Broadcast.enable();
  }
}


// DNS resolution task
Task task_Dns(0, TASK_ONCE, []() {

    Serial.println("Resolving DNS...");
    IPAddress resolvedIP;
    //   Serial.println("Resolving DNS...");
    if (WiFi.hostByName(remoteHostName.c_str(), resolvedIP)) {
        g_remoteIp = resolvedIP;
        dnsResolved = true;
        Serial.println("DNS resolved: " + g_remoteIp.toString());
        setupUDP();
    } else {
        Serial.println("DNS resolution failed. Retrying...");
        task_Dns.restartDelayed(TASK_SECOND * 5);  // 5초 후 재시도
    }
});


void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(sensor_pin, INPUT_PULLUP);
    pinMode(relay_pin, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(relay_pin, LOW);

    // 인터럽트 셋업
    attachInterrupt(digitalPinToInterrupt(sensor_pin), detect_sensor, FALLING);

#ifdef ESP8266
    g_chipId = ESP.getChipId();
#elif ESP32
    g_chipId = ESP.getEfuseMac();
#endif

    trigger_delay = g_config.get<int>("trigger_delay");
    g_chipIdStr += String(g_chipId);

    Serial.begin(115200);

    // delay(1000);

    Serial.println(":-[");

    Serial.print("chip id : ");
    Serial.println(g_chipIdStr);

    // device index
    if (g_config.hasKey("index"))
    {
        deviceIndex = g_config.get<int>("index");
    }

    if(g_config.hasKey("bc_port")) {
        bc_port = g_config.get<int>("bc_port");
    }

    if(g_config.hasKey("udp_port")) {
        udp_port_data = g_config.get<int>("udp_port");
    }

    // remote host name, port 가 존재하면 DNS 조회 task enable
    if (g_config.hasKey("remoteHost") && g_config.hasKey("remotePort"))
    {
        remoteHostName = g_config.get<String>("remoteHost");
        g_remotePort = g_config.get<int>("remotePort");

        Serial.print("Remote host: ");
        Serial.println(remoteHostName);
        Serial.print("Remote port: ");
        Serial.println(g_remotePort);

        // runner.addTask(dnsTask);
    }

    // ssid, password 가 졵재하면 wifi 연결
    if (g_config.hasKey("ssid") && g_config.hasKey("password"))
    {

        Serial.println("WiFi connecting");
        Serial.print("ssid: ");
        Serial.println(g_config.get<String>("ssid"));
        Serial.print("password: ");
        Serial.println(g_config.get<String>("password"));

        // WiFi event handlers
        // on connected
        staConnectedHandler = WiFi.onStationModeConnected([](const WiFiEventStationModeConnected &event)
                                                          { Serial.println("WiFi connected : " + event.ssid); });

        staGotIPHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP &event)
                                                  {
            Serial.println("Get IP address");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            // stop blink task
            task_Blink.disable();

            digitalWrite(LED_BUILTIN, LOW);

            if(!dnsResolved && !remoteHostName.isEmpty()) {
                task_Dns.enable();
            }
            else {
                setupUDP();
            }
            

            // UDP 수신 시작
            // if (udp.listen(udp_port))
            // { // 8888 포트로 UDP 수신 대기
            //     // Serial.println("UDP Listening on port 8888");
            //     //udp.onPacket(onPacketReceived);
            //     if (!dnsResolved && !remoteHostName.isEmpty()) {
            //         // Serial.println("Resolving DNS...ready"); 
            //         // dnsTask.enable();
            //         // task_req.enable();
            //         task_Dns.enable();
                    
            //     } else if (dnsResolved) {
            //         setupUDP();
            //     }
            // }
            // // UDP 브로드캐스트 시작
            // task_Broadcast.enable(); 
        });

        staDisconnectedHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &event) {
            Serial.println("WiFi lost connection");
            digitalWrite(LED_BUILTIN, HIGH);
            // start blink task
            task_Blink.enable();

            task_Broadcast.enable(); 
        });

        String _ssid = g_config.get<String>("ssid");
        String _password = g_config.get<String>("password");

        WiFi.begin(_ssid.c_str(), _password.c_str());
    }
    else
    {
        Serial.println("WiFi not connected");
    }

    Serial.println(":-]");
    Serial.println("Serial connected");


    runner.init();
    // runner.addTask(task_cmd);
    runner.addTask(task_Cmd);
    runner.addTask(task_Blink);
    runner.addTask(task_Broadcast);
    runner.addTask(task_Dns);
    runner.addTask(task_req);
    
    task_Cmd.enable();
    task_Blink.enable();
    // task_Broadcast.enable();
    
}


void loop()
{
    procEGCS();

    runner.execute();
}
