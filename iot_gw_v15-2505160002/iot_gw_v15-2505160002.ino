#define F_NUM "2505160002"

#define serial1 Serial1
//#define dbg_serial    Serial // uncomment if not

/*********************************************
  IOTGW-OTA by qst

  + github test

  + V15_2505160001
  - Ethernet.linkStatus 상태체크 추가
  - 와치독 4초확인으로 셋팅 - 20250516
  - <avr/wdt.h> - 20250428 와치독 추가

  + V14_2412170001
  - mqtt sub (debugging)
  - th sensor (debugging)

  + V13_2403132007
  - farm sensor board

  + V13_2403122005
  * farm only ***  
  - ini file

  - D8 ~ D15
  - D_IN0/D_OUT0 emergency

  - ph & nt sensor scan
  - cd sensor (4-20mA)

  + V13_2403122004
  - modbusRTUmaster z_filter (0,rx,0)

  + V13_2403122003
  - NET_VEN control (bd_v2)
  - true json topic
  - D_OUT control speed doubled

  + V12_3c_2402260000
  - mqtt reconnection

  + V12_3c
  - OTA by LAN
  - th sensor scan debugged (0,rx,0)
  - simple task switch
  - gw_server(?) removed

  + V10_3 (original)
*********************************************/

#include <avr/wdt.h> // 20250428 추가

#include <MsTimer2.h>

#include <SD.h>
#include <IniFile.h>

#include <Ethernet.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include "iotgw_cfg.h"
#include "iotgw_pin_v2.h"

#include "ModbusRTUMaster.h"

ModbusRTUMaster modbus_2(RS485_2, RS485_DE2);
ModbusRTUMaster modbus_3(RS485_3, RS485_DE3);

/*******************************************************/
/* setup variable                                      */
/*******************************************************/
IPAddress subnet(255, 255, 255, 0);
IPAddress myDns (168, 126, 63, 1);

static int mqttPort = 1883;

const char* ota_user = "arduino"; // can not be changed by its bug
const char* ota_pass = "passpass"; 

/*******************************************************/
/* task table & general variable                       */
/*******************************************************/
void task_farm_emg();
void task_ph_tx();
void task_ph_rx();

void task_mqtt_dn();
void task_mqtt_dn_(); // MAIN LOOP 에서 포트제어
void task_mqtt_up();
void task_http();
void task_io();
void task_ota();
void task_th_tx();
void task_th_rx();

void (*func_task[10])() = {
  task_th_tx,
  task_io,
  task_mqtt_up, 
  task_th_rx,
  0,//task_mqtt_dn, task_mqtt_dn_

  task_ph_tx,
  task_http,
  task_ota, // task_farm_emg
  task_ph_rx,
  0,//task_mqtt_dn,
};

EthernetServer http_server(80);

int8_t task_count = 0;

int8_t task_count_ = 0;
int8_t task_count_0 = 0;

bool timer1_fire = false;

uint8_t timeout_cnt = 0; 

uint8_t firstconnect = 0; 

/*******************************************************/
/* setup & loop                                        */
/*******************************************************/
void setup() 
{
  // 20250428 추가 / 20250516 와치독 제거 - 와치독 실행시 OTA 업데이트 안되는 증상 발현
  byte mcusr_mirror = MCUSR; // 리셋 원인 저장
  MCUSR = 0;                 // 레지스터 초기화
  wdt_disable();             // 부트할 때 와치독 끄기

  pinMode(nLED_USER, OUTPUT);

#ifdef bd_v2
  pinMode(NET_VEN, OUTPUT);
  digitalWrite(NET_VEN, HIGH);

  pinMode(SD_SCSn, OUTPUT);
  digitalWrite(SD_SCSn, HIGH);
#endif

  pinMode(NET_RSTn, OUTPUT);
  digitalWrite(NET_RSTn, HIGH);

  pinMode(D_OUT0, OUTPUT);
  pinMode(D_OUT1, OUTPUT);
  pinMode(D_OUT2, OUTPUT);
  pinMode(D_OUT3, OUTPUT);
  pinMode(D_OUT4, OUTPUT);
  pinMode(D_OUT5, OUTPUT);
  pinMode(D_OUT6, OUTPUT);
  pinMode(D_OUT7, OUTPUT);

#ifdef bd_v2
  pinMode(D_OUT8, OUTPUT);
  pinMode(D_OUT9, OUTPUT);
  pinMode(D_OUT10, OUTPUT);
  pinMode(D_OUT11, OUTPUT);
  pinMode(D_OUT12, OUTPUT);
  pinMode(D_OUT13, OUTPUT);
  pinMode(D_OUT14, OUTPUT);
  pinMode(D_OUT15, OUTPUT);
#endif

  pinMode(RS485_DE2, OUTPUT);
  pinMode(RS485_DE3, OUTPUT);

  pinMode(PIN_RX0, INPUT_PULLUP);
  pinMode(PIN_RX1, INPUT_PULLUP);
  pinMode(PIN_RX2, INPUT_PULLUP);
  pinMode(PIN_RX3, INPUT_PULLUP);

#ifdef dbg_serial
  dbg_serial.begin(115200);
#endif

  serial1.begin(9600);

  modbus_2.begin(9600);
  modbus_3.begin(9600);

#ifdef bd_v2
  pinMode(SD_DETn, INPUT_PULLUP);
  if (digitalRead(SD_DETn)) {
    dbg_println("sd: none");
  }
  else {
    dbg_println("sd: detected");
  }

  ini_setup();
#endif  

  net_setup();

  MsTimer2::set(100, timer1);
  MsTimer2::start(); // simple task switch

  // 리셋 원인 판별 20250428 추가
  if (mcusr_mirror & (1 << PORF)) {
    if (Serial1) {Serial1.println("mainlog_ Power-On Reset");} 
  }
  else if (mcusr_mirror & (1 << EXTRF)) {
    if (Serial1) {Serial1.println("mainlog_ External Reset Pin");}     
  }
  else if (mcusr_mirror & (1 << BORF)) {
    if (Serial1) {Serial1.println("mainlog_ Brown-Out Reset");}         
  }
  else if (mcusr_mirror & (1 << WDRF)) {
    if (Serial1) {Serial1.println("mainlog_ Watchdog-Reset 4S");}    
  }
  else {
    if (Serial1) {Serial1.println("mainlog_ Unknown Reset");}             
  }

  wdt_enable(WDTO_4S);
}

#ifdef bd_v2

void ini_setup()
{
  const size_t bufferLen = 80;
  char buffer[bufferLen];

  const char *filename = "cfg_farm.ini";
//  SPI.begin();
  if (!SD.begin(SD_SCSn)) {
    dbg_println("SD.begin() failed");
    return;
  }
  
  IniFile ini(filename);
  if (!ini.open()) {
    dbg_println("ini.open() failed");
    return;
  }  

  if (!ini.validate(buffer, bufferLen)) {
    dbg_print(ini.getFilename());
    dbg_println(" not valid: ");
    return;
  }

  if (ini.getMACAddress("network", "mac", buffer, bufferLen, mac)) {
    dbg_print("mac=");
    for(uint8_t i=0; i<6; i++) {
      dbg_print_(mac[i], HEX);
      if (i < 5) {
        dbg_print(":");
      }
    }
    dbg_println("");
  }

  if (ini.getIPAddress("network", "ip", buffer, bufferLen, ip)) {
    dbg_print("ip=");
    dbg_println(ip);
  }

  if (ini.getIPAddress("network", "gateway", buffer, bufferLen, gateway)) {
    dbg_print("gateway=");
    dbg_println(gateway);
  }

//  if (ini.getIPAddress("mqtt", "server", buffer, bufferLen, mqttServer)) {
  if (ini.getValue("mqtt", "server", buffer, bufferLen, mqttServer, sizeof(mqttServer))) {
    dbg_print("server=");
    dbg_println(mqttServer);
  }

  if (ini.getValue("mqtt", "client", buffer, bufferLen, ClientID_NUM, sizeof(ClientID_NUM))) {
    dbg_print("client=");
    dbg_println(ClientID_NUM);
  }

  if (ini.getValue("topic", "head", buffer, bufferLen, topic_head, sizeof(topic_head))) {
    dbg_print("head=");
    dbg_println(topic_head);
  }

  if (ini.getValue("topic", "tail", buffer, bufferLen, topic_tail, sizeof(topic_tail))) {
    dbg_print("tail=");
    dbg_println(topic_tail);
  }

  ini.close();
}

#endif

void net_setup()
{
  digitalWrite(NET_RSTn, LOW);
  delay(50); // enough
  digitalWrite(NET_RSTn, HIGH);
  delay(100); // enough

  Ethernet.init(NET_SCSn);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);

  do {
    delay(100); // blink fast if something wrong
    digital_toggle(nLED_USER);
  } while (Ethernet.hardwareStatus() == EthernetNoHardware);
  
  http_server.begin();
  SetMQTTInfo();

  // OTA 시작 시 콜백: WDT 비활성화
  ArduinoOTA.onStart([]() {    
    if (Serial1) {Serial1.println("mainlog_ OTA Start - Disabling WDT");}   
    wdt_disable();  // OTA 중에는 WDT 끔
  });

  ArduinoOTA.begin(Ethernet.localIP(), &ota_pass[0], &ota_pass[0], InternalStorage);
}

void timer1() 
{
  timer1_fire = true;
}

void task_ota() 
{
  ArduinoOTA.poll();
  task_farm_emg();
}

unsigned long ck_timer = 0;

unsigned long loop_qty = 0;

void loop() 
{

  wdt_reset();

  loop_qty++;

  unsigned long now = millis();  

  if (timer1_fire) {
    timer1_fire = false;
    if (func_task[task_count]) func_task[task_count]();
    if (++task_count == 10) task_count = 0;
  }  

  task_mqtt_dn(); 
  
  unsigned long now1 = millis();
  if (firstconnect != 0) {
    if ((now - ck_timer) > 1000) { // 죽었음    
      if (Serial1) {
        Serial1.print("mainlog_ LOOP > ");    // 
        Serial1.print(now - ck_timer);    // 
        Serial1.print(" over  /  operation time : ");    //            
        Serial1.print(now1 - now);    //       
        Serial1.print("  /  loop time : ");    //            
        Serial1.print(loop_qty);    //       
        Serial1.print("  /  millis : ");    //            
        Serial1.print(now1);    //       
        Serial1.println(" / ");    //      
        loop_qty = 0;
      } 
      ck_timer = now;
    } else {
      ck_timer = now;
    }
  } else {
    ck_timer = now;
  }

}

/*******************************************************/
/* i/o & http                                          */
/*******************************************************/
uint8_t DI[16] = {0,};
uint8_t DO[16] = {0,};
uint16_t AI[4] = {0,};

bool flag_emergency = false;

void task_io() 
{
  DI[0] = !digitalRead(D_IN0);

  if (DI[0]) {
    if (!flag_emergency) {
      PORTA = 0x00; // D_OUT15 ~ 0
      PORTC = 0x01; // D_OUT7 ~ 0

      flag_emergency = true;
    }
  }
  else {
    if (flag_emergency) {
      digitalWrite(D_OUT0, LOW);

      flag_emergency = false;
    }
  }

  DI[1] = !digitalRead(D_IN1);
  DI[2] = !digitalRead(D_IN2);
  DI[3] = !digitalRead(D_IN3);
  DI[4] = !digitalRead(D_IN4);
  DI[5] = !digitalRead(D_IN5);
  DI[6] = !digitalRead(D_IN6);
  DI[7] = !digitalRead(D_IN7);
  DI[8] = !digitalRead(D_IN8);
  DI[9] = !digitalRead(D_IN9);
  DI[10] = !digitalRead(D_IN10);
  DI[11] = !digitalRead(D_IN11);
  DI[12] = !digitalRead(D_IN12);
  DI[13] = !digitalRead(D_IN13);
  DI[14] = !digitalRead(D_IN14);
  DI[15] = !digitalRead(D_IN15);

  DO[0] = digitalRead(D_OUT0);
  DO[1] = digitalRead(D_OUT1);
  DO[2] = digitalRead(D_OUT2);
  DO[3] = digitalRead(D_OUT3);
  DO[4] = digitalRead(D_OUT4);
  DO[5] = digitalRead(D_OUT5);
  DO[6] = digitalRead(D_OUT6);
  DO[7] = digitalRead(D_OUT7);

#ifdef bd_v2
  DO[8] = digitalRead(D_OUT8);
  DO[9] = digitalRead(D_OUT9);
  DO[10] = digitalRead(D_OUT10);
  DO[11] = digitalRead(D_OUT11);
  DO[12] = digitalRead(D_OUT12);
  DO[13] = digitalRead(D_OUT13);
  DO[14] = digitalRead(D_OUT14);
  DO[15] = digitalRead(D_OUT15);
#endif

  AI[0] = analogRead(A_IN0);
  AI[1] = analogRead(A_IN1);
  AI[2] = analogRead(A_IN2);
  AI[3] = analogRead(A_IN3);
}

#define MAX_TH_COUNT  5
uint8_t th_dog[MAX_TH_COUNT+1] = {0,};

#define MAX_PH_COUNT  5
uint8_t ph_dog[MAX_PH_COUNT+1] = {0,};

void task_http() 
{
  char s0[17];
  char tmpS[100];

  EthernetClient client = http_server.available();
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");

          client.print("FWV=");
          client.print(F_NUM);
          client.println("<br />");

          client.print("MAC=");
          for(uint8_t i=0; i<6; i++) {
            client.print(mac[i], HEX);
            if (i < 5) client.print('-');
          }
          client.println("<br />");

          client.print("MID=");
          client.print(ClientID_NUM);
          client.println("<br />");

          uint8_t val1 = (DI[0]<<3)+(DI[1]<<2)+(DI[2]<<1)+DI[3];
          uint8_t val2 = (DI[4]<<3)+(DI[5]<<2)+(DI[6]<<1)+DI[7];
          uint8_t val3 = (DI[8]<<3)+(DI[9]<<2)+(DI[10]<<1)+DI[11];
          uint8_t val4 = (DI[12]<<3)+(DI[13]<<2)+(DI[14]<<1)+DI[15];
          client.print("D/I=");
          client.print(val1, HEX);
          client.print(val2, HEX);
          client.print(val3, HEX);
          client.print(val4, HEX);
          client.println("<br />");

          val1 = (DO[0]<<3)+(DO[1]<<2)+(DO[2]<<1)+DO[3];
          val2 = (DO[4]<<3)+(DO[5]<<2)+(DO[6]<<1)+DO[7];
          client.print("D/O=");
          client.print(val1, HEX);
          client.print(val2, HEX);
#ifdef bd_v2
          val3 = (DO[8]<<3)+(DO[9]<<2)+(DO[10]<<1)+DO[11];
          val4 = (DO[12]<<3)+(DO[13]<<2)+(DO[14]<<1)+DO[15];
          client.print(val3, HEX);
          client.print(val4, HEX);
#endif
          client.println("<br />");

          client.print("232=00"); // unused
          client.println("<br />");

          uint8_t state_ph = 0;
          for(uint8_t i=1; i<=MAX_PH_COUNT; i++) {
            if (ph_dog[i]) {
              state_ph = 1;
              break;
            }
          }

          uint8_t state_th = 0;
          for(uint8_t i=1; i<=MAX_TH_COUNT; i++) {
            if (th_dog[i]) {
              state_th = 1;
              break;
            }
          }

          client.print("485=");
          client.print(state_ph, HEX);
          client.print(state_th, HEX);
          client.println("<br />");

          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }  
}

/*******************************************************/
/* th sensor                                          */
/*******************************************************/
#define MAX_TEMP_CNT  10
uint8_t th_addr = 0;

uint16_t sHumid[11] = {0,};
int16_t sTemperature[11] = {0,};

void task_th_tx()
{
  if (++th_addr > MAX_TEMP_CNT) th_addr = 1;
  modbus_3.begin_readHoldingRegisters(th_addr, 0, 2);
}

void task_th_rx() 
{
  uint16_t th_val[2];

  uint8_t ret = modbus_3.readHoldingRegisters_end(th_addr, th_val, 2);
  if (ret) {
    sHumid[th_addr] = th_val[0];
    sTemperature[th_addr] = th_val[1];

    th_dog[th_addr] = 10; // 10 * 5 sec
  }
  else {
    if (th_dog[th_addr]) {
      if (--th_dog[th_addr] == 0) { // saturated
        sHumid[th_addr] = 0;
        sTemperature[th_addr] = 0;
      }
    }
  }
}

uint8_t ph_addr = 0;

uint16_t sPH[6] = {0,};
uint16_t sNT[6] = {0,};
uint16_t sCD[6] = {0,};

void task_farm_emg()
{
  if (flag_emergency) {
    digital_toggle(D_OUT0);
  }
}

void task_ph_tx()
{
  if (++ph_addr > MAX_PH_COUNT) ph_addr = 1;
  modbus_2.begin_readHoldingRegisters(ph_addr, 0, 3); // ph, ec & co2 (rh & temp)
}

void task_ph_rx()
{
  uint16_t ph_val[3];

  uint8_t ret = modbus_2.readHoldingRegisters_end(ph_addr, ph_val, 3);
  if (ret) {
    sPH[th_addr] = ph_val[0];
    sNT[th_addr] = ph_val[1];
    sCD[th_addr] = ph_val[2]; 

    ph_dog[ph_addr] = 10; // 10 * 5 sec
  }
  else {
    if (ph_dog[ph_addr]) {
      if (--ph_dog[ph_addr] == 0) { // saturated
        sPH[ph_addr] = 0;
        sNT[th_addr] = 0;
        sCD[th_addr] = 0; 
      }
    }
  }
}

/*******************************************************/
/* mqtt up/dn                                          */
/*******************************************************/
EthernetClient ethClient;
PubSubClient mqttClient(mqttServer, mqttPort, callback, ethClient);

void task_mqtt_dn() 
{
  {
    mqttClient.loop();
  }
}

void task_mqtt_dn_() 
{
  if (task_count_ == 0) {
    if (task_count_0 == 0) {

      if (Serial1) Serial1.println("=========================================== D_OUT7 , D_OUT5 : ON  ");    // 

      int randValue = random(0, 2);
      digitalWrite(D_OUT7, HIGH);
      int randValue_ = random(0, 2);  
      digitalWrite(D_OUT5, HIGH);
      task_count_0 = 1;      
    } else {

      if (Serial1) Serial1.println("=========================================== D_OUT7 , D_OUT5 : OFF  ");    // 

      int randValue = random(0, 2);
      digitalWrite(D_OUT7, LOW);
      int randValue_ = random(0, 2);  
      digitalWrite(D_OUT5, LOW);
      task_count_0 = 0;            
    }

    task_count_ = 1;
  } else {
    if (Serial1) Serial1.println("=========================================== ");    //     
    task_count_ = 0;
  }  
}

char UP_TOPICNAME[128];
char DN_TOPICNAME[128];

unsigned long lastReconnectAttempt = 0;
unsigned long lastReconnectAttempt_ = 0;

void task_mqtt_up()
{
  char s0[32];
  char tmpS[100];
  char sndbuf[100]; 
  char sndPacket[512]; 
    
  sndPacket[0] = 0;

  // Digital Input
  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=0; i<16; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", DI[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "{\x22\x44I\x22:\x22%s\x22,", tmpS); // "D
  strcat(sndPacket, sndbuf);

  // Digital Output
  sndbuf[0] = 0;
  tmpS[0] = 0;
#ifdef bd_v2
  for(uint8_t i=0; i<16; i++) {
#else
  for(uint8_t i=0; i<8; i++) {
#endif	
    s0[0] = 0; 
    sprintf(s0, "%d|", DO[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22\x44O\x22:\x22%s\x22,", tmpS); // "D
  strcat(sndPacket, sndbuf);

  // Analog Input
  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=0; i<4; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", AI[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22\x41I\x22:\x22%s\x22,", tmpS); // "A
  strcat(sndPacket, sndbuf);

  // H Input
  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=1; i<=MAX_TH_COUNT; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", sHumid[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22HM\x22:\x22%s\x22,", tmpS);
  strcat(sndPacket, sndbuf);

  // T Input
  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=1; i<=MAX_TH_COUNT; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", sTemperature[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22TM\x22:\x22%s\x22,", tmpS);
  strcat(sndPacket, sndbuf);

  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=1; i<=MAX_PH_COUNT; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", sCD[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22\x43\x44\x22:\x22%s\x22,", tmpS); // "CD 
  strcat(sndPacket, sndbuf);

  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=1; i<=MAX_PH_COUNT; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", sNT[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22NT\x22:\x22%s\x22,", tmpS);
  strcat(sndPacket, sndbuf);

  sndbuf[0] = 0;
  tmpS[0] = 0;
  for(uint8_t i=1; i<=MAX_PH_COUNT; i++) {
    s0[0] = 0; 
    sprintf(s0, "%d|", sPH[i]);
    strcat(tmpS, s0);
  }
  tmpS[strlen(tmpS)-1] = 0;
  sprintf(sndbuf, "\x22PH\x22:\x22%s\x22}", tmpS);
  strcat(sndPacket, sndbuf);

  unsigned long now = millis();

  if (mqttClient.connected()) {
    mqttClient.publish(UP_TOPICNAME, sndPacket);

    digital_toggle(nLED_USER); 

    lastReconnectAttempt = now;
  }
  else {
    if (now - lastReconnectAttempt > 2000) { // 마지막 MQTT publish 와 2초 이상 차이나면 mqtt or LAN 리셋
      lastReconnectAttempt = now;
      if (Ethernet.linkStatus() != LinkON) {
        // 링크 문제 있을 때만 LAN 리셋
        #ifdef bd_v2
          digitalWrite(NET_VEN, LOW);
          delay(100);
          digitalWrite(NET_VEN, HIGH);
        #endif

      if (Serial1) Serial1.print("mainlog_ Ethernet.linkStatus - FALSE  /  Millis : ");      
      if (Serial1) Serial1.println(now);

        net_setup();
      } else {
        // 링크는 OK → MQTT만 재연결 시도
        beginConnection();
      }    
    }  

    /* 20250516 
    digitalWrite(nLED_USER, LOW);

    if (firstconnect != 0) {  
      if (now - lastReconnectAttempt > 2000) {
        if (timeout_cnt == 3) {
          if (now - lastReconnectAttempt > 1000) {  //리커넥션후 연결 안되면 바로 장비 초기화
            lastReconnectAttempt = now;
            if (beginConnection()) {        
              lastReconnectAttempt = 0;
            }
          }
        }
        else {
          if (now - lastReconnectAttempt > 5000) {  // 5초 처음 리커넥션 - 
            lastReconnectAttempt = now;
            if (beginConnection()) {        
              lastReconnectAttempt = 0;
            }
          }
        }
      }
    }
    else {
      lastReconnectAttempt = now;
      if (beginConnection()) {        
        lastReconnectAttempt = 0;
      }
    }
    */ 
  }  
}

char ClientID[16];

boolean beginConnection() 
{
  unsigned long now = millis();  
  if(mqttClient.connect(ClientID, mqtt_user, mqtt_pass))
  {
    mqttClient.subscribe(DN_TOPICNAME);
    if (Serial1) Serial1.print("mainlog_ MQTT Connected & Subscribed re_cnt : ");
    if (Serial1) Serial1.print(timeout_cnt);
    if (Serial1) Serial1.print("  /   Millis : ");
    if (Serial1) Serial1.println(now);
    timeout_cnt = 0;
    return true;  // 연결 성공
  }
  else {
    if (timeout_cnt > 1) { //세번 재접속 실패시 리셋 -> 2번 시도  /  로그확인결과 1,2번째 재접속 시도시 붙은 적이 없음 세번까지 재접속할 필요 없음
      timeout_cnt = 0;
      mqttClient.disconnect();

      #ifdef bd_v2
        digitalWrite(NET_VEN, LOW);
        delay(100);
        digitalWrite(NET_VEN, HIGH);
      #endif

      net_setup();
      if (Serial1) Serial1.print("mainlog_Ethernet reinitialized (Millis : ");
      if (Serial1) Serial1.print(now);
      if (Serial1) Serial1.println(")");
    } else {

      if (Serial1) Serial1.print("mainlog_ MQTT not Connected re_cnt : ");
      if (Serial1) Serial1.print(timeout_cnt);
      if (Serial1) Serial1.print("  /   Millis : ");
      if (Serial1) Serial1.println(now);
      timeout_cnt++;          
      return false; // 연결 실패    
    }
  }  


/*
  unsigned long now = millis();
  if (firstconnect != 0) {  
    if (now - lastReconnectAttempt_ < 5000) {
      return false; // 아직 재시도하지 않음
    }
  }
  
  firstconnect = 1;

  lastReconnectAttempt_ = now;

  if(mqttClient.connect(ClientID, mqtt_user, mqtt_pass))
  {
    mqttClient.subscribe(DN_TOPICNAME);
    timeout_cnt = 0;

    if (Serial1) Serial1.println("MQTT Connected & Subscribed");       //

    return true;  // 연결 성공
  }
  else {
    timeout_cnt++;

    if (Serial1) {
      Serial1.print("mainlog_MQTT connect failed. Count: ");
      Serial1.println(timeout_cnt);
    }

    if (timeout_cnt >= 2) {
      timeout_cnt = 0;
      mqttClient.disconnect();

#ifdef bd_v2
      digitalWrite(NET_VEN, LOW);
      delay(100);
      digitalWrite(NET_VEN, HIGH);
#endif
      //resetW5500();
      net_setup();
      if (Serial1) Serial1.println("mainlog_Ethernet reinitialized");
    }
    return false; // 연결 실패    
  }
*/

}

char rcvPacket[128];
int rcv_cnt = 0;
int step = 0;

String p_str = "";
char p_val;

void relay_on_00()
{
  digitalWrite(D_OUT0, HIGH);
}

void relay_on_01()
{
  digitalWrite(D_OUT1, HIGH);
}

void relay_on_02()
{
  digitalWrite(D_OUT2, HIGH);
}

void relay_on_03()
{
  digitalWrite(D_OUT3, HIGH);
}

void relay_on_04()
{
  digitalWrite(D_OUT4, HIGH);
}

void relay_on_05()
{
  digitalWrite(D_OUT5, HIGH);
}

void relay_on_06()
{
  digitalWrite(D_OUT6, HIGH);
}

void relay_on_07()
{
  digitalWrite(D_OUT7, HIGH);
}

void relay_on_08()
{
  digitalWrite(D_OUT8, HIGH);
}

void relay_on_09()
{
  digitalWrite(D_OUT9, HIGH);
}

void relay_on_10()
{
  digitalWrite(D_OUT10, HIGH);
}

void relay_on_11()
{
  digitalWrite(D_OUT11, HIGH);
}

void relay_on_12()
{
  digitalWrite(D_OUT12, HIGH);
}

void relay_on_13()
{
  digitalWrite(D_OUT13, HIGH);
}

void relay_on_14()
{
  digitalWrite(D_OUT14, HIGH);
}

void relay_on_15()
{
  digitalWrite(D_OUT15, HIGH);
}

void relay_off_00()
{
  digitalWrite(D_OUT0, LOW);
}

void relay_off_01()
{
  digitalWrite(D_OUT1, LOW);
}

void relay_off_02()
{
  digitalWrite(D_OUT2, LOW);
}

void relay_off_03()
{
  digitalWrite(D_OUT3, LOW);
}

void relay_off_04()
{
  digitalWrite(D_OUT4, LOW);
}

void relay_off_05()
{
  digitalWrite(D_OUT5, LOW);
}

void relay_off_06()
{
  digitalWrite(D_OUT6, LOW);
}

void relay_off_07()
{
  digitalWrite(D_OUT7, LOW);
}

void relay_off_08()
{
  digitalWrite(D_OUT8, LOW);
}

void relay_off_09()
{
  digitalWrite(D_OUT9, LOW);
}

void relay_off_10()
{
  digitalWrite(D_OUT10, LOW);
}

void relay_off_11()
{
  digitalWrite(D_OUT11, LOW);
}

void relay_off_12()
{
  digitalWrite(D_OUT12, LOW);
}

void relay_off_13()
{
  digitalWrite(D_OUT13, LOW);
}

void relay_off_14()
{
  digitalWrite(D_OUT14, LOW);
}

void relay_off_15()
{
  digitalWrite(D_OUT15, LOW);
}

void (*func_relay_on[16])() = {
  relay_on_00,
  relay_on_01,
  relay_on_02,
  relay_on_03,
  relay_on_04,
  relay_on_05,
  relay_on_06,
  relay_on_07,

  relay_on_08,
  relay_on_09,
  relay_on_10,
  relay_on_11,
  relay_on_12,
  relay_on_13,
  relay_on_14,
  relay_on_15,
};

void (*func_relay_off[16])() = {
  relay_off_00,
  relay_off_01,
  relay_off_02,
  relay_off_03,
  relay_off_04,
  relay_off_05,
  relay_off_06,
  relay_off_07,

  relay_off_08,
  relay_off_09,
  relay_off_10,
  relay_off_11,
  relay_off_12,
  relay_off_13,
  relay_off_14,
  relay_off_15,
};

#if 1
void relay_on(uint8_t p)
{
  if (p_val == '1') func_relay_on[p]();
  else if (p_val == '0') func_relay_off[p]();
}
#else
void relay_on(uint8_t p)
{
          if (p_val == '1') {
                 if (p==0) digitalWrite(D_OUT0, HIGH); 
            else if (p==1) digitalWrite(D_OUT1, HIGH);
            else if (p==2) digitalWrite(D_OUT2, HIGH);
            else if (p==3) digitalWrite(D_OUT3, HIGH);
            else if (p==4) digitalWrite(D_OUT4, HIGH);
            else if (p==5) digitalWrite(D_OUT5, HIGH);
            else if (p==6) digitalWrite(D_OUT6, HIGH);
            else if (p==7) digitalWrite(D_OUT7, HIGH);

#ifdef bd_v2
            else if (p==8) digitalWrite(D_OUT8, HIGH);
            else if (p==9) digitalWrite(D_OUT9, HIGH);
            else if (p==10) digitalWrite(D_OUT10, HIGH);
            else if (p==11) digitalWrite(D_OUT11, HIGH);
            else if (p==12) digitalWrite(D_OUT12, HIGH);
            else if (p==13) digitalWrite(D_OUT13, HIGH);
            else if (p==14) digitalWrite(D_OUT14, HIGH);
            else if (p==15) digitalWrite(D_OUT15, HIGH);
#endif
          }
          else if (p_val == '0') {
                 if (p==0) digitalWrite(D_OUT0, LOW); 
            else if (p==1) digitalWrite(D_OUT1, LOW);
            else if (p==2) digitalWrite(D_OUT2, LOW);
            else if (p==3) digitalWrite(D_OUT3, LOW);
            else if (p==4) digitalWrite(D_OUT4, LOW);
            else if (p==5) digitalWrite(D_OUT5, LOW);
            else if (p==6) digitalWrite(D_OUT6, LOW);
            else if (p==7) digitalWrite(D_OUT7, LOW);

#ifdef bd_v2
            else if (p==8) digitalWrite(D_OUT8, LOW);
            else if (p==9) digitalWrite(D_OUT9, LOW);
            else if (p==10) digitalWrite(D_OUT10, LOW);
            else if (p==11) digitalWrite(D_OUT11, LOW);
            else if (p==12) digitalWrite(D_OUT12, LOW);
            else if (p==13) digitalWrite(D_OUT13, LOW);
            else if (p==14) digitalWrite(D_OUT14, LOW);
            else if (p==15) digitalWrite(D_OUT15, LOW);
#endif
          }
}
#endif

void callback(char* topic, byte* payload, unsigned int length) 
{
  if (flag_emergency) return;

  // handle message arrived
  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];    

    switch(step) {
      case 1 :
        if (c == '}') {
          step = 2;
        }
        else {
          if(rcv_cnt > sizeof(rcvPacket)) { // overflow
            step = 0;
            break;
          }

          rcvPacket[rcv_cnt] = c;
          rcv_cnt++;
          break;
        }
      case 2 :
        if (!(rcvPacket[1] == 'D') || !(rcvPacket[2] == 'O')) {
          step = 0;
          break;
        }
#if 1 // issue 241227
        if (rcvPacket[4] == ':') { 
          for(uint8_t p=0; p<DO_NUM; p++) {
            p_val = rcvPacket[6+2*p];
            relay_on(p);
          }
        }
        else if ((rcvPacket[3] == '_') && (rcvPacket[7] == ':')) {
#if 1 // issue 241227
          uint8_t p = 0;
          if (rcvPacket[4] == '1') p = 10;
          p += (rcvPacket[5] - '0');         
          if (p <= 15) {
            p_val = rcvPacket[9];
            relay_on(p);
          }
#else
          p_str = &rcvPacket[4];
          rcvPacket[6] = 0;

          p_val = rcvPacket[9];
          relay_on(p_str.toInt());
#endif        
        }
#endif
        step = 0;
        break;
      case 0:
        if (c == '{') {
          step = 1;
          rcv_cnt = 0;
          rcvPacket[0] = 0;
        }  
        break;
    }
  }

}

void SetMQTTInfo()
{
  mqttClient.setBufferSize(512); // default 256

  String _ClientID = "GWID_"+(String)ClientID_NUM;
  _ClientID.toCharArray(ClientID, sizeof(ClientID));

  String _UP_TOPICNAME = (String)topic_head+"PUB/"+(String)topic_tail+(String)ClientID_NUM;
  _UP_TOPICNAME.toCharArray(UP_TOPICNAME, sizeof(UP_TOPICNAME));

  String _DN_TOPICNAME = (String)topic_head+"SUB/"+(String)topic_tail+(String)ClientID_NUM;
  _DN_TOPICNAME.toCharArray(DN_TOPICNAME, sizeof(DN_TOPICNAME));
}

/*eof*/