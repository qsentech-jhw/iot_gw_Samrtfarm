/***************************/
/* common setup            */
/***************************/
#define sd_card _sd_card_ // comment out if not

char* mqtt_user = "admin";
char* mqtt_pass = "pass123";

#ifdef sd_card

  uint8_t mac[6];
  IPAddress ip;
  IPAddress gateway;

  char mqttServer[16];
  char ClientID_NUM[5+1];

  char topic_head[32+1];
  char topic_tail[32+1];

#else // NON-SD

  uint8_t mac[] = { 0x70, 0xB3, 0xD5, 0xE1, 0x20, 0x94 };
  IPAddress ip(192, 168, 200, 94);
  IPAddress gateway(192, 168, 200, 1);

  char* mqttServer = "192.168.200.115";
  char* ClientID_NUM = "028"; // 3 digit GWID_ suffix

  char* topic_head = "SAMSUNG/HWASUNG/H2/AAA/";
  char* topic_tail = "1/3/1/GWY/";

#endif

/*eof*/