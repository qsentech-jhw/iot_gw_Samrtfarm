/*******************************************************/
/* board definition                                    */
/*******************************************************/
//#define bd_sample _bd_sample_ // comment out if not
#define bd_v2	_bd_v2_ // comment out if bd_v1

#ifdef bd_v2
  #define DO_NUM 16
#else
  #define DO_NUM 8
#endif

/*******************************************************/
/* pin definition                                      */
/*******************************************************/
#define nLED_USER  53
#define digital_toggle(a)  digitalWrite(a, !digitalRead(a))

#define PIN_RX0  0		// INPUT_PULLUP
#define PIN_RX1  19		// INPUT_PULLUP
#define PIN_RX2  17		// INPUT_PULLUP
#define PIN_RX3  15		// INPUT_PULLUP

#define RS232_0  Serial
#define RS232_1  Serial1
#define RS485_2  Serial2
#define RS485_3  Serial3

#define RS485_DE2  2
#define RS485_DE3  3

#ifdef bd_sample
  #define A_IN0    A3
  #define A_IN1    A2
  #define A_IN2    A1
  #define A_IN3    A0

  #define NET_SCSn  22
  #define NET_INTn  23
  #define NET_RSTn  24
#else                 
  #define A_IN0    A0
  #define A_IN1    A1
  #define A_IN2    A2
  #define A_IN3    A3

  #define NET_SCSn  7
  #define NET_INTn  8
  #define NET_RSTn  9
#endif

#define D_OUT0    37
#define D_OUT1    36
#define D_OUT2    35
#define D_OUT3    34
#define D_OUT4    33
#define D_OUT5    32
#define D_OUT6    31
#define D_OUT7    30

#ifdef bd_v2
  #define D_OUT8    22
  #define D_OUT9    23
  #define D_OUT10   24
  #define D_OUT11   25
  #define D_OUT12   26
  #define D_OUT13   27
  #define D_OUT14   28
  #define D_OUT15   29

#if 0
  #define NET_VEN	PJ3 // non-arduino
  #define SD_DETn	PE6 // non-arduino	
  #define SD_SCSn	PE7 // non-arduino
#else
  #define NET_VEN	70  // mega\pins_arduino.h modified
  #define SD_DETn	71  // mega\pins_arduino.h modified
  #define SD_SCSn	72  // mega\pins_arduino.h modified
#endif

  #define SOFT_RSTn  39
#else
  #define SOFT_RSTn  25
#endif

#define SOFT_RX4   10
#define SOFT_TX4   11

#define SOFT_RX5   12
#define SOFT_TX5   13

#define D_IN0    62
#define D_IN1    63
#define D_IN2    64
#define D_IN3    65
#define D_IN4    66
#define D_IN5    67
#define D_IN6    68
#define D_IN7    69
#define D_IN8    49
#define D_IN9    48
#define D_IN10   47
#define D_IN11   46
#define D_IN12   45
#define D_IN13   44
#define D_IN14   43
#define D_IN15   42

/*******************************************************/
/* debug macro                                         */
/*******************************************************/
#ifdef dbg_serial    
  #define dbg_print(a) dbg_serial.print(a)
  #define dbg_println(a) dbg_serial.println(a)
  #define dbg_print_(a, b) dbg_serial.print(a, b)
  #define dbg_println_(a, b) dbg_serial.println(a, b)
#else 
  #define dbg_print(a) {}
  #define dbg_println(a) {}
  #define dbg_print_(a, b) {}
  #define dbg_println_(a, b) {}
#endif    

/*eof*/