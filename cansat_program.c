// CANSAT PROGRAM : 2016.10.13 hypark@satrec
#include <Arduino.h> // USE Arduino
#include <EEPROM.h>  // USE arduino EEPROM
#include <Servo.h>   // USE arduino SERVO 

// BT
#define BT_NEED_INIT 1

// SD CARD //
#include <SPI.h>
#include <SD.h>
File csLogFile ;


#define GS_0_PL_1  // if defined  GS = UART_PORT_0 , PL = UART_PORT_1 VERSION 2014
//#define GS_1_PL_0  // if defined  GS = UART_PORT_0 , PL = UART_PORT_1 VERSION 2012,3

const unsigned char MASK_BIT_OR [8] = { 0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01 };
const unsigned char MASK_BIT_AND[8] = { 0x7F,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,0xFE };

unsigned char  time_tick=0 ;
unsigned short time_servo_tick=0;
unsigned short time_msec=0 ; // use millis() % 1000 ;
unsigned long time_run =0 ;  // use millis() / 1000 ; 
unsigned long time_sec =0 ;  // use gps + time_run 

// PORT 0 is Camera
// PORT 1 is GS
#define MAX_PL_RX_QUEUE     128
#define MAX_GS_RX_QUEUE      32
#define MAX_PL_TX_QUEUE      32
#define MAX_GS_TX_QUEUE     256
#define PL_RX_QUEUE_IDX_MASK 0x7F
#define GS_RX_QUEUE_IDX_MASK 0x1F
#define PL_TX_QUEUE_IDX_MASK 0x1F
#define GS_TX_QUEUE_IDX_MASK 0xFF

#define TX_SPEED_9600       9600   
#define TX_SPEED_38400      38400  
#define TX_SPEED_115200     115200 

// baud_setting = (F_CPU / 8 / baud - 1) / 2;

#define PL_TX_SPEED        115200
#define GS_TX_SPEED        115200

#ifdef GS_0_PL_1

#define GS_PORT Serial
#define PL_PORT Serial1

#else

#define PL_PORT Serial
#define GS_PORT Serial1

#endif

///////////////////////////////////////////////////////////////////////////////////
// USE ARDUINO's begin function.. OK 
//    baud_setting = (F_CPU / 8 / baud - 1) / 2;
#define GS_PORT_SPEED_SET(macSpeed) GS_PORT.begin(macSpeed)
#define PL_PORT_SPEED_SET(macSpeed) PL_PORT.begin(macSpeed) 
///////////////////////////////////////////////////////////////////////////////////
void PL_UART_INIT() { PL_PORT.begin(PL_TX_SPEED); }
void GS_UART_INIT() { GS_PORT.begin(GS_TX_SPEED); }
///////////////////////////////////////////////////////////////////////////////////

unsigned char Pl_Rx_QueueInIdx  = 0 ;
unsigned char Pl_Rx_QueueOutIdx = 0 ;
unsigned char Pl_Rx_Queue[MAX_PL_RX_QUEUE];	// for packet data

unsigned char Gs_Rx_QueueInIdx  = 0 ;
unsigned char Gs_Rx_QueueOutIdx = 0 ;
unsigned char Gs_Rx_Queue[MAX_GS_RX_QUEUE];	// for packet data


unsigned char Pl_Tx_QueueInIdx  = 0 ;
unsigned char Pl_Tx_QueueOutIdx = 0 ;
unsigned char Pl_Tx_Queue[MAX_PL_TX_QUEUE];	// for packet data

unsigned char Gs_Tx_QueueInIdx  = 0 ;
unsigned char Gs_Tx_QueueOutIdx = 0 ;
unsigned char Gs_Tx_Queue[MAX_GS_TX_QUEUE];	// for packet data


#define Pl_Rx_QueueIn(macData)  { Pl_Rx_QueueInIdx  &= PL_RX_QUEUE_IDX_MASK;	Pl_Rx_Queue[Pl_Rx_QueueInIdx] = (macData) ; Pl_Rx_QueueInIdx++   ; }
#define Pl_Rx_QueueOut(macData) { Pl_Rx_QueueOutIdx &= PL_RX_QUEUE_IDX_MASK;	(macData) = Pl_Rx_Queue[Pl_Rx_QueueOutIdx]; Pl_Rx_QueueOutIdx++  ; }
#define Pl_Rx_QueueFlush()      { Pl_Rx_QueueInIdx = Pl_Rx_QueueOutIdx; }
							   
#define Gs_Rx_QueueIn(macData)  { Gs_Rx_QueueInIdx  &= GS_RX_QUEUE_IDX_MASK;	Gs_Rx_Queue[Gs_Rx_QueueInIdx] = (macData) ; Gs_Rx_QueueInIdx++   ; }
#define Gs_Rx_QueueOut(macData) { Gs_Rx_QueueOutIdx &= GS_RX_QUEUE_IDX_MASK;	(macData) = Gs_Rx_Queue[Gs_Rx_QueueOutIdx]; Gs_Rx_QueueOutIdx++  ; }
#define Gs_Rx_QueueFlush()      { Gs_Rx_QueueInIdx = Gs_Rx_QueueOutIdx; }


#define Pl_Tx_QueueIn(macData)  { Pl_Tx_QueueInIdx  &= PL_TX_QUEUE_IDX_MASK;	Pl_Tx_Queue[Pl_Tx_QueueInIdx] = (macData) ; Pl_Tx_QueueInIdx++   ; }
#define Pl_Tx_QueueOut(macData) { Pl_Tx_QueueOutIdx &= PL_TX_QUEUE_IDX_MASK;	(macData) = Pl_Tx_Queue[Pl_Tx_QueueOutIdx]; Pl_Tx_QueueOutIdx++  ; }
#define Pl_Tx_QueueFlush()      { Pl_Tx_QueueInIdx = Pl_Tx_QueueOutIdx; }
							   
#define Gs_Tx_QueueIn(macData)  { Gs_Tx_QueueInIdx  &= GS_TX_QUEUE_IDX_MASK;	Gs_Tx_Queue[Gs_Tx_QueueInIdx] = (macData) ; Gs_Tx_QueueInIdx++   ; }
#define Gs_Tx_QueueOut(macData) { Gs_Tx_QueueOutIdx &= GS_TX_QUEUE_IDX_MASK;	(macData) = Gs_Tx_Queue[Gs_Tx_QueueOutIdx]; Gs_Tx_QueueOutIdx++  ; }
#define Gs_Tx_QueueFlush()      { Gs_Tx_QueueInIdx = Gs_Tx_QueueOutIdx; }


#define Pl_Rx_QueueIsEmpty() ( Pl_Rx_QueueInIdx == Pl_Rx_QueueOutIdx )
#define Gs_Rx_QueueIsEmpty() ( Gs_Rx_QueueInIdx == Gs_Rx_QueueOutIdx )
#define Pl_Tx_QueueIsEmpty() ( Pl_Tx_QueueInIdx == Pl_Tx_QueueOutIdx )
#define Gs_Tx_QueueIsEmpty() ( Gs_Tx_QueueInIdx == Gs_Tx_QueueOutIdx )

#define Pl_Rx_QueueIsNotEmpty() ( Pl_Rx_QueueInIdx != Pl_Rx_QueueOutIdx )
#define Gs_Rx_QueueIsNotEmpty() ( Gs_Rx_QueueInIdx != Gs_Rx_QueueOutIdx )
#define Pl_Tx_QueueIsNotEmpty() ( Pl_Tx_QueueInIdx != Pl_Tx_QueueOutIdx )
#define Gs_Tx_QueueIsNotEmpty() ( Gs_Tx_QueueInIdx != Gs_Tx_QueueOutIdx )

#define RxMode_DEFAULT		0
#define RxMode_CAMERA		1
#define RxMode_CAMERA_SIZE	2
#define RxMode_CAMERA_IMAGE	3
#define RxMode_GPS		4
#define RxMode_IMU		5
#define RxMode_ATLM		6
#define RxMode_DTLM		7


unsigned char hyRxMode = RxMode_CAMERA ;  // when 0 

#define RxMode_CAMERA_PORT 0x00  // 0000 0000 = 0
#define RxMode_GPS_PORT    0x01  // 0010 0001 = 1
#define RxMode_IMU_PORT    0x02  // 0100 0020 = 2
// need direction is 0

void hyRxMode_PortSet( unsigned char iMode)
{
  unsigned char b = PINC  ;
  b     &= 0xF0  ; // direction is 0
  b     |= iMode ;
  PORTC = b      ; 
}



// 각 단계별 필요한 수행 횟수 정의 ( 기준 11.0592 Mhz Clock 사용시 )
#define WAIT_CNT_SEND		    0x000100
#define WAIT_CNT_GPS		    0x020000
#define WAIT_CNT_IMU		    0x010000
#define WAIT_CNT_ATLM		    0x000080
//#define WAIT_CNT_CAMERA_STEP	    0x002000
#define WAIT_CNT_CAMERA_STEP	    0x001000
#define WAIT_CNT_CAMERA_INIT_STEP   0x007000
#define WAIT_CNT_CAMERA_IMAGE_160   0x010000
#define WAIT_CNT_CAMERA_IMAGE_320   0x020000
#define WAIT_CNT_CAMERA_IMAGE_640   0x080000

unsigned long  WAIT_CNT_CAMERA_IMAGE = WAIT_CNT_CAMERA_IMAGE_640 ;
//#define WAIT_CNT_CAMERA_IMAGE	WAIT_CNT_CAMERA_IMAGE_640

#define WAIT_CNT_BT_STEP        0x010000


// Payload 데이터를 얻기 위해 UART 포트의 수신 속도 및 MUX를 설정한다.
void hyRxMode_Set(unsigned char iMode)
{
	switch(iMode)
	{
		case RxMode_CAMERA      :
		case RxMode_CAMERA_SIZE :
		case RxMode_CAMERA_IMAGE: PL_PORT_SPEED_SET(115200) ; hyRxMode = iMode ; hyRxMode_PortSet(RxMode_CAMERA_PORT) ; break;
		case RxMode_IMU         : PL_PORT_SPEED_SET(115200) ; hyRxMode = iMode ; hyRxMode_PortSet(RxMode_IMU_PORT)    ; break;
		case RxMode_GPS         : PL_PORT_SPEED_SET(9600)   ; hyRxMode = iMode ; hyRxMode_PortSet(RxMode_GPS_PORT)    ; break;
		case RxMode_ATLM        : PL_PORT_SPEED_SET(115200) ; hyRxMode = iMode ; break;
		case RxMode_DTLM        : PL_PORT_SPEED_SET(115200) ; hyRxMode = iMode ; break;
	}
}

void hyCANSAT_USER_OPERATION();
void hyTransUartWait(unsigned long cnt);

/////////////////////////
// 한 바이트 송신 함수 //
#define hyPlTxByte(data)     PL_PORT.write(data)
#define hyGsTxByte(data)     GS_PORT.write(data)
#define hyGsSetTxByte(data)  hyGsTxByte(data)
//#define hyGsSetTxByte(data)  GS_PORT.write(data)

// read one byte processing //
// 각 포트로부터 한바이트 데이터를 받았을 경우 처리한다.
void hyGs_RecvByte(unsigned char b);
void hyCamera_RecvByte(unsigned char b);
void hyImu_RecvByte(unsigned char b);
void hyGps_RecvByte(unsigned char b);

///////////////////////////////////////////////////
// Analog Inputs  :: ADC Conver 할 갯수를 설정한다.
#define ADC_TLMS 8
#define ADC_MASK 0x07

///////////////////////////////////////////////////
// LED output
unsigned char cansat_led  = 0xFF  ;

///////////////////////////////////////////////////
// COMMANDS 4 바이트 32개의 명령을 다룬다.
// cansat_dcmd[4] 처럼 array 로 다루어도 되지만,
// 각각의 처리 시간을 아끼기 위해서 그냥 변수로 사용하였다.
// DTLM & CMD & iCMD
unsigned char cansat_dcmd_0 = 0xFF  ; // operation flag 1111 1111
unsigned char cansat_dcmd_1 = 0x00  ; // dcmd_1 for use Digital Port
unsigned char cansat_dcmd_2 = 0x00  ; // dcmd_2 for use sub command
unsigned char cansat_dcmd_3 = 0x00  ; // dcmd_3 for use Analog Port 

unsigned char cansat_byte_cmd[8] = { 0,0,0,0,0,0,0,0 } ;

// 2012.6.29 : Append OP_MASK_USER for USER Defined Operation

// cansat_dcmd_0 를 다룬다.
#define OP_MASK_GPS        0x80
#define OP_MASK_IMU        0x40
#define OP_MASK_CAMERA     0x20 
#define OP_MASK_ATLM       0x10
#define OP_MASK_DTLM       0x08
#define OP_MASK_USER       0x04
#define OP_MASK_CAMERA_160 0x02 // default 320 when 1 0 then 160
#define OP_MASK_CAMERA_640 0x01 // default 320 when 0 1 then 640

#define IS_NEED_OP(macMask) (( cansat_dcmd_0 & (macMask) ) ? 1 : 0 )
#define TOGGLE_OP(macMask)  {cansat_dcmd_0 ^= (macMask) ; }
#define SET_OP(macMask)     {cansat_dcmd_0 |=  (macMask) ; }
#define CLEAR_OP(macMask)   {cansat_dcmd_0 &= ~(macMask) ; }
	
// cansat_dcmd_2 를 다룬다.
#define ICMD_MASK_RESET             0x80 // reset cansat 
#define ICMD_MASK_CAMERA_RESET      0x40 // reset camera command
#define ICMD_MASK_ATLM_TO_DIGITAL   0x20 // when user operation
#define ICMD_MASK_ATLM_TO_LED       0x10 // when user operation
#define ICMD_MASK_PWM_0_L           0x20 // when not user/pwm operation
#define ICMD_MASK_PWM_0_R           0x10 // when not user/pwm operation
#define ICMD_MASK_PWM_1_L           0x08 // when not user/pwm operation
#define ICMD_MASK_PWM_1_R           0x04 // when not user/pwm operation
#define ICMD_MASK_GS                0x02
#define ICMD_MASK_SIGN              0x01

#define IS_NEED_ICMD(macMask) (( cansat_dcmd_2 & (macMask) ) ? 1 : 0 )
#define TOGGLE_ICMD(macMask)  {cansat_dcmd_2 ^=  (macMask) ; }
#define SET_ICMD(macMask)     {cansat_dcmd_2 |=  (macMask) ; }
#define CLEAR_ICMD(macMask)   {cansat_dcmd_2 &= ~(macMask) ; }

//////////////////////////////////////////////////////////////////////////////	
// eeprom 을 다룬다.	
#define OP_EEPROM_ADDR     0x10
#define OP_TO_EEPROM()     { EEPROM.write(OP_EEPROM_ADDR, cansat_dcmd_0); } 
#define OP_FROM_EEPROM()   { cansat_dcmd_0 = EEPROM.read(OP_EEPROM_ADDR); }
	
//////////////////////////////////////////////////////////////////////////////
// 3개의 LED 를 다룬다. 
#define LED_MASK_SIGN   0x40 // Led 포트 설정에 따라 Toggling 한다.
#define LED_MASK_GS     0x80 // 지상국에서 명령을 수신했을 경우
#define LED_MASK_PL     0x20 // When Recv Data Port
#define LED_MASK_LOG    0x01 // 
//////////////////////////////////////////////////////////////////////////////
	
void CLEAR_LED (unsigned char mask){ unsigned char b = PINC ; b |=  (mask & 0xE0) ; PORTC = b ; }
void SET_LED   (unsigned char mask){ unsigned char b = PINC ; b &= ~(mask & 0xE0) ; PORTC = b ; }
void TOGGLE_LED(unsigned char mask){ unsigned char b = PINC ; b ^=  (mask & 0xE0) ; PORTC = b ; }
	
// LED 값을 쓴다.
void hyLed_PortSet()
{
	TOGGLE_LED( LED_MASK_SIGN) ;
	CLEAR_LED ( LED_MASK_GS  ) ;
        CLEAR_LED ( LED_MASK_PL  ) ;
//        CLEAR_LED ( LED_MASK_USER) ;
//      if( IS_NEED_OP(OP_MASK_USER) ) CLEAR_LED(LED_MASK_USER);  else SET_LED(LED_MASK_USER); 	
        CLEAR_LED ( LED_MASK_LOG ) ;
}

// AD convertor 를 다룬다.
unsigned char adc_mux = 0 ;
unsigned char ad_data[ADC_TLMS];// AD Converting 한 값을 저장한다.

unsigned char FILTER_MID_3(unsigned char b0, unsigned char b1, unsigned char b2 ) 
{ 
  if( b0 < b1 ) // 0 < 1 
  {
    if( b1 < b2 )              return b1 ;
    else if ( b0 < b2 )        return b2 ;
    else                       return b0 ;
  }
  else                                  // 1 < 0 
  {
    if( b0 < b2 )             return b0 ; 
    else if ( b1 < b2 )       return b2 ;
    else                      return b1 ;
  }
}

// ATLM 획득 함수
// Analog Telemetry GET : Interrupt Setting //
void hyATlm_Get(void)
{
  unsigned char b0, b1, b2 ; 
  int sensor_value         ;
  sensor_value = analogRead(adc_mux) ; // remove 
  sensor_value = analogRead(adc_mux) ; // remove 
  sensor_value = analogRead(adc_mux) ; b0 = ( sensor_value >> 2 ) & 0xFF ; // b0 = map(sensor_value,0,1023,0,255); // 10 bits ==> 8 bits ,, >> 4 
  sensor_value = analogRead(adc_mux) ; b1 = ( sensor_value >> 2 ) & 0xFF ; // b1 = map(sensor_value,0,1023,0,255); // 10 bits ==> 8 bits ,, >> 4 
  sensor_value = analogRead(adc_mux) ; b2 = ( sensor_value >> 2 ) & 0xFF ; // b2 = map(sensor_value,0,1023,0,255); // 10 bits ==> 8 bits ,, >> 4 
  ad_data[adc_mux] = FILTER_MID_3(b0,b1,b2);
  adc_mux = (adc_mux+1) & 0x07 ; 
}

// ATLM 지상 전송 함수
// 76 00 40 10 (ad_data)  * 8 //
void hyATlm_Send()
{
	unsigned char cs_atlm=0;
	Gs_Tx_QueueIn(0x76);
	Gs_Tx_QueueIn(0x00);
	Gs_Tx_QueueIn(0x40);
	Gs_Tx_QueueIn(0x10);
	Gs_Tx_QueueIn(ad_data[0]); cs_atlm+= ad_data[0];
	Gs_Tx_QueueIn(ad_data[1]); cs_atlm+= ad_data[1];
	Gs_Tx_QueueIn(ad_data[2]); cs_atlm+= ad_data[2];
	Gs_Tx_QueueIn(ad_data[3]); cs_atlm+= ad_data[3];
	Gs_Tx_QueueIn(ad_data[4]); cs_atlm+= ad_data[4];
	Gs_Tx_QueueIn(ad_data[5]); cs_atlm+= ad_data[5];
	Gs_Tx_QueueIn(ad_data[6]); cs_atlm+= ad_data[6];
	Gs_Tx_QueueIn(ad_data[7]); cs_atlm+= ad_data[7];
	Gs_Tx_QueueIn(cs_atlm)   ;
}

////////////////////////////////////////////////////////////////////////////////
// for 10 ms
// 16000000/1024 ==> 15625  // 15625==125*125==> 125   ==> 0x7D
// 11059200/1024 ==> 10800  // 10800/100     ==> 108   ==> 0x6C
// 9830400/1024 ==> 9600	// 9600/100      ==> 96    ==> 0x60 ==> 0x90

// TIMER 를 이용하여 SERVO를 제어해 보자.
// 20 msec 주기를 갖는다.
// 1.0 msec 왼쪽
// 1.5 msec 중간
// 2.0 msec 오른쪽

unsigned char pwm_0_cnt = 0 ;
unsigned char pwm_1_cnt = 0 ; 
/////////////////////////////////////////////////////////////////////////////////////////////


#if 0 // when 0.25msec ticks
#define PWM_20_MSEC_CNT  80   // 20 msec
#define PWM_CNT_NULL      0   //  no control
#define PWM_CNT_LEFT      5   //  1   msec   // over 4 + 1
#define PWM_CNT_CENTER    6   //  1.5 msec
#define PWM_CNT_RIGHT     7   //  2   msec  // under 8 - 1
#elif 0 // when 0.1 msec ticks
#define PWM_20_MSEC_CNT 200   // 20 msec
#define PWM_CNT_NULL      0   //  no control
#define PWM_CNT_LEFT     11   //  1   msec  // over  10 + 1
#define PWM_CNT_CENTER   15   //  1.5 msec  // 
#define PWM_CNT_RIGHT    19   //  2   msec  // under 20 - 1
#elif 0 // when 0.05 msec ticks
#define PWM_20_MSEC_CNT 400   // 20 msec
#define PWM_CNT_NULL      0   //  no control
#define PWM_CNT_START    15
#define PWM_CNT_LEFT     20   //  1   msec  // over  10 + 1
#define PWM_CNT_CENTER   30   //  1.5 msec  //
#define PWM_CNT_RIGHT    40   //  2   msec  // under 20 - 1
#define PWM_CNT_END      55
#elif 0 // when 0.25 msec ticks
#define PWM_20_MSEC_CNT 800   // 20 msec
#define PWM_CNT_NULL      0   //  no control
#define PWM_CNT_START    30
#define PWM_CNT_LEFT     41   //  1   msec  // over  10 + 1
#define PWM_CNT_CENTER   60   //  1.5 msec  //
#define PWM_CNT_RIGHT    79   //  2   msec  // under 20 - 1
#define PWM_CNT_END     140
#elif 0 // when 0.02 msec ticks ( 20 us )
#define PWM_20_MSEC_CNT 1000   // 20 msec
#define PWM_CNT_NULL       0   //  no control
#define PWM_CNT_LEFT      51   //  1   msec  // over   + 1
#define PWM_CNT_CENTER    75   //  1.5 msec  //
#define PWM_CNT_RIGHT     99   //  2   msec  // under  - 1
#else  // when arduino's count == degree
#define PWM_20_MSEC_CNT 800   // 20 msec
#define PWM_CNT_NULL      0   //  no control
#define PWM_CNT_START     0
#define PWM_CNT_LEFT      0   //    0 deg
#define PWM_CNT_CENTER   90   //   90 deg
#define PWM_CNT_RIGHT   180   //  180 deg
#define PWM_CNT_END     180
#endif

// 다음과 같이 작동하도록 하였다.
// 00 : NULL : 아무 작업 안한다.
// 01 : LEFT
// 11 : CENTER
// 10 : RIGHT  
inline void hyCANSAT_PWM_OPERATION() // 0.1 msec 단위로 처리한다.
{
	// pwm processing  20 msec processing
	time_servo_tick++ ;
	if ( PWM_20_MSEC_CNT <= time_servo_tick )	time_servo_tick = 0 ;
	
	
	// 0 == 0.5ms , 1 == 1ms , 2 == 1.5ms, 3 == 2 ms
	// PWM_0 COUNT 계산
	if( IS_NEED_ICMD(ICMD_MASK_PWM_0_L) ) {	pwm_0_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_0_R) ) ? PWM_CNT_CENTER : PWM_CNT_LEFT         ;}
	else	                              {	pwm_0_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_0_R) ) ? PWM_CNT_RIGHT  : cansat_byte_cmd[0]   ;}
	if( IS_NEED_ICMD(ICMD_MASK_PWM_1_L) ) {	pwm_1_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_1_R) ) ? PWM_CNT_CENTER : PWM_CNT_LEFT         ;}
	else	                              {	pwm_1_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_1_R) ) ? PWM_CNT_RIGHT  : cansat_byte_cmd[1]   ;}
	
	if( PWM_CNT_START <= pwm_0_cnt && time_servo_tick <= pwm_0_cnt && time_servo_tick <= PWM_CNT_END )    { cansat_dcmd_1 |= 0x80 ; } else { cansat_dcmd_1 &= 0x7F;}
	if( PWM_CNT_START <= pwm_1_cnt && time_servo_tick <= pwm_1_cnt && time_servo_tick <= PWM_CNT_END )    { cansat_dcmd_1 |= 0x40 ; } else { cansat_dcmd_1 &= 0xBF;}
	PORTA = cansat_dcmd_1 ; 
}

//// 1ms 단위 처리를 수행한다.
//inline void hyCANSAT_MSEC_OPERATION() // 1 msec 단위로 처리한다.
//{
//	// msec 단위로 시간을 증가한다. 
//	time_msec++   ;
//	
//	// wait ms processing
//	if( 0 < time_wait_msec ) time_wait_msec-- ;
//	
//	// 1000ms 즉 1초 단위의 Operation 을 수행한다.
////	if( 1000 < time_msec ) // 1 SEC // 약 80개가 더 많네..
//	if( 1080 < time_msec ) // 1 SEC // 약 80개가 더 많네..
//	{
//		time_msec = 0 ;
//		time_run++    ;
//		time_sec++    ; // 1 sec
//	}
//}

//   1  msec : 0xAA
// 0.5  msec : 0xD5 
// 0.25 msec : 0xEC
// 0.20 msec : 0xEF
// 0.10 msec : 0xF8 : when TCCR0 == 5 // 100 us
// 0.05 msec : 0xF? : when TCCR0 == 5 // 50  us
// 0.02 msec : 0xF9 : when TCCR0 == 3 // 20  us
// 0.01 msec : 0x?? : when TCCR0 == 3 // 10  us 
//SIGNAL(TIMER0_OVF_vect)
//{
////	TCNT0 = 0xAA ;	// 클럭에 따른 변경 필요 : 1ms 를 생성해 보자.   
////	TCNT0 = 0xD5 ;	// 클럭에 따른 변경 필요 : 0.5ms 를 생성해 보자
////	TCNT0 = 0xEC ;	// 클럭에 따른 변경 필요 : 0.25ms 를 생성해 보자
////	TCNT0 = 0xEF ;	// 클럭에 따른 변경 필요 : 0.2ms 를 생성해 보자
////	TCNT0 = 0xF8 ;	// 클럭에 따른 변경 필요 : 0.1ms 를 생성해 보자  , TCCR0 == 5
//	TCNT0 = 0xFC ;	// 클럭에 따른 변경 필요 : 0.05ms 를 생성해 보자 , TCCR0 == 5
////	TCNT0 = 0xFE ;	// 클럭에 따른 변경 필요 : 0.025ms를 생성해 보자 , TCCR0 == 5 .. image 오류발생하고 있음.
////	TCNT0 = 0xF9 ;	// 클럭에 따른 변경 필요 : 0.02ms 를 생성해 보자 , TCCR0 == 3
//	
//	// increase time_msec
//	time_tick++ ; // 초기값이 0 이면, 1 ==> 0 ==> 1 // 1과 0을 반복한다.
//	if( 20 <= time_tick  ) { time_tick = 0 ; hyCANSAT_MSEC_OPERATION() ; } // 1msec 에 한번씩 수행
//	if( !IS_NEED_OP(OP_MASK_USER  ) ) hyCANSAT_PWM_OPERATION()  ;          // NOT USER DEFINED ==> PWM OPERATION
////   cansat_dcmd_1 ^= 0x80 ;	PORTA = cansat_dcmd_1 ;
//}


// DTLM 처리 함수
// 1. Flag Toggle
// 2. PORT 에 데이터 설정

void hyDTlm_OP()
{
	TOGGLE_ICMD(ICMD_MASK_SIGN); 
	// 아래 부분은 사용자가 바꾸어도 괜찮을 것이다.
	
	unsigned char pf = PINF ;        // analog data status get
	cansat_dcmd_3 = 0 ;
	if( pf & 0x01 ) cansat_dcmd_3 |= 0x80 ;
	if( pf & 0x02 ) cansat_dcmd_3 |= 0x40 ;
	if( pf & 0x04 ) cansat_dcmd_3 |= 0x20 ;
	if( pf & 0x08 ) cansat_dcmd_3 |= 0x10 ;
	if( pf & 0x10 ) cansat_dcmd_3 |= 0x08 ;
	if( pf & 0x20 ) cansat_dcmd_3 |= 0x04 ;
	if( pf & 0x40 ) cansat_dcmd_3 |= 0x02 ;
	if( pf & 0x80 ) cansat_dcmd_3 |= 0x01 ;
		
	PORTA = cansat_dcmd_1 ; // digital setting 
}

// DTLM 지상 전송 함수
#if 0 // OLD VERSION
// 76 00 48 02 DTLM(2)  CS(1)  //
void hyDTlm_Send()
{
	Gs_Tx_QueueIn(0x76);
	Gs_Tx_QueueIn(0x00);
	Gs_Tx_QueueIn(0x48);
	Gs_Tx_QueueIn(0x02);
	Gs_Tx_QueueIn(cansat_dcmd_0);
	Gs_Tx_QueueIn(cansat_dcmd_1);
	Gs_Tx_QueueIn((cansat_dcmd_0 ^ cansat_dcmd_1));
}
#else // new version
// 76 00 48 04 DTLM(4)  CS(1)  //
void hyDTlm_Send()
{
	Gs_Tx_QueueIn(0x76);
	Gs_Tx_QueueIn(0x00);
	Gs_Tx_QueueIn(0x48);
	Gs_Tx_QueueIn(0x04);
	Gs_Tx_QueueIn(cansat_dcmd_0);
	Gs_Tx_QueueIn(cansat_dcmd_1);
	Gs_Tx_QueueIn(cansat_dcmd_2);
	Gs_Tx_QueueIn(cansat_dcmd_3);
	Gs_Tx_QueueIn((cansat_dcmd_0 ^ cansat_dcmd_1 ^ cansat_dcmd_2 ^ cansat_dcmd_3));
}
#endif
////////////////////////////////////////////////////////////////////////

// read one byte processing //
void hyGps_RecvByte(unsigned char b);
void hyCamera_Receive_Size(unsigned char rx);
void hyCamera_Receive_Image(unsigned char rx);

// UART 통신 처리 메인 함수
// 1. Pl_Rx_Queue 처리 (모드별 처리 수행)
// 2. Gs_Rx_Queue 처리 (지상 명령 처리 수행 )
// 3. Gs_Tx_Queue 처리 (지상 전송 명령 처리 )
// 4. Pl_Tx_Queue 처리 (내부 명령 전송 처리 )
unsigned char hyTransUart()
{
	unsigned char b ;
	unsigned char worked =0;

	// Pl_Rx ==> Gs_Tx
	if( PL_PORT.available() ) { Pl_Rx_QueueIn(PL_PORT.read()); } // When no interrupt processing ... 

	if( Pl_Rx_QueueIsNotEmpty()  )
	{
		worked = 1 ;
		Pl_Rx_QueueOut(b);

		switch( hyRxMode )
		{
			case RxMode_CAMERA	 :  Gs_Tx_QueueIn(b); break;
			case RxMode_CAMERA_SIZE  :  Gs_Tx_QueueIn(b); hyCamera_Receive_Size(b) ;  break;
			case RxMode_CAMERA_IMAGE :  Gs_Tx_QueueIn(b); hyCamera_Receive_Image(b);  break;
			case RxMode_GPS          :  hyGps_RecvByte(b); break;
			case RxMode_IMU          :  hyImu_RecvByte(b); break;
			default                  :  Gs_Tx_QueueIn(b) ; break;
		}
	}

//	if( GS_PORT.available() ) { Gs_Rx_QueueIn(PL_PORT.read()); } // When no interrupt processing ...  // use interrupt OK ..
	// Gs_Rx ==> Pl_Tx
	if( Gs_Rx_QueueIsNotEmpty() )
	{
		worked = 1 ;
		Gs_Rx_QueueOut(b) ;
		hyGs_RecvByte(b);
	}

	// Gs_Tx ==> Gs_Tx PORT
	if( Gs_Tx_QueueIsNotEmpty()  )
	{
		worked = 1 ;
		Gs_Tx_QueueOut(b);
		hyGsTxByte(b);
	}

	// Pl_Tx ==> Pl_Tx PORT
	if( Pl_Tx_QueueIsNotEmpty()  )
	{
		worked = 1 ;
		Pl_Tx_QueueOut(b);
		hyPlTxByte(b);
	}
	return worked ; // 0 means do nothing  //
}


// 블루투스 초기화 함수
// 1. UART1 초기화        : 9600
// 2. BT 통신 속도 초기화  : AT+UARTCONFIG,115200,N,1
// 3. BT MODE 초기화      : AT+BTMODE,3
// 4. BT 재 설정          : ATZ

void hyBT_STOP()
{
	hyGsSetTxByte('+');
	hyGsSetTxByte('+');
	hyGsSetTxByte('+');
	hyGsSetTxByte(0x0D);
	hyGsSetTxByte(0x0A);
}

void hyBT_115200()
{
	hyGsSetTxByte('a');
	hyGsSetTxByte('t');
	hyGsSetTxByte('+');
	hyGsSetTxByte('u');
	hyGsSetTxByte('a');
	hyGsSetTxByte('r');
	hyGsSetTxByte('t');
	hyGsSetTxByte('c');
	hyGsSetTxByte('o');
	hyGsSetTxByte('n');
	hyGsSetTxByte('f');
	hyGsSetTxByte('i');
	hyGsSetTxByte('g');
	hyGsSetTxByte(',');
	hyGsSetTxByte('1');
	hyGsSetTxByte('1');
	hyGsSetTxByte('5');
	hyGsSetTxByte('2');
	hyGsSetTxByte('0');
	hyGsSetTxByte('0');
	hyGsSetTxByte(',');
	hyGsSetTxByte('N');
	hyGsSetTxByte(',');
	hyGsSetTxByte('1');
	hyGsSetTxByte(',');
	hyGsSetTxByte('0');
	hyGsSetTxByte(0x0D);
	hyGsSetTxByte(0x0A);
}


void hyBT_MODE()
{
	hyGsSetTxByte('a');
	hyGsSetTxByte('t');
	hyGsSetTxByte('+');
	hyGsSetTxByte('b');
	hyGsSetTxByte('t');
	hyGsSetTxByte('m');
	hyGsSetTxByte('o');
	hyGsSetTxByte('d');
	hyGsSetTxByte('e');
	hyGsSetTxByte(',');
	hyGsSetTxByte('3');
	hyGsSetTxByte(0x0D);
	hyGsSetTxByte(0x0A);
}

void hyBT_ATS()
{
	hyGsSetTxByte('a');
	hyGsSetTxByte('t');
	hyGsSetTxByte('s');
	hyGsSetTxByte('1');
	hyGsSetTxByte('0');
	hyGsSetTxByte('=');
	hyGsSetTxByte('1');
	hyGsSetTxByte(0x0D);
	hyGsSetTxByte(0x0A);
}
void hyBT_ATZ()
{
	hyGsSetTxByte('a');
	hyGsSetTxByte('t');
	hyGsSetTxByte('z');
	hyGsSetTxByte(0x0D);
	hyGsSetTxByte(0x0A);
}

void hyBluetooth_Init()
{
	GS_UART_INIT();

       GS_PORT_SPEED_SET(9600)    ;
	hyTransUartWait(WAIT_CNT_BT_STEP) ;
	
	hyBT_STOP()  ;  hyTransUartWait(WAIT_CNT_BT_STEP) ;
	hyBT_MODE()  ;	hyTransUartWait(WAIT_CNT_BT_STEP) ;
	hyBT_115200();	hyTransUartWait(WAIT_CNT_BT_STEP) ;
//	hyBT_ATS()   ;	hyTransUartWait(WAIT_CNT_BT_STEP) ;
	hyBT_ATZ()   ;	hyTransUartWait(WAIT_CNT_BT_STEP) ;
	
	
        GS_PORT_SPEED_SET(GS_TX_SPEED);
	
//	hyBT_MODE()  ;	hyTransUartWait(WAIT_CNT_BT_STEP) ;
//	hyBT_ATZ()   ;	hyTransUartWait(WAIT_CNT_BT_STEP) ;
	//
        GS_PORT_SPEED_SET(GS_TX_SPEED);
}



// SIZE :: when received 76 00 34 00 04 00 00 XX YY //
unsigned char sCameraReceive_SizeFlag = 0 ;
unsigned char hyCameraImageSize[2];


// DATA :: when received 76 00 34 00 04 00 00 XX YY //
void hyCamera_Reset()
{
	Pl_Tx_QueueFlush() ;
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x26);
	Pl_Tx_QueueIn(0x00);
}

void hyCamera_Reset_Set()
{
	Pl_Tx_QueueFlush() ;
	hyPlTxByte(0x56);
	hyPlTxByte(0x00);
	hyPlTxByte(0x26);
	hyPlTxByte(0x00);
}


void hyGs_Reset_Reply()
{
	Gs_Tx_QueueIn(0x76);
	Gs_Tx_QueueIn(0x00);
	Gs_Tx_QueueIn(0x26);
	Gs_Tx_QueueIn(0x00);
}


// 카메라 Sleep 함수
void hyCamera_Sleep()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x3E);
	Pl_Tx_QueueIn(0x03);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x01);
}

// 카메라 Wake 함수
void hyCamera_Wake()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x3E);
	Pl_Tx_QueueIn(0x03);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x00);
}


// 카메라 전송 속도 9600 설정 요청 함수
void hyCamera_9600_bps()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x24);
	Pl_Tx_QueueIn(0x03);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0xAE);
	Pl_Tx_QueueIn(0xC8);
}

// 카메라 이미지 320 모드 설정 요청 함수 (직접명령)
void hyCamera_38400_bps()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x24);
	Pl_Tx_QueueIn(0x03);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x2A);
	Pl_Tx_QueueIn(0xF2);
}

// 카메라 전송 속도 115200 설정 요청 함수
void hyCamera_115200_bps()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x24);
	Pl_Tx_QueueIn(0x03);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x0D);
	Pl_Tx_QueueIn(0xA6);
}

// 카메라 전송 속도 115200 설정 요청 함수 ( 직접 명령 )
void hyCamera_115200_bps_Set()
{
	Pl_Tx_QueueFlush();
	hyPlTxByte(0x56);
	hyPlTxByte(0x00);
	hyPlTxByte(0x24);
	hyPlTxByte(0x03);
	hyPlTxByte(0x01);
	hyPlTxByte(0x0D);
	hyPlTxByte(0xA6);
}

// 카메라 이미지 160 모드 설정 요청 함수 
void hyCamera_160()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x31);
	Pl_Tx_QueueIn(0x05);
	Pl_Tx_QueueIn(0x04);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x19);
	Pl_Tx_QueueIn(0x22);
	
	WAIT_CNT_CAMERA_IMAGE = WAIT_CNT_CAMERA_IMAGE_160 ;
}

// 카메라 이미지 320 모드 설정 요청 함수
void hyCamera_320()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x31);
	Pl_Tx_QueueIn(0x05);
	Pl_Tx_QueueIn(0x04);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x19);
	Pl_Tx_QueueIn(0x11);

	WAIT_CNT_CAMERA_IMAGE = WAIT_CNT_CAMERA_IMAGE_320 ;
	
}

// 카메라 이미지 640 모드 설정 요청 함수
void hyCamera_640()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x31);
	Pl_Tx_QueueIn(0x05);
	Pl_Tx_QueueIn(0x04);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x19);
	Pl_Tx_QueueIn(0x00);

	WAIT_CNT_CAMERA_IMAGE = WAIT_CNT_CAMERA_IMAGE_640 ;
	
}

// 카메라 이미지 Stop 요청 함수
void hyCamera_Stop()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x36);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x03);
}

// 카메라 이미지 Take 요청 함수
void hyCamera_Take()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x36);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x00);
}

// 카메라 이미지 Resume 요청 함수
void hyCamera_Resume()
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x36);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x02);
}

unsigned char Char_to_OSD(char ich)
{
  if( '0' <= ich && ich <= '9' ) return ich - '0'      ; // 0  : 10
  if( 'A' <= ich && ich <= 'Z' ) return ich - 'A' + 10 ; // 10 : 10 + 26      
  if( 'a' <= ich && ich <= 'z' ) return ich - 'a' + 36 ; // 36 : 10 + 26 + 26 => 62
  switch ( ich ) 
  {
    case '-' : return 62 ;
    case '_' : return 63 ;
    case ':' : return 64 ;
    case '.' : return 65 ;
    case '/' : return 66 ;
    case '*' : return 67 ;
    case '(' : return 68 ;
    case ')' : return 69 ;
    case '[' : return 70 ;
    case ']' : return 71 ;
    case '@' : return 72 ;
    case '!' : return 73 ;
    case '+' : return 74 ;
    case '|' : return 75 ;
    case '\\': return 76 ;
    case '#' : return 77 ;
    case '=' : return 78 ;
  }
  return 79 ;
}

#if 0
void hyCamera_Osd_Add_Char(char *istr)
{
        unsigned char slen ;  for( slen = 0 ; slen < 14 && *istr ; slen++ ) ; 
        unsigned char i ; 
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x45);
	Pl_Tx_QueueIn(slen+1); // data_length
	Pl_Tx_QueueIn(slen); // chracter number : max 14
	Pl_Tx_QueueIn(0x00); // starting address [x][y] = x 2 bits y 5 bits 
        for ( i = 0 ; i < slen ; i++ )  
          Pl_Tx_QueueIn(Char_to_OSD(istr[i])); // characters
}
#else
void hyCamera_Osd_Add_Char(char *istr)
{
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x45);
	Pl_Tx_QueueIn(0x07); // data_length
	Pl_Tx_QueueIn(0x06); // chracter number : max 14
	Pl_Tx_QueueIn(0x24); // starting address [x][y] = x 2 bits y 5 bits 
        Pl_Tx_QueueIn(0x1F); // characters
        Pl_Tx_QueueIn(0x2C); // characters
        Pl_Tx_QueueIn(0x30); // characters
        Pl_Tx_QueueIn(0x2C); // characters
        Pl_Tx_QueueIn(0x26); // characters
        Pl_Tx_QueueIn(0x35); // characters
        Pl_Tx_QueueIn(0x32); // characters
}
#endif


// 카메라 이미지 크기 요청 함수
void hyCamera_Size()
{
	sCameraReceive_SizeFlag = 0 ;
	Pl_Tx_QueueIn(0x56);
	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x34);
	Pl_Tx_QueueIn(0x01);
	Pl_Tx_QueueIn(0x00);
}


// 카메라 이미지 수신 요청 함수
void hyCamera_ImageGet(unsigned char b0,unsigned char b1,unsigned char b2,unsigned char b3)
{
	Pl_Tx_QueueIn(0x56);	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(0x32);	Pl_Tx_QueueIn(0x0C);
	Pl_Tx_QueueIn(0x00);	Pl_Tx_QueueIn(0x0A);
	Pl_Tx_QueueIn(0x00);	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(b0)  ;	Pl_Tx_QueueIn(b1)  ;
	Pl_Tx_QueueIn(0x00);	Pl_Tx_QueueIn(0x00);
	Pl_Tx_QueueIn(b2);	Pl_Tx_QueueIn(b3);
	Pl_Tx_QueueIn(0x00);	Pl_Tx_QueueIn(0x0A);
}

// 카메라 이미지 크기 수신 함수
// 카메라로 부터 영상 키기 신호를 수신 받으면
// sCameraImageSize 에 값 설정
// Receive : 76 00 34 00 04 00 00 XX YY //
void hyCamera_Receive_Size(unsigned char rx)
{
	static unsigned char sSizeMode = 0 ; 

	switch( sSizeMode )
	{
		case 0 :	sSizeMode =  ( rx == 0x76 ) ? 1  : 0  ; break; // 76 
		case 1 :	sSizeMode =  ( rx == 0x00 ) ? 2  : (( rx == 0x76 ) ? 1 : 0)  ; break; // 00 
		case 2 :	sSizeMode =  ( rx == 0x34 ) ? 3  : (( rx == 0x76 ) ? 1 : 0)  ; break; // 34
		case 3 :	sSizeMode =  ( rx == 0x00 ) ? 4  : (( rx == 0x76 ) ? 1 : 0)  ; break; // 00 
		case 4 :	sSizeMode =  ( rx == 0x04 ) ? 5  : (( rx == 0x76 ) ? 1 : 0)  ; break; // 04 
		case 5 :	sSizeMode =  ( rx == 0x00 ) ? 6  : (( rx == 0x76 ) ? 1 : 0)  ; break; // 00 
		case 6 :	sSizeMode =  ( rx == 0x00 ) ? 7  : (( rx == 0x76 ) ? 1 : 0)  ; break; // 00 
		case 7 :	hyCameraImageSize[0] = rx ; sSizeMode = 8 ; break; // data 
		case 8 :	hyCameraImageSize[1] = rx ; sSizeMode = 0 ; 
				sCameraReceive_SizeFlag = 1 ; 
                                hyLOG_CAMERA_SIZE(); // 
				break; // data 
		default:	sSizeMode = 0 ; sCameraReceive_SizeFlag = 0 ;
	}
}



// 카메라 이미지 수신 함수
// 카메라로 부터 영상 종료 신호를 수신 받으면
// img_recv_flag 를 1로 설정
// Receive : FF D9 76 00 32 00 //
unsigned char img_recv_flag=0;
void hyCamera_Receive_Image(unsigned char rx)
{
	static unsigned char sImageMode=0;

	switch( sImageMode )
	{
		case 0 :	sImageMode =  ( rx == 0xFF ) ? 1  : 0  ; break; // FF 
		case 1 :	sImageMode =  ( rx == 0xD9 ) ? 2  : (( rx == 0xFF ) ? 1 : 0)  ; break; // D9 
		case 2 :	sImageMode =  ( rx == 0x76 ) ? 3  : (( rx == 0xFF ) ? 1 : 0)  ; break; // 76
		case 3 :	sImageMode =  ( rx == 0x00 ) ? 4  : (( rx == 0xFF ) ? 1 : 0)  ; break; // 00 
		case 4 :	sImageMode =  ( rx == 0x32 ) ? 5  : (( rx == 0xFF ) ? 1 : 0)  ; break; // 32 
		case 5 :	sImageMode =  ( rx == 0x00 ) ? 6  : (( rx == 0xFF ) ? 1 : 0)  ;
				if( rx == 0x00 ) 
                                { img_recv_flag = 1 ;
                                  hyLOG_CAMERA_IMAGE();
                                }
				break; // 00 
		default:	sImageMode = 0 ; 
	}
}
// 카메라 초기화 함수
// 1. UART0 , UART1 초기화
// 2. 38400 , 115200 으로 설정
// 3. 카메라 Reset 처리
// 4. 카메라 320 모드로 설정
// 5. 카메라를 115200 모드로 재 설정
// 6. UART0, 115200 속도로 설정

void hyCamera_PictureSize()
{
	switch( (IS_NEED_OP(OP_MASK_CAMERA_160)<<1) + IS_NEED_OP(OP_MASK_CAMERA_640) )
	{
		case 1 : hyCamera_640(); break;
		case 2 : hyCamera_160(); break;
		case 0 : 
		case 3 : 
		default: hyCamera_320(); break;
	}
}

void hyCamera_Init()
{
       volatile unsigned short wait_cnt ;
       hyLOG_CAMERA_INIT();

	PL_UART_INIT();
	GS_UART_INIT();

        PL_PORT_SPEED_SET(38400) ;
        GS_PORT_SPEED_SET(GS_TX_SPEED);

	hyCamera_Reset()         ;	hyTransUartWait(WAIT_CNT_CAMERA_INIT_STEP);
	hyCamera_PictureSize()   ;	hyTransUartWait(WAIT_CNT_CAMERA_INIT_STEP);
	for( unsigned char i = 0 ; i < 10 ; i++ )
	{
		hyCamera_115200_bps();	hyTransUartWait(WAIT_CNT_CAMERA_INIT_STEP);
	}

        PL_PORT_SPEED_SET(115200);
	
	// hypark : 2013.06.03 appended
	hyCamera_PictureSize()    ;	hyTransUartWait(WAIT_CNT_CAMERA_INIT_STEP);
	
	// SLEEP Mode 추가 //
//	hyCamera_Sleep(); hyTransUartWait(WAIT_CNT_CAMERA_INIT_STEP);
}


// 지상국 데이터 수신 처리 함수
// 한 문자 입력에 대한 처리 수행하며
// 56 00 .......  프로토콜이 들어오는 것을 처리하도록 함.
// TCMD : 56 00  48 : 00 ID CMD CS
/////////////////////////////////////////////////////////////////////////////////////////////
// 56 00 CMD LEN .... 

// BITMAP CMD 
// 40 08 XX XX XX XX XX XX XX XX XX XX XX XX XX XX ( 블럭(128BYTE) 당 1 bit 사용, 8*16*128 = 16.384 임 )
// 76 00 40 bidx ....   송신 예정                                                                                  

void hyGs_RecvByte(unsigned char b)
{
	static unsigned char sRecvMode=0;
	static unsigned char cRecvCmd[4]={0,0,0,0};
	//unsigned char   tb_0=0                    ;
	//unsigned char   tb_1=0                    ;
//	SET_OP(ICMD_MASK_GS);
	// 56 00 .... 
	switch( sRecvMode ) 
	{
		case 0x00 : sRecvMode = ( b == 0x56 ) ?  0x01 : 0 ;                       break; // 56
		case 0x01 : sRecvMode = ( b == 0x00 ) ?  0x02 : (( b == 0x56 ) ? 1 : 0) ; break; // 00
		case 0x02 : sRecvMode = ( b == 0x48 ) ?  0x10 : (( b == 0x56 ) ? 1 : 0) ; break; // 48	
		
		case 0x10 : sRecvMode = ( b == 0x00 ) ?  sRecvMode+1 : (( b == 0x56 ) ? 1 : 0) ; break; // 00
		case 0x11 : sRecvMode++ ; cRecvCmd[0] = b ; break; // ID
		case 0x12 : sRecvMode++ ; cRecvCmd[1] = b ; break; // SET
		case 0x13 : sRecvMode= 0; cRecvCmd[2] = b ;        // CSUM
					if( 0 <= cRecvCmd[0] && cRecvCmd[0] < 64 ) // when Digital Bit Command
					{
						if( 0 <= cRecvCmd[1] && cRecvCmd[1] <= 1 )
						{
							if( (cRecvCmd[0] ^ cRecvCmd[1]) == cRecvCmd[2]) // XOR
							{
								SET_ICMD(ICMD_MASK_GS); SET_LED(LED_MASK_GS) ; 
							
								if( 0 <= cRecvCmd[0] && cRecvCmd[0] < 8  ) // flag
								{
									if( cRecvCmd[1] ) 	cansat_dcmd_0 |=  MASK_BIT_OR[cRecvCmd[0]];
									else                cansat_dcmd_0 &= ~MASK_BIT_OR[cRecvCmd[0]];
									OP_TO_EEPROM();
								}
							
								if( 8 <= cRecvCmd[0] && cRecvCmd[0] < 16 ) // 직접 명령 수행
								{
									if( cRecvCmd[1] ) 	cansat_dcmd_1 |=  MASK_BIT_OR[cRecvCmd[0] & 0x07];
									else                cansat_dcmd_1 &= ~MASK_BIT_OR[cRecvCmd[0] & 0x07];
									PORTA = cansat_dcmd_1 ;
								}
							
								if( 16 <= cRecvCmd[0] && cRecvCmd[0] < 24 ) // 내부 설정 명령 수행
								{
									if( cRecvCmd[1] ) 	cansat_dcmd_2 |=  MASK_BIT_OR[cRecvCmd[0] & 0x07];
									else                cansat_dcmd_2 &= ~MASK_BIT_OR[cRecvCmd[0] & 0x07];
	//								ICMD_TO_EEPROM(); // 저장하지 말자
								}
							
								if( 24 <= cRecvCmd[0] && cRecvCmd[0] < 32 ) // but not used 
								{
									if( cRecvCmd[1] ) 	cansat_dcmd_3 |=  MASK_BIT_OR[cRecvCmd[0] & 0x07];
									else                cansat_dcmd_3 &= ~MASK_BIT_OR[cRecvCmd[0] & 0x07];
								}
							}
						}					
					}
					else if ( 64 <= cRecvCmd[0] && cRecvCmd[0] < 64+8) // when 8 bit data value
					{
						if( (cRecvCmd[0] ^ cRecvCmd[1]) == cRecvCmd[2]) // XOR
						{
							SET_ICMD(ICMD_MASK_GS); SET_LED(LED_MASK_GS) ;
							cansat_byte_cmd[cRecvCmd[0]-64] = cRecvCmd[1];
						}
					}
					break;

		default   : sRecvMode = ( b == 0x56 ) ?  1 : 0 ; break;
	}
}

// IMU 데이터 수신 처리 함수
// 한 문자 입력에 대한 처리 수행하며
// *값...\n\a 프로토콜이 들어오는 것을 처리하도록 함.
////////////////////////////////////////////////
// * .... /a/d
// 개별 6 바이트에서 7바이트 이므로
// 7 바이트 + 10 개 하면 70바이트네...
#define IMU_DATA_RETRY_CNT 1
#define IMU_DATA_SIZE 128
unsigned char imu_data[IMU_DATA_SIZE];
unsigned char imu_data_idx=0;
unsigned char imu_recv_flag=0;

void hyImu_RecvByte(unsigned char b)
{
	static unsigned char imu_i_state=0 ;
	unsigned char csum_imu, csum_high, csum_low ; 
	if( b == '*' )									  
	{ 
		imu_data[0] = b ; 
		imu_data_idx =1 ; 
		imu_data[imu_data_idx]=0;
	}
	else if( 0 < imu_data_idx  && imu_data_idx < IMU_DATA_SIZE-2 ) 
	{ 
		if( b == '\n' || b == '\a' )	// when receive the IMU DATA //
		{
			if( imu_data[0] == '*' )
			{
				if( imu_i_state == 0 )  // remove first data for sync //
				{
					imu_i_state++   ;
					for( unsigned char i = 0 ; i < IMU_DATA_SIZE ; i++ ) imu_data[i] = 0 ;
					imu_data_idx = 0; 
				}
				else if( imu_i_state != 0 ) 
				{
					// receive 3rd times //
					imu_i_state =  ( imu_i_state < IMU_DATA_RETRY_CNT ) ? imu_i_state+1 : 0 ;
//					if( imu_i_state == 0 ) imu_recv_flag = 1 ;
					if( imu_i_state == 0 ){ imu_recv_flag = 1 ; hyLOG_IMU_DATA(); }

					Gs_Tx_QueueIn(0x76) ;
					Gs_Tx_QueueIn(0x00) ;
					Gs_Tx_QueueIn(0x60) ;
					Gs_Tx_QueueIn(0x00) ;
					csum_imu=0   ;
					for( unsigned char idx=0 ; idx < imu_data_idx ; idx++ )
					{
						Gs_Tx_QueueIn(imu_data[idx]) ;
						csum_imu += imu_data[idx]  ;
					}
					Gs_Tx_QueueIn(',' ) ;				    // , for csum
					csum_high = ( csum_imu >> 4 ) & 0x0F ;
					csum_low  = csum_imu & 0x0F ;

					csum_high += ( csum_high < 10 ) ? '0' : 'a'-10  ; 
					csum_low  += ( csum_low  < 10 ) ? '0' : 'a'-10  ; 

					Gs_Tx_QueueIn(csum_high);	//   add for csum
					Gs_Tx_QueueIn(csum_low );	//   add for csum
					Gs_Tx_QueueIn(0x0D) ;
					Gs_Tx_QueueIn(0x0A) ;

					for( unsigned char i = 0 ; i < IMU_DATA_SIZE ; i++ ) imu_data[i] = 0 ;
					imu_data_idx = 0; 
				}
			}
		}
		else
		{
			imu_data[imu_data_idx] = b ; 
			imu_data_idx++; 
			imu_data[imu_data_idx]=0;
		}
	}
	else
	{ 
		imu_data[0]  = 0; 
		imu_data_idx = 0; 
	} 
}

// GPS 데이터 수신 처리 함수
// 한 문자 입력에 대한 처리 수행하며
// $GPGGA 프로토콜이 들어오는 것을 처리하도록 함.
////////////////////////////////////////////////
// $GPRMC,112320.000,A,3622.3375,N,12721.9571,E,0.03,0.00,010212,,,D*61
// $GPGGA,HHMMSS.000,.....      
unsigned char gps_data[96];
unsigned char gps_data_idx=0;
unsigned char gps_recv_flag=0;
unsigned long gps_time_t0=0;
unsigned long gps_time_t1=0;

void hyGps_RecvByte(unsigned char b)
{
	unsigned char csum_gps, csum_high, csum_low ;
	if( b == '$' )									  
	{ 
		gps_data[0] = b ; 
		gps_data_idx =1 ; 
		gps_data[gps_data_idx]=0;
	}
	else if( 0 < gps_data_idx  && gps_data_idx < 94 ) 
	{ 
		if( b == '\n' || b == '\a' )	// when receive the GPSR DATA //
		{

			if( gps_data[0] == '$' )
			if( gps_data[1] == 'G' )
			if( gps_data[2] == 'P' )
			if( gps_data[3] == 'G' )
			if( gps_data[4] == 'G' )
			if( gps_data[5] == 'A' )
			if( gps_data[6] == ',' )
			{
				gps_recv_flag = 1;
				Gs_Tx_QueueIn(0x76);
				Gs_Tx_QueueIn(0x00);
				Gs_Tx_QueueIn(0x58);
				Gs_Tx_QueueIn(0x00);
				Gs_Tx_QueueIn(gps_data[0]) ;
				csum_gps = 0 ; 
				unsigned char idx     ;
				for( idx=1 ; idx < gps_data_idx && gps_data[idx] != '*' ; idx++ )
				{
					Gs_Tx_QueueIn(gps_data[idx]) ;
					csum_gps ^= gps_data[idx] ; 
				}
				for(       ; idx < gps_data_idx ; idx++ )
				{
					Gs_Tx_QueueIn(gps_data[idx]) ;
				}
				// add csum
				Gs_Tx_QueueIn('*') ;
				csum_high = ( csum_gps >> 4 ) & 0x0F ;
				csum_low  = csum_gps & 0x0F ;

				csum_high += ( csum_high < 10 ) ? '0' : 'a'-10  ; 
				csum_low  += ( csum_low  < 10 ) ? '0' : 'a'-10  ; 

				Gs_Tx_QueueIn(csum_high);	//   add for csum
				Gs_Tx_QueueIn(csum_low );	//   add for csum


				Gs_Tx_QueueIn(0x0D);
				Gs_Tx_QueueIn(0x0A);

                                hyLOG_GPS_DATA() ; // use gps_data, gps_data_idx 
                                
                                // init gps data
				gps_data_idx = 0 ; 
				gps_data[0]  = 0 ;
				
				gps_time_t0 = gps_data[ 7] - '0';	gps_time_t1  = gps_time_t0 *  36000 ;
				gps_time_t0 = gps_data[ 8] - '0';	gps_time_t1 += gps_time_t0 *  3600  ;
				gps_time_t0 = gps_data[ 9] - '0';	gps_time_t1 += gps_time_t0 *  600   ;
				gps_time_t0 = gps_data[10] - '0';	gps_time_t1 += gps_time_t0 *  60    ;
				gps_time_t0 = gps_data[11] - '0';	gps_time_t1 += gps_time_t0 *  10    ;
				gps_time_t0 = gps_data[12] - '0';	gps_time_t1 += gps_time_t0 ;
				time_sec    = gps_time_t1  ;
			}
		}
		else
		{
			gps_data[gps_data_idx] = b ; 
			gps_data_idx++; 
			gps_data[gps_data_idx]=0;
		}
	}
	else
	{ 
		gps_data[0]  = 0; 
		gps_data_idx = 0; 
	} 
}





unsigned short watch_dog_cnt=0;

// 한번의 통신처리 기능 수행
#define hyTransUartOne() { if( hyTransUart() ) { watch_dog_cnt++; } }

// 주어진 횟수 만큼 통신처리 기능 수행
void hyTransUartWait(unsigned long cnt)
{
	for( unsigned long loop_cnt = 0 ; loop_cnt < cnt ; loop_cnt++ )
		hyTransUartOne() ;
}

// 특정 신호를 유지하거나, 주어진 횟수 만큼 통신처리 기능 수행 
void hyTransUartUntil(unsigned char *flag, unsigned char value,unsigned long cnt)
{
	for( unsigned long loop_cnt = 0 ; loop_cnt < cnt && *flag == value; loop_cnt++ )
		hyTransUartOne() ;
}

//////////////////////////////////////////////////////////
// GPS 메인 동작 함수
// 1. UART 모드 설정
// 2. GPS 로부터 데이터 획득
// 3. GPS 데이터 지상 전송
void hyCANSAT_GPS_OPERATION()
{
	gps_recv_flag = 0 ;
	hyRxMode_Set(RxMode_GPS) ;
	hyTransUartUntil(&gps_recv_flag,0,WAIT_CNT_GPS );
	hyTransUartWait(WAIT_CNT_SEND);
}

//////////////////////////////////////////////////////////
// IMU 메인 동작 함수
// 1. UART 모드 설정
// 2. IMU 로부터 데이터 획득
// 3. IMU 데이터 지상 전송
void hyCANSAT_IMU_OPERATION()
{
	imu_recv_flag = 0 ;
	hyRxMode_Set(RxMode_IMU);
	hyTransUartUntil(&imu_recv_flag,0,WAIT_CNT_IMU );
	hyTransUartWait(WAIT_CNT_SEND);
}

//////////////////////////////////////////////////////////
// 카메라 메인 동작 함수
// 1. 카메라에 STOP 명령 전송
// 2. 카메라에 TAKE 명령 전송
// 3. 카메라에 영상 크기 명령 전송
// 4. 카메라로 부터 영상 수신 명령 전송
// 5. 카메라 영상 지상 전송
// 6. 오류 발생시 카메라 초기화

void hyCANSAT_CAMERA_OPERATION()
{
	if( IS_NEED_ICMD(ICMD_MASK_CAMERA_RESET) ) // When Need Camera Reset //
	{ 
		CLEAR_ICMD(ICMD_MASK_CAMERA_RESET); 
		hyRxMode_Set(RxMode_CAMERA) ; 
		hyCamera_PictureSize() ; hyTransUartWait(WAIT_CNT_CAMERA_STEP) ; 
		hyCamera_Reset_Set()   ; hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
	}
	else // only camera operation
	{
		img_recv_flag = 0 ;
//		hyRxMode_Set(RxMode_CAMERA)	 ; hyCamera_Stop()        ; hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
		hyRxMode_Set(RxMode_CAMERA)	 ; hyCamera_Take()        ; hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
		hyRxMode_Set(RxMode_CAMERA_SIZE) ; hyCamera_Size()        ; hyTransUartWait(WAIT_CNT_CAMERA_STEP) ; 
		hyRxMode_Set(RxMode_CAMERA_IMAGE); hyCamera_ImageGet(0,0,hyCameraImageSize[0],hyCameraImageSize[1]);  
		hyTransUartUntil(&img_recv_flag,0,WAIT_CNT_CAMERA_IMAGE );

		if( (img_recv_flag == 0) )	// when not receive image 
		{
			hyRxMode_Set(RxMode_CAMERA)   ; hyCamera_Init(); hyTransUartWait(WAIT_CNT_CAMERA_STEP);  
		}
		hyTransUartWait(WAIT_CNT_SEND);
//		hyRxMode_Set(RxMode_CAMERA)	 ; hyCamera_Stop(); hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
		hyRxMode_Set(RxMode_CAMERA)	 ; hyCamera_Resume(); hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
//		hyRxMode_Set(RxMode_CAMERA)	 ; hyCamera_Osd_Add_Char("cansat")        ; hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
	}
}

void hyCANSAT_CAMERA_NO_OPERATION()
{
	hyRxMode_Set(RxMode_CAMERA)		 ; hyCamera_Stop(); hyTransUartWait(WAIT_CNT_CAMERA_STEP) ;
}

void hyCANSAT_Gs_Tx_Message(unsigned char msg_id,const char *msg)
{
  
  unsigned char msg_len = 0 ;
  for( msg_len = 0 ; msg_len < 255 && msg[msg_len] ; msg_len++ ) ;
  
  Gs_Tx_QueueIn(0x76);
  Gs_Tx_QueueIn(0x00);
  Gs_Tx_QueueIn(0xA0+(msg_id&0x0f));
  Gs_Tx_QueueIn(msg_len);
  for( unsigned char idx=0 ; idx < msg_len ; idx++ )  
  {
    Gs_Tx_QueueIn(msg[idx]);
  }
  hyTransUartWait(WAIT_CNT_SEND);
}

// Send time_sec to GS //
void hyCANSAT_LIFE_SIGN_OPERATION()
{
        static long previousTime = 0 ; 
       long now =  millis() ;
       	unsigned char b ; unsigned char cs=0;
     
       if( 1000 < (now  - previousTime) || now < previousTime )  // time is different 
       {
          previousTime = now ;
               time_sec = time_sec + time_run - (now/1000) ; 
               time_run = now / 1000 ; 
//          if( ls_state )
          {
              	Gs_Tx_QueueIn(0x76);
              	Gs_Tx_QueueIn(0x00);
              	Gs_Tx_QueueIn(0x4C);
              	Gs_Tx_QueueIn(0x09);
              	b = ( time_sec >> 24 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_sec >> 16 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_sec >>  8 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_sec >>  0 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              #if 1	
              	b = ( time_run >> 24 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_run >> 16 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_run >>  8 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_run >>  0 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              #else
              	b = 0 ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = 0 ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_msec >>  8 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              	b = ( time_msec >>  0 ) & 0xFF ;	Gs_Tx_QueueIn(b); cs ^= b ;
              #endif	
              	b = cs                        ;	Gs_Tx_QueueIn(b);
              	hyTransUartWait(WAIT_CNT_SEND);
                hyLOG_LIFE_SIGN() ; // use time_sec, time_run // 
          }
       }
}	

// ATLM 메인 동작 함수
// 1. UART 수신 모드 설정
// 2. ATLM 8개 획득
// 3. ATLM 전송
void hyCANSAT_ATLM_OPERATION()
{
	hyRxMode_Set(RxMode_ATLM); hyTransUartWait(WAIT_CNT_ATLM);
//	for( unsigned char step = 0 ; step < 8 ; step++ )
	for( unsigned char step = 0 ; step < 8 ; step++ ) // 3 times run
	{
		hyATlm_Get();	
		hyTransUartWait(WAIT_CNT_ATLM);
	}
	hyATlm_Send();      hyTransUartWait(WAIT_CNT_SEND);
        hyLOG_ATLM_DATA();
}

// DTLM 메인 동작 함수
// 1. 필요안 DTLM 설정 동작 수행
// 2. UART 수신 모드 설정
// 3. DTLM 상태 지상 전송
// 4. 필요한 FLAG OFF 동작
void hyCANSAT_DTLM_OPERATION()
{
	hyDTlm_OP();
	hyRxMode_Set(RxMode_DTLM); hyTransUartWait(WAIT_CNT_ATLM);
	hyDTlm_Send();             hyTransUartWait(WAIT_CNT_SEND);

        hyLOG_DTLM_DATA(); 

	CLEAR_ICMD(ICMD_MASK_GS  );
//	CLEAR_LED (LED_MASK_GS   );

}

// 사용자 정의 OPERATION 수행 
void hyCANSAT_ATLM_TO_DIGITAL()
{
#if 1	
	unsigned char pf = PINF ; // ANALOG PIN PORT
	cansat_dcmd_1 = 0 ;
	if( pf & 0x01 ) cansat_dcmd_1 |= 0x80 ;
	if( pf & 0x02 ) cansat_dcmd_1 |= 0x40 ;
	if( pf & 0x04 ) cansat_dcmd_1 |= 0x20 ;
	if( pf & 0x08 ) cansat_dcmd_1 |= 0x10 ;
	if( pf & 0x10 ) cansat_dcmd_1 |= 0x08 ;
	if( pf & 0x20 ) cansat_dcmd_1 |= 0x04 ;
	if( pf & 0x40 ) cansat_dcmd_1 |= 0x02 ;
	if( pf & 0x80 ) cansat_dcmd_1 |= 0x01 ;
#else	
	cansat_dcmd_1 = PINF ;
#endif	
	PORTA = cansat_dcmd_1 ; // ANALOG to DIGITAL
}

void hyCANSAT_ATLM_TO_LED()
{
	cansat_led = 0 ;
	if( 0x10 < ad_data[0] ) cansat_led |= 0x01 ;
	if( 0x30 < ad_data[0] ) cansat_led |= 0x02 ;
	if( 0x50 < ad_data[0] ) cansat_led |= 0x04 ;
	if( 0x70 < ad_data[0] ) cansat_led |= 0x08 ;
	if( 0x90 < ad_data[0] ) cansat_led |= 0x10 ;
	if( 0xB0 < ad_data[0] ) cansat_led |= 0x20 ;
	if( 0xD0 < ad_data[0] ) cansat_led |= 0x40 ;
	if( 0xF0 < ad_data[0] ) cansat_led |= 0x80 ;
	if ( ! IS_NEED_OP(OP_MASK_USER  ) ) { cansat_led = cansat_dcmd_0 ; }
//	else                              { cansat_led = cansat_dcmd_0; }
//	cansat_led = cansat_dcmd_0       ;
//	cansat_led = cansat_led ^ 0xFF ;
	PORTB = cansat_led             ;	
}

// SERVO OPERATION
Servo csServo[2];
void hyCANSAT_SERVO_ATTACH()
{
  if(! csServo[0].attached() )   csServo[0].attach(D0);
  if(! csServo[1].attached() )   csServo[1].attach(D1);
}
void hyCANSAT_SERVO_DETACH()
{
  if( csServo[0].attached() )   csServo[0].detach();
  if( csServo[1].attached() )   csServo[1].detach();
}

void hyCANSAT_SERVO_OPERATION()
{
  hyCANSAT_SERVO_ATTACH();
  if( IS_NEED_ICMD(ICMD_MASK_PWM_0_L) ) {	pwm_0_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_0_R) ) ? PWM_CNT_CENTER : PWM_CNT_LEFT         ;} // 11 , 10
  else	                                {	pwm_0_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_0_R) ) ? PWM_CNT_RIGHT  : cansat_byte_cmd[0]   ;} // 01 , 00
  
  if( IS_NEED_ICMD(ICMD_MASK_PWM_1_L) ) {	pwm_1_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_1_R) ) ? PWM_CNT_CENTER : PWM_CNT_LEFT         ;} // 11 , 10
  else	                                {	pwm_1_cnt = ( IS_NEED_ICMD(ICMD_MASK_PWM_1_R) ) ? PWM_CNT_RIGHT  : cansat_byte_cmd[1]   ;} // 01 , 00
  
  csServo[0].write(pwm_0_cnt);
  csServo[1].write(pwm_1_cnt);
}

// 사용자 정의 OPERATION 수행
void hyCANSAT_USER_OPERATION()
{
  hyCANSAT_SERVO_DETACH();
  if      ( IS_NEED_ICMD(ICMD_MASK_ATLM_TO_DIGITAL) ) hyCANSAT_ATLM_TO_DIGITAL();
  else if ( IS_NEED_ICMD(ICMD_MASK_ATLM_TO_LED    ) ) hyCANSAT_ATLM_TO_LED();
}


// 캔위성에서 지상으로 문자 메시지 전달
void hyCansat_Message(char *msg)
{
	for( unsigned char i = 0 ; i < 255 && *msg ; i++ , msg++ )
		hyGsTxByte(*msg);
	hyTransUartWait(WAIT_CNT_SEND);
}

// 최초 초기화 함수
// 포트에 값을 쓸때는 PORTx 사용
// 포트에서 값을 읽을 때는 PINx 사용 
// 포트를 설정할때는 DDRx 사용 : 이때 0 은 Read , 1 은 Write
void hyCANSAT_AVR_INIT()
{
	// 초기화 부분
	MCUCR		= 0x00 ;

	DDRA  = 0xFF ; // for DTLM CMD : output
	PORTA = 0x00 ;

	DDRB  = 0xFF;  // for OUTPUT LED
	PORTB = 0xFF;

	DDRC  =0xFF; // for selection output
	DDRF  =0x00; // for adc input
	//PORTF =0x00;

	DDRE  =0xFF; // 1110 0010 : OUT for 1 Read for 0 

	PL_UART_INIT();
	GS_UART_INIT();

  // Timer 설정 부분 
	// Timer :: for 1ms 
  
#ifdef CANSAT_AT90CAN128  // when at90can128
  TIMSK0  = (1<<TOIE0) ;
  TCNT0 = 0 ;
  TCCR0A = 5 ; // 1 , 2=/8, 3 =/64 , 4 =/256 5 =/1024 
#else
  TIMSK = (1<<TOIE0) ;
  TCNT0 = 0 ;
  TCCR0 = 5 ; // 1 , 2=/8, 3 =/64 , 4 =/256 5 =/1024 
//  TCCR0 = 3 ; // 1 , 2=/8, 3 =/64 , 4 =/256 5 =/1024
#endif

	DDRD = 0xFF ; // 양방향

  //	Telemetry 를 얻기 위해 AMUX 초기화 부분
//	ADC_INIT(); 

	// for watch dog enable
//	wdt_enable(0x07); 
//	sei();
}

// 1. stop
// 2. mode
// 3. uart
// 4. reset 
void hyCANSAT_BT_INIT()
{
	// 9600 bps 설정
	GS_PORT_SPEED_SET(9600)     ;
	
	hyCansat_Message("+++\n" );
	hyCansat_Message("at+btmode,3\n" );
	hyCansat_Message("at+uartconfig,115200,N,1,0\n" );
	hyCansat_Message("atz\n" );
	
	GS_PORT_SPEED_SET(115200) ;
}

void hyCANSAT_PAYLOAD_INIT()
{
	watch_dog_cnt=0 ;

	hyCansat_Message("CANSAT FSW (hypark@satrec)[" __DATE__ "]\n" );
//	hyBluetooth_Init();
//	hyCamera_Init() ;	
}

void hyCANSAT_INIT(unsigned char ucCheck)
{
  hyCANSAT_AVR_INIT();
//	hyCANSAT_BT_INIT();
  if( ucCheck ) hyBluetooth_Init();
  hyCANSAT_PAYLOAD_INIT();
  SET_ICMD(ICMD_MASK_CAMERA_RESET);
}


#ifdef GS_0_PL_1

//void serialEvent() { Gs_Rx_QueueIn(Serial.read()); } //  OK
//void serialEvent1(){ Pl_Rx_QueueIn(Serial1.read()); } // NOT OK : why ??? 
void serialEvent() { SET_LED(LED_MASK_PL); while(Serial.available() )  Gs_Rx_QueueIn(Serial.read()); } // OK 
//void serialEvent1(){ SET_LED(LED_MASK_PORT); while(Serial1.available() ) Pl_Rx_QueueIn(Serial1.read()); }

#else

void serialEvent() { Pl_Rx_QueueIn(Serial.read()); }
//void serialEvent1(){ Gs_Rx_QueueIn(Serial1.read()); }

#endif

//unsigned long TIME_TO_YEAR(unsigned long itime) 
//{
//  unsigned long days ;
//  days = itime/(24*3600)  ; // days from 1970 
//  leap_days = days/360/4  ; // every 4 years 
//  years = days/365 
//  
//}

// use ctime

char sLogFileName[10] ;
void hyLOG_FILE_OPEN()
{
  unsigned long t          ; 
  unsigned long local_time = time_sec + (9*3600) ;  // KST : local time + 9 hour //
  unsigned char pos=0 ; 
  
  t = local_time/3600%24 ;   sLogFileName[pos++] = (t/10) + '0'  ; sLogFileName[pos++] = (t%10) + '0' ;// hour // 
  t = local_time/60%60   ;   sLogFileName[pos++] = (t/10) + '0'  ; sLogFileName[pos++] = (t%10) + '0' ;// min  // 
  sLogFileName[pos++] = '.' ; 
  sLogFileName[pos++] = 'C' ;
  sLogFileName[pos++] = 'S' ;
  sLogFileName[pos++] = 'V' ;
  sLogFileName[pos++] = 0  ;
    
  csLogFile = SD.open(sLogFileName,FILE_WRITE);
  if( csLogFile ) SET_LED( LED_MASK_LOG ) ; 
}

void hyLOG_FILE_CLOSE()
{
  if( csLogFile ) csLogFile.close() ; 
}

void hyLOG_GPS_DATA()
{
  if( csLogFile )
  {
//    csLogFile.println("GPS DATA");
    csLogFile.write(gps_data,gps_data_idx);
    csLogFile.println();
  }
}

void hyLOG_IMU_DATA()
{
  if( csLogFile )
  {
    csLogFile.print("IMU_DATA,");
    csLogFile.write(imu_data,imu_data_idx);
    csLogFile.println();
  }
}

void hyLOG_CAMERA_INIT()
{
  if( csLogFile )
  {
    csLogFile.println("CAMERA_INIT");
  }
}

void hyLOG_CAMERA_SIZE()
{
  if( csLogFile )
  {
    unsigned short us = hyCameraImageSize[0] ;
    us <<= 8 ; us += hyCameraImageSize[1] ;
    csLogFile.print("IMAGE_SIZE,");
    csLogFile.print(us);
    csLogFile.println();
  }
}

void hyLOG_CAMERA_IMAGE()
{
  if( csLogFile )
  {
    csLogFile.println("IMAGE_RECEIVED");
  }
}

void hyLOG_ATLM_DATA() 
{
  if( csLogFile )
  {
    csLogFile.print("ATLM,");
    csLogFile.print(ad_data[0]); csLogFile.print(',') ;
    csLogFile.print(ad_data[1]); csLogFile.print(',') ;
    csLogFile.print(ad_data[2]); csLogFile.print(',') ;
    csLogFile.print(ad_data[3]); csLogFile.print(',') ;
    csLogFile.print(ad_data[4]); csLogFile.print(',') ;
    csLogFile.print(ad_data[5]); csLogFile.print(',') ;
    csLogFile.print(ad_data[6]); csLogFile.print(',') ;
    csLogFile.print(ad_data[7]); 
    csLogFile.println();
  }
}

void hyLOG_DTLM_DATA()
{
  if( csLogFile )
  {
    csLogFile.print("DTLM4,");
    csLogFile.print( (cansat_dcmd_0 & 0x80) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x40) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x20) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x10) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x08) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x04) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x02) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_0 & 0x01) ? '1' : '0' ) ; csLogFile.print(',') ;
    
    csLogFile.print( (cansat_dcmd_1 & 0x80) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x40) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x20) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x10) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x08) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x04) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x02) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_1 & 0x01) ? '1' : '0' ) ; csLogFile.print(',') ;

    csLogFile.print( (cansat_dcmd_2 & 0x80) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x40) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x20) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x10) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x08) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x04) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x02) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_2 & 0x01) ? '1' : '0' ) ; csLogFile.print(',') ;
    
    csLogFile.print( (cansat_dcmd_3 & 0x80) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x40) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x20) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x10) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x08) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x04) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x02) ? '1' : '0' ) ; csLogFile.print(',') ;
    csLogFile.print( (cansat_dcmd_3 & 0x01) ? '1' : '0' ) ; csLogFile.print(',') ;
    
    csLogFile.println();
  }
}

void hyLOG_LIFE_SIGN()
{
  unsigned long local_time = time_sec + ( 9*3600 ) ; // KST : 9 hour 
  hyLOG_FILE_CLOSE();  
  hyLOG_FILE_OPEN();
  if( csLogFile )
  {
    // yy,mm,dd.hh.mm,ss //
    csLogFile.print("LIFESIGN,");
    csLogFile.print(local_time/3600%24); csLogFile.print(':'); // hh
    csLogFile.print(local_time/60%60)  ; csLogFile.print(':'); // mm
    csLogFile.print(local_time%60)     ; csLogFile.print(','); // ss
    csLogFile.print(time_run/3600%24); csLogFile.print(':'); // hh
    csLogFile.print(time_run/60%60)  ; csLogFile.print(':'); // mm
    csLogFile.print(time_run%60)     ;
    csLogFile.println();
  }
}

void setup() 
{
  hyCANSAT_INIT(BT_NEED_INIT);
  hyLed_PortSet() ;
  OP_FROM_EEPROM();
  // SD Card Init Setting //
  pinMode(SS,OUTPUT) ;    // SS pin
  SD.begin(SS);
  csLogFile = SD.open("test.txt",FILE_WRITE);
// if( csLogFile ) csLogFile.println("testing 1.2.3");
 if( csLogFile ) csLogFile.close();
}



void loop() 
{
	hyLed_PortSet() ;

	if( IS_NEED_ICMD(ICMD_MASK_RESET) ) { CLEAR_ICMD(ICMD_MASK_RESET) ; hyCANSAT_INIT(1); }
		
	if( watch_dog_cnt == 0 ) { sei(); } 
	//		else wdt_reset() ;                                            // reset hardware watch_dog when data received //
	watch_dog_cnt=0 ;

	hyCANSAT_LIFE_SIGN_OPERATION() ;
	if( IS_NEED_OP(OP_MASK_GPS   ) ) hyCANSAT_GPS_OPERATION()   ; // 9600
  if( IS_NEED_OP(OP_MASK_GPS   ) ) hyCANSAT_Gs_Tx_Message(0,"After GPS OP");
	if( IS_NEED_OP(OP_MASK_IMU   ) ) hyCANSAT_IMU_OPERATION()   ; // 115200
  if( IS_NEED_OP(OP_MASK_IMU   ) ) hyCANSAT_Gs_Tx_Message(1,"After IMU OP");
	if( IS_NEED_OP(OP_MASK_CAMERA) ) hyCANSAT_CAMERA_OPERATION(); // Digital Picture 115200
  if( IS_NEED_OP(OP_MASK_CAMERA) ) hyCANSAT_Gs_Tx_Message(2,"After CAMERA OP");
	if( IS_NEED_OP(OP_MASK_ATLM  ) ) hyCANSAT_ATLM_OPERATION()  ; 
	if( IS_NEED_OP(OP_MASK_DTLM  ) ) hyCANSAT_DTLM_OPERATION()  ;
	if( IS_NEED_OP(OP_MASK_USER  ) ) hyCANSAT_USER_OPERATION()  ; // USER DEFINED OPERATION
  else                             hyCANSAT_SERVO_OPERATION() ; // PWM OPERATION 
	hyTransUartOne();  
}
