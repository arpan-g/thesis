//////////////////////////////////////////////////////////////////////////////////////////
//
// Date: 01-09-2014
// Contact: shuai_li@philips.com
// 
///////////////////////////////////////////////////////////////////////////////////////////

#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "conf_env_ILC_8x8_positioning_ED.h"
#include "calibration_function_ILC_8x8_positioning_ED.h"


#define SAMPLE_NUM 16
#define DELTA 2930 
#define START_PACKET 0xFA
#define SYNC_PACKET 0xFB


// mrfiPacket_t packet;
mrfiPacket_t Ft;
mrfiPacket_t Fr;
mrfiPacket_t packet_dn;
mrfiPacket_t packet_uframe;
volatile uint8_t motion_detected = 0;
uint16_t tic = 0;
volatile uint8_t prev_state = S_IDLE;
uint16_t rt_vcc = 0;
uint16_t rt_volt = 0;
int16_t xn = 0;
int16_t en = 0;
volatile uint32_t timer=0;
volatile uint8_t timer_ack = 0;
uint8_t packet_no = 0;
// int16_t dn = 0;
uint8_t pid_iter = 0;
uint8_t counter = MAX_TRY;
uint16_t compare,old_capture,delta;
volatile uint8_t enable_low_power_delay = 0;
uint16_t pir_timeout = 0;
Table local_table;
uint8_t which_neighbor = 0;
uint8_t nCharge = 0;
// uint8_t ack_uframe_flag = 0;
volatile uint8_t cca_tx_success = 0;
volatile uint8_t ack_flag = 0;
uint32_t packet_data=0x0;
uint8_t radio_mode = S_RX;
volatile uint8_t current_channel = GCH;
volatile int16_t Zs = 0;
volatile int16_t ds = 7;        // 21/70 = 0.3
unsigned long occupancy_timeout = 0;
unsigned long aid_timeout = 0;
uint8_t fast_increase_cnt = 0;
uint8_t fast_decrease_cnt = 0;
volatile uint8_t batt_below_3V = 0;
uint16_t count = 0;
volatile uint8_t currentTs = TS1SEC;
uint8_t mac_addr[4] = {169,175,4,43};
uint8_t bit_count = 0;
uint16_t tic_sec = 0;
uint8_t isSynced = 0;


// functions declared

void get_rt_voltage();
uint8_t check_rt_voltage(uint16_t run_volts_setup);
void Get_Lux_Reading(mrfiPacket_t * pPacket);
int PaPIR_Isr(); /* this called from mrfi_board.c */
// void PID_controller();
void start_slow_timeout();
void stop_slow_timeout();
void start_fast_timeout();
void stop_fast_timeout();
void pause_low_power_delay();
void continue_low_power_delay();
uint8_t tx_to_ap(uint32_t start_time,uint32_t end_time);
void create_packet(uint8_t bit_count);
void increase_preamble_duty_cycle();
void decrease_preamble_duty_cycle();
void bound_dimming_level();
void get_lux_reading();
void close_loop_control();
uint8_t nac_mrfi_tx_cca(mrfiPacket_t * pPacket, const uint8_t carrier_sel);
void get_lux_reading_repeat();
void read_pir();
void start_timer();
void delay_msec(uint8_t miliseconds);
void stop_timer();

void transmit_sync_packet();

void low_power_delay(uint16_t tic)
{
  // PIR interrupt not allowed during LPD
  // otherwise TBCCR0 will be smaller than FAST_TIMEOUT
  // P2IE &= ~0x02;                            // P2.1 interrupt disabled
  // P2IFG &= ~0x02;                           // P2.1 IFG cleared
  enable_low_power_delay = 1;
  uint16_t TimerTemp = 0;
  TimerTemp = TBCCR0;                       // Save current content of TBCCR0
  TBCCR0 = tic;                             // Set new TBCCR0 delay
  
  TBCTL |= TBCLR;                           // Clear TBR counter
  TBCCTL0 &= ~CCIFG;                        // Clear CCIFG Flag
  TBCCTL0 = CCIE;                           // TBCCR0 interrupt enabled
  TBCTL |= TBSSEL_1 + MC_1 + ID_3;          // Start Timer B, cannot use SMCLK in LPM3, only ACLK remains alive
  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3
  TBCTL &= ~(MC_1);                         // Stop Timer B
  TBCCR0 = TimerTemp;
  TACCTL0 |= CCIE;
  // P2IFG &= ~0x02;                           // FIXED: P2.1 IFG cleared
  // P2IE |= 0x02;                             // P2.1 interrupt enabled
  occupancy_timeout = occupancy_timeout + tic;
  aid_timeout = aid_timeout + tic;
}


void Button_Init()
{
  P1DIR |=  0x03;
  P1DIR &= ~0x04;
  P1REN |= 0x04;
  P1IE |= 0x04;
}


int main(void)
{
  uint32_t time;
  uint32_t start_time;
  // Initialize board
  BSP_Init();
   
#ifdef TO_TERMINAL
  // Initialize UART
  P3SEL    |= 0x30;     // P3.4,5 = USCI_A0 TXD/RXD
  UCA0CTL1  = UCSSEL_2; // SMCLK
  UCA0BR0   = 0x41;     // 9600 from 8Mhz
  UCA0BR1   = 0x3;
  UCA0MCTL  = UCBRS_2;                     
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
  IE2      |= UCA0RXIE; // Enable USCI_A0 RX interrupt
#endif
  // Initialize radio interface
  MRFI_Init();
  
  // initialize button
  Button_Init();
  
  
  P4DIR |= BIT3;
  P4OUT &= ~BIT3;
  P2DIR |= BIT3;
  P2OUT &= ~BIT3;
  P2DIR |= BIT4;
  P2OUT &= ~BIT4;

#ifdef FDMA
  // Frequency agility
  mrfiSpiWriteReg(PATABLE,0xFF);// Tx power
  mrfiSpiWriteReg(MDMCFG1,0x23);
  mrfiSpiWriteReg(MDMCFG0,0xF8);// 400kHz channel spacing
  mrfiSpiWriteReg(FREQ2,0x5C);
  mrfiSpiWriteReg(FREQ1,0x80);
  mrfiSpiWriteReg(FREQ0,0x00);  // 2.405GHz base frequency
  mrfiSpiWriteReg(CHANNR,MCH);  // stay at master channel
#endif
  MRFI_Sleep();                 // turn off radio asap to meet power budget
  
  // packet format
  // |-------------------------------------|
  // | MAC ID (4xbytes) | Xn@WSP (2xbytes) |
  // |-------------------------------------|
  packet_uframe.frame[0] = 8 + 13;
  packet_uframe.frame[9] = mac_addr[2];
//  packet_uframe.frame[10] = mac_addr[1];
 //packet_uframe.frame[11] = mac_addr[2];
//  packet_uframe.frame[12] = mac_addr[3];
  
  // Initialize TimerA & TimerB
  // set LFXT1 to the VLO @ 12kHz
  // :( VLO drifts with temperature and supply voltage
  BCSCTL3 |= LFXT1S_2;
  // 1060;  //slow timeout, ~100msec
  TBCTL = MC_0;
  TBCCTL0 = 0;
  TBCCR0 = FAST_3MSEC;//10MSEC;
  // 41781; //(16-bit, <65535);41781;//41781; //fast timeout, ~10msec
  start_timer();
  // Initialize light sensor (APDS9004)
  P4OUT &= ~BIT5;
  P4REN |= BIT5;                              // enable IN14 pull-down resistor
  P4DIR &= ~BIT5;                             // P4.5 is sensor output
  P4OUT &= ~(BIT4+BIT6);                      // P4.4,6 = VDD
  P4DIR |= (BIT4+BIT6);                       // switch off APDS9004
  
  
  P3DIR |= BIT4;
  P3OUT &= ~BIT4;                           // batoff, disable Enerchip is useful in very low ambient energy conditions to steer all of the avaiable energy into the load
  P3DIR &= ~BIT5;                           // nCharge
  
  P2DIR |= BIT2;        // turn on pir sensor
  P2OUT |= BIT2;
  
  // wait 30 sec to setup
  low_power_delay(LPD_30SEC);
  
  // configure P2.1 as input port before read
  P2REN &= ~BIT1;
  P2DIR &= ~BIT1;
  MRFI_WakeUp();
  mrfiSpiWriteReg(CHANNR, PCH);
  MRFI_RxOn();
  
#ifdef LED_INDICATOR
  P1OUT |= 0x01;
#endif
  P1OUT |= 0x01;
  // Enter low-power listen
  // start_slow_timeout();
 
  
  start_time = timer;
  while (1) {
    //get_lux_reading_repeat();
    if(bit_count > 31){
      bit_count = 0;
      packet_data = 0;
      start_time = timer;
    }
    read_pir();
    create_packet(bit_count);
    time = timer;
    bit_count++;
    

      
      
      if(bit_count > 31 ){
         
          tx_to_ap(start_time,time);

      }else{
      
      low_power_delay(LPD_100MSEC);
      }
    
  }
}



void create_packet(uint8_t bit_count){
  
  packet_data |=  motion_detected << bit_count;
  
}
void read_pir(){
  
  if (P2IN & BIT1){
    motion_detected = 1; 
    // P1OUT &= 0xFE; //turn off LED red
    return;
  }
  // P1OUT |=0x01;  //turn on LED red
  motion_detected = 0;
  
  
}

void start_timer()
{
  //  pause_low_power_delay();
  P1OUT |= 0x01;
  //Enable Interrupts on Timer
  TACCR0 = 11;	//Number of cycles in the timer
  TACTL = TASSEL_1 + MC_1; //TASSEL_1 use aclk as source of timer MC_1 use up mode timer
  //  continue_low_power_delay();
  
  
}



void sync_clock(mrfiPacket_t packet_ack){
  

  uint32_t t;

 t = ((packet_ack.frame[10] )|  (((uint32_t) packet_ack.frame[11])<< 8)    |
         (((uint32_t) packet_ack.frame[12])<< 16) | (((uint32_t) packet_ack.frame[13])<< 24));
  
  //t4 = timer;
  //delta = (uint16_t)0.5*((t2-t1)-(t4-t3));
  TACTL = MC_0;
  timer = t ;
  TACTL = TASSEL_1 + MC_1;
  
  
}



uint8_t tx_to_ap(uint32_t start_time,uint32_t end_time)
{
 
  
  
  packet_uframe.frame[10] = start_time & 0xff;  
  packet_uframe.frame[11] = (start_time>>8) & 0xff ;  
  packet_uframe.frame[12] = (start_time>>16) & 0xff;
  packet_uframe.frame[13] = packet_no++;
  packet_uframe.frame[14] = packet_data & 0xff;  
  packet_uframe.frame[15] = (packet_data>>8) & 0xff ;  
  packet_uframe.frame[16] = (packet_data>>16) & 0xff;
  packet_uframe.frame[17] = (packet_data>>24) & 0xff;
  packet_uframe.frame[18] = end_time & 0xff;  
  packet_uframe.frame[19] = (end_time>>8) & 0xff ;  
  packet_uframe.frame[20] = (end_time>>16) & 0xff;
  packet_uframe.frame[21] = (end_time>>24) & 0xff;
  
  
  
  
  // hop to public channel and send dn to AP (pseudo distributed protocol)
  //  do {
  
  
  cca_tx_success = nac_mrfi_tx_cca(&packet_uframe, PCH/*GCH*/);
  // keep RxOn and wait for ack
  //__delay_cycles(1e6);
  
  
  //  } while (cca_tx_success != 1 || ack_flag != 1);
  
  
  cca_tx_success = 0;
  ack_flag = 0;
  
  return 1;
}

void transmit_sync_packet(){
  mrfiPacket_t sync_packet;
  //  P1OUT &= ~0x02; 
  sync_packet.frame[0] = 8+8;
  sync_packet.frame[9] = SYNC_PACKET;
  sync_packet.frame[14] = mac_addr[2];
  sync_packet.frame[15] = 0;
  sync_packet.frame[10] = timer & 0xff;  
  sync_packet.frame[11] = (timer>>8) & 0xff ;  
  sync_packet.frame[12] = (timer>>16) & 0xff;
  sync_packet.frame[13] = (timer>>24) & 0xff;
  //MRFI_Transmit(&sync_packet, MRFI_TX_TYPE_FORCED);
  cca_tx_success = nac_mrfi_tx_cca(&sync_packet, PCH/*GCH*/);
  
}


uint8_t nac_mrfi_tx_cca(mrfiPacket_t * pPacket, const uint8_t carrier_sel)
{
  uint8_t mrfi_tx_success = 0;
  uint16_t taccr0_tmp = 0;
  uint16_t tactl_tmp = 0;
  
  MRFI_WakeUp();
  mrfiSpiWriteReg(CHANNR, carrier_sel);
  MRFI_RxOn();
  
  // stop/reset TimerA for Mrfi_DelayUsec();
  TACCTL0 = 0;
  taccr0_tmp = TACCR0;
  tactl_tmp = TACTL;
  TACTL = TASSEL_2 + MC_0;
  
  // TODO: setup back-off-N
  if (MRFI_Transmit(pPacket, MRFI_TX_TYPE_CCA) == MRFI_TX_RESULT_SUCCESS) {
    mrfi_tx_success = 1;
  }
  // MRFI_Sleep();
  
  // recover TimerA for correct LPL
  TACTL = tactl_tmp;
  TACCR0 = taccr0_tmp;
  TACCTL0 |= CCIE;
  return mrfi_tx_success;
}


void MRFI_RxCompleteISR()
{
  
#ifdef LED_INDICATOR
  // P1OUT ^= BIT1;
#endif
  P4OUT |= BIT3;
  // Ft received
  //stop_fast_timeout();
  //stop_slow_timeout();
  
  mrfiPacket_t packet_ack;
  MRFI_Receive(&packet_ack);
  MRFI_Sleep();
   if(packet_ack.frame[9] == SYNC_PACKET){
    sync_clock(packet_ack);
  }
  
}



int PaPIR_Isr()
{
  if (P2IFG & 0x02){
    motion_detected = 1;  
    return 0;
  }
  motion_detected = 0;
  return 0;
}




void start_slow_timeout()
{
  // TACCR0 = SLOW_100MSEC - SLOW_1MSEC*(FRAME_LENGTH-counter);
  TACTL |= TACLR;
  TACCTL0 = CCIE;
  TACTL = TASSEL_1 + MC_1;
}


void stop_slow_timeout()
{
  TACTL = MC_0;
  // TACTL = TASSEL_2 + MC_0;
  TACCTL0 = 0; // do not disable compare/capture interrupt for CCA
}


void start_fast_timeout()
{
  TBCCR0 = FAST_5MSEC;
  TBCTL |= TBCLR;
  TBCCTL0 = CCIE;
  TBCTL = TBSSEL_2 + MC_1 + ID_3;
}


void stop_fast_timeout()
{
  TBCTL = MC_0;
  TBCCTL0 = 0;
}


void pause_low_power_delay()
{
  TBCTL |= MC_0;
}


void continue_low_power_delay()
{
  TBCCTL0 = CCIE;
  TBCTL |= TBSSEL_1 + MC_1 + ID_3; 
}

#pragma vector=TIMERA0_VECTOR
__interrupt void interrupt_slow_timeout (void)
{
  
  timer++;
  
}


#pragma vector=TIMERB0_VECTOR
__interrupt void interrupt_fast_timeout (void)
{ 
  if (enable_low_power_delay == 1) {
    enable_low_power_delay = 0;
    // Clear LPM3 bit from 0(SR)
    __bic_SR_register_on_exit(LPM3_bits);
  } else {
    /*
    // turn off radio for low-power listening
    MRFI_Sleep();
    stop_fast_timeout();
    P4OUT &= ~BIT3;
#ifdef LED_INDICATOR
    P1OUT &= ~BIT0;
#endif        
    */
    // enter LPM3 on ISR exit
    __bis_SR_register_on_exit(LPM3_bits);
  }
}


#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
}




#pragma vector=PORT1_VECTOR
__interrupt void Port_1 (void)
{
  P1IFG &= ~0x04;
  //    P1OUT ^= BIT1;
  
  if (currentTs == TS60SEC) {
    currentTs = TS1SEC;
  } else
    if (currentTs == TS1SEC) {
      currentTs = TS60SEC;
    }
}