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
uint32_t timer=0;
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
uint8_t bit_count = 0;
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
static unsigned int my_data[SAMPLE_NUM];
volatile uint8_t currentTs = TS1SEC;
uint8_t mac_addr[4] = {169,175,1,43};

uint16_t tic_sec = 0;



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
uint8_t tx_to_ap();
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
  
  /* debug
  // P4DIR |= BIT3;
  // P4OUT &= ~BIT3;
  // P3DIR |= BIT4;
  // P3OUT |= BIT4;
  MRFI_Sleep();
  __bis_SR_register(GIE+LPM3_bits);
  */
  P4DIR |= BIT3;
  P4OUT &= ~BIT3;
  P2DIR |= BIT3;
  P2OUT &= ~BIT3;
  P2DIR |= BIT4;
  P2OUT &= ~BIT4;
  
#ifdef FDMA
  // Frequency agility
  mrfiSpiWriteReg(PATABLE,TPL);// Tx power
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
  char *Flash_Addr;
  Flash_Addr = (char *)0x1169;              // MAC Address is 4 bytes starting from 0x10F0
  packet_uframe.frame[9] = mac_addr[0];
  packet_uframe.frame[10] = mac_addr[1];
  packet_uframe.frame[11] = mac_addr[2];
  packet_uframe.frame[12] = mac_addr[3];
  
  // Initialize TimerA & TimerB
  // set LFXT1 to the VLO @ 12kHz
  // :( VLO drifts with temperature and supply voltage
  BCSCTL3 |= LFXT1S_2;
  //TACTL = MC_0;
 // TACCTL0 = 0;
  //TACCR0 = SLOW_1SEC;
  // TACCR0 = SLOW_1SEC;                      // 1060;  //slow timeout, ~100msec
  TBCTL = MC_0;
  TBCCTL0 = 0;
  TBCCR0 = FAST_3MSEC;//10MSEC;               // 41781; //(16-bit, <65535);41781;//41781; //fast timeout, ~10msec
  
  // Initialize light sensor (APDS9004)
  P4OUT &= ~BIT5;
  P4REN |= BIT5;                              // enable IN14 pull-down resistor
  P4DIR &= ~BIT5;                             // P4.5 is sensor output
  P4OUT &= ~(BIT4+BIT6);                      // P4.4,6 = VDD
  P4DIR |= (BIT4+BIT6);                       // switch off APDS9004
  
  /* Initialize interface to EVAL-CBC-5300
  P3DIR |= 0xD0;                            // port 3 set after initilization
  P3OUT &= ~0x20;                           // \charge
  P3REN |= 0x20;                            // \charge pulldown
  */
  P3DIR |= BIT4;
  P3OUT &= ~BIT4;                           // batoff, disable Enerchip is useful in very low ambient energy conditions to steer all of the avaiable energy into the load
  P3DIR &= ~BIT5;                           // nCharge
  //P3OUT &= ~BIT5;                           // enable pull-down resistor
  //P3REN |= BIT5;
  
  // Enable PIR interrupt
  /*P2DIR |= BIT2;
  P2OUT |= BIT2;
  P2DIR &= ~BIT1;
  P2OUT &= ~0x03;                           // P2.1,0 pulldown
  P2REN |= 0x03;                            // P2.1,0 pulldown
  P2IES &= ~0x02;                           // P2.1 lo/hi edge
  low_power_delay(LPD_30SEC);
  P2IFG &= ~0x02;                           // P2.1 IFG cleared
  P2IE |= 0x02;                             // P2.1 interrupt enabled*/
  
  // source vcc to pir sensor
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
  while (1) {
    //get_lux_reading_repeat();
    if(bit_count > 31){
      bit_count = 0;
      packet_data = 0;
      
    }
    read_pir();
    
    if( timer_ack == 1){
      
      create_packet(bit_count); 
      bit_count++;
      if(bit_count > 31 ){
        tx_to_ap();
      }
      low_power_delay(LPD_100MSEC);
    }
  }
}

void delay_msec(uint8_t miliseconds)
{
    for (tic_sec = 0; tic_sec < miliseconds; tic_sec++) {
        __delay_cycles(8000);
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
  TACCR0 = SLOW_100MSEC;	//Number of cycles in the timer
  TACTL = TASSEL_1 + MC_1; //TASSEL_1 use aclk as source of timer MC_1 use up mode timer
//  continue_low_power_delay();
  
  
}

uint8_t tx_to_ap()
{
  // TODO: insert PIR sensor output (motion_detected) into packet
  
//  P1OUT &= ~0x02; 
  packet_uframe.frame[13] = packet_no++;
  packet_uframe.frame[14] =  packet_data & 0xff;  
  packet_uframe.frame[15] = packet_data>>8 & 0xff ;  
  packet_uframe.frame[16] = packet_data>>16 & 0xff;
  packet_uframe.frame[17] = packet_data>>24 & 0xff;
  packet_uframe.frame[18] = timer & 0xff;  
  packet_uframe.frame[19] = timer>>8 & 0xff ;  
  packet_uframe.frame[20] = timer>>16 & 0xff;
  packet_uframe.frame[21] = timer>>24 & 0xff;
  
  
  
  
  // hop to public channel and send dn to AP (pseudo distributed protocol)
//  do {
    P1OUT ^=0x02;
    
    cca_tx_success = nac_mrfi_tx_cca(&packet_uframe, PCH/*GCH*/);
    // keep RxOn and wait for ack
    __delay_cycles(1e6);
    
    
//  } while (cca_tx_success != 1 || ack_flag != 1);
  
  
  cca_tx_success = 0;
  ack_flag = 0;
  
  return 1;
}


void get_lux_reading_repeat()
{
  // turn on APDS9004
  P4OUT |= (BIT4+BIT6);
  
  // delay to allow setup
  __delay_cycles(1e6);
  
  // initialize ADC
  ADC10CTL1 = INCH_14 + CONSEQ_2;
  ADC10CTL0 = SREF_0 + ADC10SHT_3 + MSC + ADC10ON + ADC10IE;
  __delay_cycles(350);
  ADC10DTC1 = SAMPLE_NUM;
  ADC10SA = (unsigned short)my_data;
  ADC10CTL0 &= ~ENC;
  while (ADC10CTL1 & BUSY); // Wait if ADC10 core is active
  
  // start sampling
  ADC10CTL0 |= ENC + ADC10SC; // Sampling and conversion start
  __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled  
  ADC10CTL0 &= ~ENC;
  
  // data filtering
  xn = 0;
  for (int i = 0; i < SAMPLE_NUM; i++) {
    xn = xn + my_data[i];
  }
  xn = xn / SAMPLE_NUM;
  
  // switch off APDS9004
  P4OUT &= ~(BIT4+BIT6);
}


void get_lux_reading()
{
  // set P4.4,6
  P4OUT |= (BIT4+BIT6);
  //    P4OUT |= BIT5;
  //    P4REN &= ~BIT5;
  //    // P4OUT &= ~BIT5;
  //    // P4REN |= BIT5;                         // turn off P4.5 pulldown resistor
  //    P4DIR &= ~BIT5;
  
  // P4OUT |= 0x50;                          // FIXME: port delay affects fuction fc
  // P4OUT |= BIT3;
  // low_power_delay(1);
  __delay_cycles(1e5);
  // P4OUT &= ~BIT3;
  ADC10CTL1 = INCH_14 + ADC10DIV_4;       // Temp Sensor ADC10CLK/5;
  //lis: ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE + ADC10SR + REF2_5V;
  ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE + ADC10SR;
  __delay_cycles(350);                    // delay to allow reference to settle
  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
  __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
  // rt_volt = ADC10MEM;
  rt_volt = ADC10MEM;
  ADC10CTL0 &= ~ENC;
  ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power
  
  // shut down light sensor to save power
  // P4OUT &= ~0x50;
  // P4REN &= ~BIT5;                          // Set P4.5 pulldown resistor
  // P4DIR |= BIT5;
  //    P4DIR |= BIT5;
  //    P4OUT &= ~BIT5;      // forward bias
  
  P4OUT &= ~(BIT4+BIT6);
  
  // xn = FC(rt_volt); 
  xn = rt_vcc - rt_volt;
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


void get_rt_voltage()
{
  ADC10CTL1 = INCH_11;                    // AVcc/2
  // ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE + REF2_5V;
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;
  __delay_cycles(250);                    // delay to allow reference to settle
  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
  __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
  rt_volt = ADC10MEM;
  ADC10CTL0 &= ~ENC;
  ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power
  
  rt_vcc = rt_volt * 2;
  
  // dynamic adjust ADC reference voltage between REF2_5V and REF1_5V
  // in order to improve xn accuracy
  if (rt_volt < 1023) {
    // Vcc is below 3.0V, switch to REF1_5V
    batt_below_3V = 1;
  } else if (rt_volt == 1023) {
    // otherwise, still safe to use 2.5V reference
    batt_below_3V = 0;
  }
}


uint8_t check_rt_voltage(uint16_t run_volts_setup)
{
  ADC10CTL1 = INCH_11;                    // AVcc/2
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE + REF2_5V;
  __delay_cycles(250);                    // delay to allow reference to settle
  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
  __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
  rt_volt = ADC10MEM;
  ADC10CTL0 &= ~ENC;
  ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power
  
  if (rt_volt < run_volts_setup) {
    return 0;
  } else {
    return 1;
  }
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


uint8_t is_neighbor(mrfiPacket_t *pPacket)
{
  uint8_t channel_index = pPacket->frame[CHANNEL_POS];
  uint8_t neighbor_found = 0;
  for (uint8_t i = 0; i < SCH_NR; i++) {
    if (channel_index == sch[i]) {
      neighbor_found = 1;
    }
  }
  // __delay_cycles(SYNC_DELAY);
  return neighbor_found;
}


uint8_t need_aid(mrfiPacket_t *pPacket)
{
  int8_t lux_error = pPacket->frame[ERR_POS];
  if (lux_error > 0) {
    return 1;
  } else {
    return 0;
  }
}


void bound_dimming_level()
{
  // bound dimming level
  if (ds > F1_SLOPE) {
    ds = F1_SLOPE;
  } else
    if (ds < 0) {
      ds = 0;
    }
}


void neighbor_aid_control(mrfiPacket_t *pPacket)
{
  int8_t neighbor_lux_error = pPacket->frame[ERR_POS];
  //TODO: Zs = F3(lux_error);
  //FIXME: avoid abrrupt dimming
  // ds = ds + MIN(neighbor_lux_error, 7);
  // Zs = Zs + MIN(neighbor_lux_error, 7);
  // ds = ds + neighbor_lux_error;
  if (ds < F1_SLOPE) {
    Zs = Zs + (neighbor_lux_error);
  }
  // take care of overflow
  // bound_dimming_level();
}


void ack_uframe()
{
  Ft.frame[DIM_POS] = (ds&0xFF);
  MRFI_WakeUp();
  mrfiSpiWriteReg(CHANNR, current_channel);
  // better not using cca, cause it is time critical
  MRFI_Transmit(&Ft, MRFI_TX_TYPE_FORCED);
  P2OUT &= ~BIT4;
  MRFI_Sleep();
}


void update_local_table(mrfiPacket_t *pPacket)
{
  uint8_t channel_index = pPacket->frame[CHANNEL_POS];
  for (uint8_t i = 0; i < SCH_NR; i++) {
    if (local_table.neighbor[i].ch == channel_index) {
      local_table.neighbor[i].dn = pPacket->frame[DIM_POS];
      break;
    }
  }
}


void sort_local_table()
{
  int16_t min_dn = local_table.neighbor[0].dn;
  int16_t dn = 0;
  current_channel = local_table.neighbor[0].ch;
  for (uint8_t i = 1; i < SCH_NR; i++) {
    dn = local_table.neighbor[i].dn;
    if (min_dn > dn) {
      min_dn = dn;
      current_channel = local_table.neighbor[i].ch;
    }
  }
}


void MRFI_RxCompleteISR()
{
  
#ifdef LED_INDICATOR
 // P1OUT ^= BIT1;
#endif
  P4OUT |= BIT3;
  // Ft received
  stop_fast_timeout();
  //stop_slow_timeout();
  
  mrfiPacket_t packet_ack;
  MRFI_Receive(&packet_ack);
  MRFI_Sleep();
  
  // check if ACK is for me
  /*char *Flash_Addr;
  Flash_Addr = (char *)0x1169;*/
  if(packet_ack.frame[9] == 0xFA & timer_ack == 0 ){
      start_timer();
      timer_ack = 1;
      
  }
  if (packet_ack.frame[9] == mac_addr[0] && packet_ack.frame[10] == mac_addr[1] && packet_ack.frame[11] == mac_addr[2] && packet_ack.frame[12] == mac_addr[3]) {
    ack_flag = 1;
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


#pragma vector=TIMERA0_VECTOR
__interrupt void interrupt_slow_timeout (void)
{
  if(timer%2){
  P1OUT &= ~0x02;
  }
  else{
  P1OUT |= 0x02;
 
  }
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
