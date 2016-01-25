//////////////////////////////////////////////////////////////////////////////////////////
//
// Date: 01-09-2014
// Contact: shuai_li@philips.com
// Access Point (AP)
//
///////////////////////////////////////////////////////////////////////////////////////////

#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "conf_env.h"
// #include "calibration_function.h"


// mrfiPacket_t packet;
mrfiPacket_t packet_uframe;
mrfiPacket_t packet_ack_uframe;
mrfiPacket_t packet_dn;
// mrfiPacket_t packet_dn_buffer[32];
uint16_t uFrame_ack = 0;
uint16_t msg_ack = 0;
uint16_t semLinkTo = 0;
volatile uint8_t motion_detected = 0;
uint16_t tic_sec = 0;
uint16_t tic = 0;
uint8_t prev_state = S_IDLE;
uint16_t rt_volt = 0;
uint16_t xn = 0;
int16_t en = 0;
int16_t dn = 0;
uint8_t pid_iter = 0;
uint8_t counter = MAX_TRY;
uint8_t enable_low_power_delay = 0;
uint8_t matlab_connected = 0;
uint8_t send_packet = 0;
volatile uint32_t timer;

int PaPIR_Isr(); /* this called from mrfi_board.c */
void TxData(uint32_t time);
void TxString(char* string, int length);
void transmit_start();

void pause_low_power_delay()
{
    TBCTL |= MC_0;
}

void continue_low_power_delay()
{
    TBCCTL0 = CCIE;
    TBCTL |= TBSSEL_1 + MC_1 + ID_3; 
}

void low_power_delay(uint16_t tic)
{
    enable_low_power_delay = 1;
    uint16_t TimerTemp = 0;
    TimerTemp = TBCCR0;                       // Save current content of TBCCR0
    TBCCR0 = tic;                             // Set new TBCCR0 delay
    
    TBCTL |= TBCLR;                           // Clear TBR counter
    TBCCTL0 &= ~CCIFG;                        // Clear CCIFG Flag
    TBCCTL0 = CCIE;                           // TBCCR0 interrupt enabled
    TBCTL |= TBSSEL_1 + MC_1 + ID_3;          // Start Timer B, |= OR =???
    __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3
    TBCTL &= ~(MC_1);                         // Stop Timer B
    TBCCR0 = TimerTemp;
}


void delay_msec(uint8_t miliseconds)
{
    for (tic_sec = 0; tic_sec < miliseconds; tic_sec++) {
        __delay_cycles(8000);
    }
}

void delay_sec(uint8_t seconds)
{
    for (tic_sec = 0; tic_sec < seconds; tic_sec++) {
        for (tic = 0; tic < 1000; tic++) {
            __delay_cycles(8000);
        }
    }
}

#ifdef TO_TERMINAL
void print_counter(int8_t counter)
{
  char output[] = {"    "};
  output[0] = '0' + ((counter/100)%10);
  output[1] = '0'+((counter/10)%10);
  output[2] = '0'+ (counter%10);
  TXString(output, (sizeof output)-1);
}
#endif
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
    
    // Initialize TimerA & TimerB
    BCSCTL3 |= LFXT1S_2;
    // TACTL=MC_0; TACCTL0=0; TACCR0=1060;  //slow timeout, ~100msec
    // TBCTL=MC_0; TBCCTL0=0; TBCCR0=41781; //fast timeout, ~10msec
/*    
    MRFI_WakeUp();
    MRFI_RxOn();
    mrfiSpiWriteReg(CHANNR,MCH);
*/
 BCSCTL3 |= LFXT1S_2;
  TACCR0 = 11;	//Number of cycles in the timer
  TACTL = TASSEL_1 + MC_1;
  //delay_msec(3000);
  CCTL0 |= CCIE; 
  _EINT();
 uint8_t count =0;
    while (1) {
 
		 if(count < 1)
    {
      transmit_start();
      delay_msec(19);
      count++;
    }  
    delay_msec(1);
        delay_msec(1);
    }
}


void transmit_start()
{
  
  mrfiPacket_t transmissionStartPacket;
  transmissionStartPacket.frame[0] = 8+4;
  transmissionStartPacket.frame[9] = 0xFA;
  transmissionStartPacket.frame[10] = 0xFA;
  transmissionStartPacket.frame[11] = 0xFA;
  transmissionStartPacket.frame[12] = 0xFA;
  MRFI_WakeUp();
  mrfiSpiWriteReg(CHANNR, MCH);
  MRFI_RxOn();
  
  MRFI_Transmit(&transmissionStartPacket, MRFI_TX_TYPE_FORCED);
  
  
  
  
}

void transmit_sync_packet()
{
  mrfiPacket_t sync_packet;
  sync_packet.frame[0] = 8+5;
  sync_packet.frame[9] = 0xFB;
  sync_packet.frame[10] =  timer      & 0xff;  
  sync_packet.frame[11] = (timer>>8)  & 0xff;
  sync_packet.frame[12] = (timer>>16) & 0xff;
  sync_packet.frame[13] = (timer>>24) & 0xff;
  MRFI_WakeUp();
  mrfiSpiWriteReg(CHANNR, MCH);
  MRFI_RxOn();
  
  MRFI_Transmit(&sync_packet, MRFI_TX_TYPE_FORCED);
}


int PaPIR_Isr()
{
    motion_detected = 1;
    return 0;
}


void MRFI_RxCompleteISR()
{
    // notice that this is Rx ISR, no need to call MRFI_RxOn
    // P1OUT |= 0x02;
    uint32_t time;
    MRFI_Receive(&packet_dn);
    time = timer;

   

      
    // __delay_cycles(1000);
    // P1OUT &= ~0x02;
#ifdef LED_INDICATOR
    // P1OUT ^= 0x02;
#endif
#ifdef TO_TERMINAL
    /*print_counter(packet_dn.frame[11]); // MCH index
    print_counter(packet_dn.frame[10]); // lux reading HB
    print_counter(packet_dn.frame[9]);  // lux reading LB
    */
    ///*
    
  

    
    if (matlab_connected) {
        P1OUT |= 0x02;
        TxData(time);
        P1OUT &= ~0x02;
        // send_packet = 1;
    }
    
  
    //*/
    // TxData();
    // TxString("C1D20", 5);
#endif
}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    char rx = UCA0RXBUF;
    if ( rx == '#')
    {
        matlab_connected = 1;
#ifdef LED_INDICATOR
        P1OUT |= 0x01;
#endif
        // switch on radio
        MRFI_WakeUp();
        MRFI_RxOn();
        mrfiSpiWriteReg(CHANNR,MCH);
    } else
    if (rx == '$')
    {
        matlab_connected = 0;
    }
}


// void TxData(unsigned int data)
void TxData(uint32_t time)
{
    // 'C' & '0'+packet_dn.frame[11] & 'D' && packet_dn.frame[10] & packet_dn.frame[9] & 'X' & packet_dn.frame[13] & packet_dn.frame[12]
    // 'I' & packet_dn.frame[9 to 12] & 'R' & RSSI & 'L' & packet_dn.frame[13] & packet_dn.frame[12]
    // unsigned int data = 0;
    // char dataString[] = {"CxDxxXxx"};
    
    
    // parse MAC ID
  char dataString[] = {"IxxxxRxLxxxxxxx\r\n"};
  dataString[1] = packet_dn.frame[9];
  dataString[2] = packet_dn.frame[10];
  dataString[3] = packet_dn.frame[11];
  dataString[4] = packet_dn.frame[12];
  
  // RSSI, 1 byte
  // dataString[6] = Mrfi_CalculateRssi(packet_dn.rxMetrics[0]);
  // ref: http://e2e.ti.com/support/wireless_connectivity/f/156/t/74859.aspx
  //dataString[6] = packet_dn.rxMetrics[0];
 
  // xn, 2 bytes
  dataString[5] = packet_dn.frame[13]; //169
  dataString[6]  = packet_dn.frame[14]; // data packet
  dataString[7]  = packet_dn.frame[15];// data packet
  dataString[8]  = packet_dn.frame[16];// data packet
  dataString[9]  = packet_dn.frame[17];// data packet
  dataString[10]  = packet_dn.frame[18];
  dataString[11]  = packet_dn.frame[19];
  dataString[12]  = packet_dn.frame[20];
  dataString[13]  = packet_dn.frame[21];
  dataString[14] = time & 0xff;  
  dataString[15] = (time>>8) & 0xff ;  
  dataString[16] = (time>>16) & 0xff;
  dataString[17] = (time>>24) & 0xff;
    
    
/*	
    data = packet_dn.frame[11];
    switch (data) {
        case CH1 :
            dataString[1] = '1';
            break;
        case CH2 :
            dataString[1] = '2';
            break;
        case CH3 :
            dataString[1] = '3';
            break;
        case CH4 :
            dataString[1] = '4';
            break;
        case CH5 :
            dataString[1] = '5';
            break;
        case CH6 :
            dataString[1] = '6';
            break;
        case CH7 :
            dataString[1] = '7';
            break;
        case CH8 :
            dataString[1] = '8';
            break;
        case CH9 :
            dataString[1] = '9';
            break;
        case CH10 :
            dataString[1] = 'a';
            break;
        case CH11 :
            dataString[1] = 'b';
            break;
        case CH12 :
            dataString[1] = 'c';
            break;
        case CH13 :
            dataString[1] = 'd';
            break;
        case CH14 :
            dataString[1] = 'e';
            break;
        case CH15 :
            dataString[1] = 'f';
            break;
        default :
            // dataString[1] = 'x';
            break;
    }
    
    // start of dimming level
    // dataString[2] = 'D';
    dataString[3] = packet_dn.frame[10];     // HB
    dataString[4] = packet_dn.frame[9];      // LB
    
    // start of light sensor reading
    dataString[6] = packet_dn.frame[13];     // HB
    dataString[7] = packet_dn.frame[12];     // LB
*/
    TxString(dataString, sizeof dataString);
    // TxString(dataString, 9);
}


void TxString(char* string, int length)
{
  int pointer;
  for( pointer = 0; pointer < length; pointer++)
  {
    //volatile int i;
    UCA0TXBUF = string[pointer];
    while (!(IFG2&UCA0TXIFG));              // USCI_A0 TX buffer ready?
  }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void interrupt_slow_timeout (void)
{
  
  //P1OUT ^= 0x02;
  timer++;
  
  if(timer % 150 == 0 ){
    transmit_sync_packet();
	  
  }
  
  
  
}

/*
#pragma vector=TIMERB0_VECTOR
__interrupt void low_power_timer (void)
{
    enable_low_power_delay = 0;
    __bic_SR_register_on_exit(LPM3_bits);     // Clear LPM3 bit from 0(SR)
}*/
