
#ifndef CONF_ENV
#define CONF_ENV

// debug
#define LED_INDICATOR
#define TO_TERMINAL
#define FDMA

// define a set of available channels
// case 11: mrfiSpiWriteReg(CHANNR,0x00); break;
// case 12: mrfiSpiWriteReg(CHANNR,0x0D); break;
// case 13: mrfiSpiWriteReg(CHANNR,0x19); break;
// case 14: mrfiSpiWriteReg(CHANNR,0x26); break;
// case 15: mrfiSpiWriteReg(CHANNR,0x32); break;
// case 16: mrfiSpiWriteReg(CHANNR,0x3F); break;
// case 17: mrfiSpiWriteReg(CHANNR,0x4B); break;
// case 18: mrfiSpiWriteReg(CHANNR,0x58); break;
// case 19: mrfiSpiWriteReg(CHANNR,0x64); break;
// case 20: mrfiSpiWriteReg(CHANNR,0x71); break;
// case 21: mrfiSpiWriteReg(CHANNR,0x7D); break;
// case 22: mrfiSpiWriteReg(CHANNR,0x8A); break;
// case 23: mrfiSpiWriteReg(CHANNR,0x96); break;
// case 24: mrfiSpiWriteReg(CHANNR,0xA3); break;
// case 25: mrfiSpiWriteReg(CHANNR,0xAF); break;
// case 26: mrfiSpiWriteReg(CHANNR,0xBC); break;
#define CH1     0x00
#define CH2     0x0D
#define CH3     0x19
#define CH4     0x26
#define CH5     0x32
#define CH6     0x3F
#define CH7     0x4B
#define CH8     0x58
#define CH9     0x64
#define CH10    0x71
#define CH11    0x7D
#define CH12    0x8A
#define CH13    0x96
#define CH14    0xA3
#define CH15    0xAF
#define CH16    0xBC

// define a set of radio transmission power
// mrfiSpiWriteReg(PATABLE,0x50);// -30dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x84);// -24dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x46);// -20dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x55);// -16dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x8D);// -14dBm Tx power
// mrfiSpiWriteReg(PATABLE,0xC6);// -12dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x97);// -10dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x6E);// -8 dBm Tx power
// mrfiSpiWriteReg(PATABLE,0x7F);// -6 dBm Tx power
// mrfiSpiWriteReg(PATABLE,0xA9);// -4 dBm Tx power
// mrfiSpiWriteReg(PATABLE,0xBB);// -2 dBm Tx power
#define n30dBm  0x50
#define n24dBm  0x84
#define n20dBm  0x46
#define n16dBm  0x55
#define n14dBm  0x8D
#define n12dBm  0xC6
#define n10dBm  0x97
#define n8dBm   0x6E
#define n6dBm   0x7F
#define n4dBm   0xA9
#define n2dBm   0xBB

// define a set of sensor nodes
// #define SN939   1
// #define SN251   2
// #define SN143   3
// #define SN603   4
#define AP      16

#ifdef FDMA
#ifdef SN939
    #define MCH         CH1     // master channel
    #define SCH1        CH3     // slave (neighbor) channels
    #define SCH2        CH2
    #define TPL         n12dBm  // Tx power level: -12 dBm
    #define PCH         CH16    // public channel to send dimming level dn
#elif SN251
    #define MCH         CH3     // master channel
    #define SCH1        CH1     // slave (neighbor) channels
    #define SCH2        CH4
    #define TPL         n12dBm  // Tx power level: -12 dBm
    #define PCH         CH16    // public channel to send dimming level dn
#elif SN603
    #define MCH         CH2     // master channel
    #define SCH1        CH1     // slave (neighbor) channels
    #define SCH2        CH4
    #define TPL         n12dBm  // Tx power level: -12 dBm
    #define PCH         CH16    // public channel to send dimming level dn
#elif SN143
    #define MCH         CH4     // master channel
    #define SCH1        CH2     // slave (neighbor) channels
    #define SCH2        CH3
    #define TPL         n12dBm  // Tx power level: -12 dBm
    #define PCH         CH16    // public channel to send dimming level dn
#elif AP
    #define MCH         CH13//CH11    // master channel
    // #define MCH         CH5     // master channel
    #define TPL         n2dBm   // Tx power level: -2 dBm
#endif
#endif

#define FRAME_LENGTH    50      // Nr of micro-frame to transmit
#define FRAME_ACK       51
#define MSG             100     // currently, payload data is one byte with  value of 100
#define MSG_ACK         101
#define MAX_TRY         50
#define MAX_TRY_PID     10
#define ERROR_THRESHOLD 5

#define S_IDLE          0
#define S_TX_UFRAME     1
#define S_TX_UFRAME_ACK 2
#define S_TX_MSG        3

#define RUN_VOLTS       614     // minimum voltage to execute at 3.0V (3*512/2.5)
// #define RUN_VOLTS       0     // minimum voltage to execute at 3.0V (3*512/2.5)
#define HI              1
#define LO              0


#endif