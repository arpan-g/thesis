
#ifndef CONF_ENV_DISTRIBUTE_SEH_WSN_V2
#define CONF_ENV_DISTRIBUTE_SEH_WSN_V2

// debug
//#define LED_INDICATOR
// #define TO_TERMINAL
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
#define n12dBm  0xC6    // 11.1 mA
#define n10dBm  0x97
#define n8dBm   0x6E
#define n6dBm   0x7F    // 15.1 mA
#define n4dBm   0xA9
#define n2dBm   0xBB    // 21.2 mA

// define a set of sensor nodes
// TODO: auto select sensor nodes based on RF address
// char* Flash_Addr = (char *)0x10F0;              // RF Address = 0x10F0
// char addr_byte_0 = Flash_Addr[0];
// #if Flash_Addr[0] == 0xFF && Flash_Addr[1] == 0xFF && Flash_Addr[2] == 0xFF && Flash_Addr[3] == 0xFF
// #if addr_byte_0 == 0xFF
// #define SN143   1
// #endif

 #define SN143   1
// #define SN939   2
// #define SN092   3
// #define SN251   4
// #define SN377   5
// #define SN542   6
// #define SN826   7
// #define SN852   8

#define TPL         n2dBm  // Tx power level: -30 dBm
#define PCH         CH13    // public channel to send dimming level dn
#define GCH         CH11    // broadast channel

#ifdef FDMA
// node1
#ifdef SN143
    #define MCH         CH1     // master channel
    #define SCH_NR      2
    #define SCH1        CH2     // slave (neighbor) channels
    #define SCH2        CH3
    #define SYNC_DELAY  0
    #define Zo          106
    #define Zu          56
    #define ENABLE_PIR  1
// node2
#elif SN939
    #define MCH         CH2     // master channel
    #define SCH_NR      2
    #define SCH1        CH1     // slave (neighbor) channels
    #define SCH2        CH4
    #define SYNC_DELAY  0
    #define Zo          71
    #define Zu          38
    #define ENABLE_PIR  1
// node3
#elif SN092
    #define MCH         CH3     // master channel
    #define SCH_NR      3
    #define SCH1        CH1     // slave (neighbor) channels
    #define SCH2        CH4
    #define SCH3        CH5
    #define SYNC_DELAY  0
    #define Zo          94
    #define Zu          51
    #define ENABLE_PIR  0
// node4
#elif SN251
    #define MCH         CH4     // master channel
    #define SCH_NR      3
    #define SCH1        CH2     // slave (neighbor) channels
    #define SCH2        CH3
    #define SCH3        CH6
    #define SYNC_DELAY  18000   // 3msec
    #define Zo          51
    #define Zu          27
    #define ENABLE_PIR  0
// node5
#elif SN377
    #define MCH         CH5     // master channel
    #define SCH_NR      3
    #define SCH1        CH3     // slave (neighbor) channels
    #define SCH2        CH6
    #define SCH3        CH7
    #define SYNC_DELAY  0
    #define Zo          83
    #define Zu          44
    #define ENABLE_PIR  0
// node6
#elif SN542
    #define MCH         CH6     // master channel
    #define SCH_NR      3
    #define SCH1        CH4     // slave (neighbor) channels
    #define SCH2        CH5
    #define SCH3        CH8
    #define SYNC_DELAY  0
    #define Zo          79
    #define Zu          42
    #define ENABLE_PIR  0
// node7
#elif SN826
    #define MCH         CH7     // master channel
    #define SCH_NR      2
    #define SCH1        CH5     // slave (neighbor) channels
    #define SCH2        CH8
    #define SYNC_DELAY  0
    #define Zo          113
    #define Zu          62
    #define ENABLE_PIR  0
// node8
#elif SN852
    #define MCH         CH8     // master channel
    #define SCH_NR      2
    #define SCH1        CH6     // slave (neighbor) channels
    #define SCH2        CH7
    #define SYNC_DELAY  4e4
    #define Zo          115
    #define Zu          62
    #define ENABLE_PIR  1
#endif
#endif

#define FRAME_LENGTH    50     // Nr of micro-frame to transmit
#define FRAME_ACK       51
// #define MSG             100     // currently, payload data is one byte with  value of 100
// #define MSG_ACK         101
#define MAX_TRY         FRAME_LENGTH
#define MAX_TRY_PID     20
#define ERROR_THRESHOLD 2
#define PAYLOAD_SIZE    4
#define ERR_POS         9
#define DIM_POS         10
#define CHANNEL_POS     11

#define S_IDLE          0
#define S_RX            1
#define S_RX_TX         2

#define TS1SEC          1
#define TS60SEC         60

#define RUN_VOLTS_3V3   676     // minimum voltage to execute at 3.3V (3.3*512/2.5)
#define RUN_VOLTS_3V2   667     // minimum voltage to execute at 3.2V (3.2*512/2.5)
#define RUN_VOLTS_3V1   635     // minimum voltage to execute at 3.1V (3.1*512/2.5)
#define RUN_VOLTS_3V0   614     // minimum voltage to execute at 3.0V (3.0*512/2.5)
#define RUN_VOLTS_2V8   573     // minimum voltage to execute at 2.8V (2.8*512/2.5)
#define RUN_VOLTS_2V5   512     // minimum voltage to execute at 2.5V (2.5*512/2.5)

#define HI              1
#define LO              0
#define PIR_TIMEOUT     342
#define SLOW_3SEC       31800
#define SLOW_1SEC       10600   // for back-off-N
#define SLOW_1MSEC      11      // for adaptive duty-cyle
#define SLOW_5MSEC      53
#define SLOW_10MSEC     106
#define SLOW_20MSEC     230     // for uFrame Tx
#define SLOW_50MSEC     530
#define SLOW_90MSEC     954
#define SLOW_100MSEC    1060    // for LPL
#define SLOW_500MSEC    5300
#define SLOW_50MSEC     530     // for uFrame Tx
#define FAST_1MSEC      1e3
#define FAST_2MSEC      2e3
#define FAST_3MSEC      3e3
#define FAST_4MSEC      4e3
#define FAST_5MSEC      5e3
#define FAST_8MSEC      8e3
#define FAST_10MSEC     1e4    // 41781
#define FAST_20MSEC     2e4
#define FAST_30MSEC     3e4
#define FAST_40MSEC     4e4
#define FAST_50MSEC     5e4
#define FAST_60MSEC     6e4
#define LPD_30SEC       44000
#define LPD_20SEC       30000
#define LPD_5SEC        6250
#define LPD_CYCLE_TIME  2000// 1563 ms
#define LPD_1SEC        1250
#define LPD_500MSEC     625
#define LPD_400MSEC     500
#define LPD_100MSEC     125
#define LPD_90MSEC      138
#define LPD_2MSEC       3
#define LPD_45MSEC      69
#define LPD_50MSEC      76
#define LPD_55MSEC      83
#define LPD_60MSEC      90
#define LPD_63MSEC      95

// ACLK offset
#ifdef SN939
    // node2
    #define LPD_65MSEC  98
#elif defined SN826
    // node7
    #define LPD_65MSEC  80
#else
    // nodes others
    #define LPD_65MSEC  89
#endif

#define LPD_70MSEC      105
#define LPD             LPD_1SEC
#define ZO_TIMEOUT      2e6// 2e6     // TX_RX
#define ZU_TIMEOUT      1e6// 1e6     // RX_ONLY, thus is half of zo_timeout
#define AID_TIMEOUT     5e5


typedef struct TableEntry {
    int8_t en;
    uint8_t dn;
    uint8_t ch;
} TableEntry;

/*
#if (SCH_NR == 2)
typedef struct Table {
    TableEntry item1;
    TableEntry item2;
} Table;
#elif (SCH_NR == 3)
typedef struct Table {
    TableEntry item1;
    TableEntry item2;
    TableEntry item3;
} Table;
#endif
*/

#if (SCH_NR == 2)
typedef struct Table {
    TableEntry neighbor[2];
} Table;

uint8_t sch[2] = {SCH1, SCH2};

#elif (SCH_NR == 3)
typedef struct Table {
    TableEntry neighbor[3];
} Table;


uint8_t sch[3] = {SCH1, SCH2, SCH3};

#endif

#endif