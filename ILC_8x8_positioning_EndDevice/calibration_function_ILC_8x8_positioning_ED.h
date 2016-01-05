
#ifndef CALIBRATION_FUNCTION_DISTRIBUTE_SEH_WSN_V2
#define CALIBRATION_FUNCTION_DISTRIBUTE_SEH_WSN_V2

#define ABS(a)	        (((a) < 0) ? -(a) : (a))
#define MAX(a, b)       ((a) > (b) ? (a) : (b))
#define MIN(a, b)       ((a) < (b) ? (a) : (b))


// #define Lo		500
// #define Lu		300
// #define Zo		92 // (Lo - f2_const) / f2_slope * f1_slope + f1_const
// #define Zu		61 // (Lu - f2_const) / f2_slope * f1_slope + f1_const
// #define Zo              75//112//45
// #define Zu              64//18


// #define FC(a)   (((a)>>1) + 22) // 0.56*v + 22.4
// #define FC(a)   (1.4 * (a) + 24) // 1.436*v + 23.7
// #define F1(a)   (87 * (a) + 15) // 56*d + 16
#define FC(a)           ((a) + (a>>1))//((a)*1.5)//((a) + (a)>>1)  // 1.5*v
#define F1_SLOPE        70
#define F1_CONST        -7
#define F1(a)           (70 * (a) - 7)  // 33.73*d
// #define F1_INV(a)       ((a) + 7)       // (Zs + en + 7) / 70
#define F1_INV(a)       (a)
#define F2(a)   (553 * (a) + 7) // 552.5*d + 7.04
// #define F3(a)   ((a) < 0 ? )

#endif