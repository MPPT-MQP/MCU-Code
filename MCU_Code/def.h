/*
 *  ======== OLED SCREEN ========
 *  OLED Screen functions included if this line is uncommented
 */
//#define OLED_SCREEN


/*
 *  ======== Algorithm Toggles ========
 *  0=CV, 1=B, 2=PNO, 3=PNOV, 4=INC, 5=INCV, 6=PSO, 7=TMP, 8=AofA, 9=DTY, 10=RCC (not fully implemented)
*/
#define ALGO_TOGGLE CV
#define CV 0
#define B 1
#define PNO 2
#define PNOV 3
#define INC 4
#define INCV 5
#define PSO 6
#define TMP 7
#define AofA 8
#define DTY 9
//#define RCC 10

// Set a constant duty cycle instead of sweeping duty
// #define CONSTANT_DUTY 0.7

// Serial Monitor Data Formated for Live Plotting
// #define LIVE_PLOT

// Frequency Divider of DC-DC Converter
#define DCDCFreq 3125

extern char algorithms[11][5];
extern char selectedAlgo[5];