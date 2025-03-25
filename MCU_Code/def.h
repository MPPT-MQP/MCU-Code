/*
 *  ======== OLED SCREEN ========
 *  OLED Screen functions included if this line is uncommented
 */
// #define OLED_SCREEN


/*
 *  ======== Algorithm Toggles ========
 *  0=CV, 1=B, 2=PNO, 3=PNOV, 4=INC, 5=INCV, 6=RCC, 7=PSO, 8=TMP, 9=AofA
*/
#define ALGO_TOGGLE AofA
#define CV 0
#define B 1
#define PNO 2
#define PNOV 3
#define INC 4
#define INCV 5
//#define RCC 6
#define PSO 7
#define TMP 8
#define AofA 9
#define DTY 10

// Set a constant duty cycle instead of sweeping duty
// #define CONSTANT_DUTY 0.7

// Serial Monitor Data Formated for Live Plotting
// #define LIVE_PLOT

// Frequency Divider of DC-DC Converter
#define DCDCFreq 3125

extern char algorithms[11][5];
extern char selectedAlgo[5];