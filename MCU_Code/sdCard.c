#include "sdCard.h"
#include <time.h>


FRESULT fr;

//Mount sd card
void mountSD(){
    // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    FATFS fs;
    fr = f_mount(&fs, "0:", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

void writeSD(struct tm date, struct sensorData sensorData){
    // TODO: Setup fcn to read from buffer data and have unique filename argument
    //Open a file and write to it
    char CSVName[20] = "Test_CSV.csv";
    FIL fil;
    

    // time_t t= time(NULL);
    // struct tm date = *localtime(&t);
    //sprintf(CSVName, "%d-%02d-%02d_%02d:%02d.csv", date.tm_year + 1900, date.tm_mon+1, date.tm_mday, date.tm_hour, date.tm_min);
    
    const char* const filename = CSVName;
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }
    //Write inital header data below
    if (f_printf(&fil, "Timestamp, PM1 (V), PM1(I), PM1(W), PM2 (V), PM2(I), PM2(W), PM3 (V), PM3(I), PM3(W), Temp (C), Light (W/m^2)") < 0) {
        printf("f_printf failed\n");
    }

    //Write row data
    if (f_printf(&fil, "\n%02d-%02d_%02d:%02d:%02d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
    date.tm_year + 1900, date.tm_mon+1, date.tm_mday, date.tm_hour, date.tm_min, date.tm_sec,
     sensorData.PM1voltage, sensorData.PM1current, sensorData.PM1power, 
    sensorData.PM2voltage, sensorData.PM2current, sensorData.PM2power, sensorData.PM3voltage, 
    sensorData.PM3current, sensorData.PM3power, sensorData.temperature, sensorData.irradiance) < 0) {
        printf("f_printf failed\n");
    }

    // Close the file
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

 
/**Copied from test sd card file
 * // Open a file and write to it
    FIL fil;
    const char* const filename = "test.csv";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }
    if (f_printf(&fil, "Hello, world - 2!\nNew row?, Test\n") < 0) {
        printf("f_printf failed\n");
    }

    // Close the file
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
 */

//Unmount sd card
//f_unmount("");
