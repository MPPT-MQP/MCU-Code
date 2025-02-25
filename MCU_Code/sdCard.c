#include "sdCard.h"
#include <time.h>
#include "pico/aon_timer.h"

//Sensor Data Buffer
struct sensorData sensorBuffer[QUEUE_BUFFER_SIZE];
uint16_t BufferCounter = 0;

FATFS fs;
FIL fil;
FRESULT fr;

//SD Card Core 1 Globals
char sensorLocalBuffer[SAMPLES_TO_SAVE+10][110];

uint16_t localSensorCounter = 0;

//FileName
char CSVName[40];

//Mount sd card
void mountSD(){
    // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    fr = f_mount(&fs, "0:", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

void initSDFile(){
    struct tm timeFile;
    //Open a file and write to it
    
    
    aon_timer_get_time_calendar(&timeFile);
    snprintf(CSVName, 40, "%02d-%02d-%02d %02d.%02d.%02d.csv", timeFile.tm_year, timeFile.tm_mon, timeFile.tm_mday, timeFile.tm_hour, timeFile.tm_min, timeFile.tm_sec);
    
    const char* filename = CSVName;
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }
    //Write inital header data below
    if (f_printf(&fil, "Timestamp, PM1 (V), PM1(I), PM1(W), PM2 (V), PM2(I), PM2(W), PM3 (V), PM3(I), PM3(W), Temp (C), Light (W/m^2), Duty") < 0) {
        printf("f_printf failed\n");
    }

    // Close the file
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

/// @brief Copy sensor data from shared queue to local buffer on core 1
void copySDBuffer(){
    
    char test[SAMPLE_SIZE];
    //Returns false if the queue is empty
    bool removeQueue = queue_try_remove(&shareQueue, &test);
    strcpy(sensorLocalBuffer[localSensorCounter], test);
    if(removeQueue == false){
        //Queue empty
        //printf("\nCORE 1: QUEUE EMPTY\n");
    }else{
        if(localSensorCounter++ > SAMPLES_TO_SAVE){
            localSensorCounter = 0;
            saveFlag = true;
        }
    }
    
}

void writeSD(uint32_t bytes){
    uint bytesWritten = 0;    

    const char* filename = CSVName;
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }

    //Write buffer of data to SD card
    if(f_write(&fil, sensorLocalBuffer, bytes, &bytesWritten) != 0){
        //Some error was returned and the write did not successfully complete
        panic("\n\n\n Error writing to sd card occurred \n\n\n");
    }else{
        printf("\n\n Bytes written: %d, Desired: %d", bytesWritten, bytes);
    }

    saveFlag = false;

    // Close the file
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}




// void writeSD(struct tm date, struct sensorData sensorData){
//     // TODO: Setup fcn to read from buffer data and have unique filename argument
//     //Open a file and write to it
//     char CSVName[20] = "Test_CSV.csv";

//     // time_t t= time(NULL);
//     // struct tm date = *localtime(&t);
//     //sprintf(CSVName, "%d-%02d-%02d_%02d:%02d.csv", date.tm_year + 1900, date.tm_mon+1, date.tm_mday, date.tm_hour, date.tm_min);
    
//     const char* const filename = CSVName;
//     fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
//     if (FR_OK != fr && FR_EXIST != fr) {
//         panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
//     }
//     //Write inital header data below
//     if (f_printf(&fil, "Timestamp, PM1 (V), PM1(I), PM1(W), PM2 (V), PM2(I), PM2(W), PM3 (V), PM3(I), PM3(W), Temp (C), Light (W/m^2)") < 0) {
//         printf("f_printf failed\n");
//     }

//     //Write row data
//     if (f_printf(&fil, "\n%02d-%02d_%02d:%02d:%02d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
//     date.tm_year + 1900, date.tm_mon+1, date.tm_mday, date.tm_hour, date.tm_min, date.tm_sec,
//      sensorData.PM1voltage, sensorData.PM1current, sensorData.PM1power, 
//     sensorData.PM2voltage, sensorData.PM2current, sensorData.PM2power, sensorData.PM3voltage, 
//     sensorData.PM3current, sensorData.PM3power, sensorData.temperature, sensorData.irradiance) < 0) {
//         printf("f_printf failed\n");
//     }

//     // Close the file
//     fr = f_close(&fil);
//     if (FR_OK != fr) {
//         printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
//     }
// }

 
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
