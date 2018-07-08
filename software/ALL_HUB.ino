/**
 * ---------------------------
 *     >>> SmartHub <<<
 * Author: Vladimir Zhelezarov
 * 
 *      Software for HUB
 * ---------------------------
 *  
 * - Initial build (18.06.18)
 * - Add CMDs in the protocoll (20-21.06.18)
 * - Smoothing (01.07.18)
 * - EEPROM log (08.07.18)
*/
#define VERSION 180708

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <EEPROM.h>
#include "RTClib.h"
#include "mcp4728.h"

/* communication cycle rate */
#define HEARTBEAT 100
#define SMOOTHING 6
#define MAX_SILENCE 1000
static bool linkOK = false;

/* keep track of hardware errors */
static bool HWError = false;

/* https://www.arduino.cc/en/Tutorial/Smoothing */
static int readIndex = 0;

static short readings_POS_A[SMOOTHING] = {0};
static int total_POS_A = 0;

static short readings_POS_B[SMOOTHING] = {0};
static int total_POS_B = 0;

static short readings_POS_C[SMOOTHING] = {0};
static int total_POS_C = 0;
/* -- */

static mcp4728 dac = mcp4728(0x60);
static RTC_DS3231 rtc;
static char daysOfTheWeek[7][12] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};

/* inputs - digital */
#define P107_A     2
#define P107_B     3
#define P107_C     4
#define VCU_A      5
#define VCU_B      6
#define VCU_C      7

/* outputs - digital */
#define VCU_OUT    8
#define FB         9

/* inputs - analog */
#define POS_A_IN   A0
#define POS_B_IN   A1
#define POS_C_IN   A2

/* for talking with A3 */
static bool  VCU_OK  =  false;
static bool  P107_OK =  false;
static short POS_A   =  400;
static short POS_B   =  400;
static short POS_C   =  400;
static bool  VCU_VIN =  false;
static short POS_REF =  400;

#define START_SEQ 0x40 /* @ */
#define END_SEQ 0x5E /* ^ */

/* blade position tolerance */
#define MIN_POS 200
#define MAX_POS 2200

/* all watched signals */
enum{ sP107_A, sP107_B, sP107_C, sVCU_A, sVCU_B, sVCU_C,
      sPOS_A, sPOS_B, sPOS_C, sALL };

/* current errors */
static bool ERRORS[sALL] = {0};

/* length of the protocolls */
#define CMDLEN 3
/* START + DATA + END */
#define HUBtoA3 16
/* START + CMD + DATA + END */
#define A3toHUB (7 + CMDLEN)

/* buffer for communication */
static byte buff[A3toHUB] = {0};
static int inCount = 0;

/* VCU, POS_REF start at pos .. */
#define POS_BUFF_VCU (1 + CMDLEN)
#define POS_BUFF_PREF (2 + CMDLEN)

/* watch for stable link */
static unsigned long lastTime = 0;

/* used for start/stop silence (see cmd) */
static bool silence = false;

/* X Macros */
#define X_DIGIN \
    X(P107_A,   F("P107_A")) \
    X(P107_B,   F("P107_B")) \
    X(P107_C,   F("P107_C")) \
    X(VCU_A,    F("VCU_A")) \
    X(VCU_B,    F("VCU_B")) \
    X(VCU_C,    F("VCU_C"))

#define X_POS \
    X(POS_A_IN, POS_A, F("POS_A")) \
    X(POS_B_IN, POS_B, F("POS_B")) \
    X(POS_C_IN, POS_C, F("POS_C"))

#define X_RTC_L \
    X(year, now.year(), '/') \
    X(month, now.month(), '/') \
    X(day, now.day(), F(" ("))
#define X_RTC_R \
    X(hour, now.hour(), ':') \
    X(minute, now.minute(), ':') \
    X(second, now.second(), ' ')

#define X_CMD \
    X(help,     '?', '?', '?', F("help")) \
    X(silence,  's', 'l', 'c', F("silence (!)")) \
    X(moveOn,   'm', 'o', 'v', F("end silence")) \
    X(showAll,  'a', 'l', 'l', F("show all")) \
    X(showTime, 't', 'i', 'm', F("show time")) \
    X(printSD,  'p', 'r', 'i', F("show SD")) \
    X(delSD,    'd', 'e', 'l', F("del SD")) \
    X(clearERR, 'c', 'l', 'r', F("clear errors"))

void quickBlink(int cycles, int timeA, int timeB)
{
    if (HWError) {
        return;
    }
    for (int i = 0; i < cycles; i++) {
        digitalWrite(FB, HIGH);
        delay(timeA);
        digitalWrite(FB, LOW);
        delay(timeB);
    }
}

bool checkEEPROM()
{
    int e = EEPROM.read(0);
    if ((e != 0) && (e != 1)) {
        e = 0;
    }
    return (bool) e;
}

/** 
 * This uses the FB Led to show hardware errors 
 * 
 * TODO: it can be further improved by addind two more leds
 * to the board for a total of 3 possible error messages
 * (Errors: 01, 10, 11; OK: 00)
 */
void showError()
{
    HWError = true;
    digitalWrite(FB, HIGH);
    if (!checkEEPROM()) {
        EEPROM.write(0, 1);
    }
}

/* This formats the log like this: [date][msg]["Fehler!"][<cr>] */
void writeToSD(bool printDate, const __FlashStringHelper *msg,
            bool noInput, bool linebreak)
{
    File sdFile;
    sdFile = SD.open("HUB.TXT", FILE_WRITE);
    
    if (sdFile) {
        if (printDate) {
            DateTime now = rtc.now();
            int myear, mmonth, mday, mhour, mminute, msecond;

#define X(a, b, c) \
    m##a = b; \
    if (m##a < 10) { \
        sdFile.print('0'); \
    } \
    sdFile.print(m##a, DEC); \
    sdFile.print(c);
    X_RTC_L
            sdFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
            sdFile.print(F(") "));
    X_RTC_R
#undef X

        }
        if (noInput) {
            sdFile.print(F("Eingang ist 0: "));
        }
        sdFile.print(msg);
        if (linebreak) {
            sdFile.println();
        }
        sdFile.close();
    } else {
        showError();
    }
}

void fill4Bytes(byte *pr, short num)
{
    for (int i = 3; i >= 0; i--) {
        *(pr + i) = num % 10;
        num /= 10;
    }
}

void readBladesPos()
{
    int inRead;
    float mA;
#define X(a,b,c) \
    inRead = analogRead(a); \
    mA = 2.226 * inRead + 1.378; \
    total_##b -= readings_##b[readIndex]; \
    readings_##b[readIndex] = (short) mA; \
    total_##b += readings_##b[readIndex]; \
    if ((readings_##b[readIndex] < MIN_POS) || (readings_##b[readIndex] > MAX_POS)) { \
        if (ERRORS[s##b] == false) { \
            writeToSD(true, c, true, true); \
            ERRORS[s##b] = true; \
        } \
    } else { \
        ERRORS[s##b] = false; \
    }
    X_POS
#undef X
    readIndex++;
}

void readDigIN()
{
    int inRead;
#define X(a, b) \
    inRead = digitalRead(a); \
    if (inRead) { \
        if (ERRORS[s##a] == false) { \
            writeToSD(true, b, true, true); \
            ERRORS[s##a] = true; \
        } \
    } else { \
        ERRORS[s##a] = false; \
    }
    X_DIGIN
#undef X

    P107_OK = !ERRORS[sP107_A] && !ERRORS[sP107_B] && !ERRORS[sP107_C];
    VCU_OK  = !ERRORS[sVCU_A]  && !ERRORS[sVCU_B]  && !ERRORS[sVCU_C];
}

bool arrCmp(const byte arr1[], const byte arr2[], int len)
{
    for (int i = 0; i < len; i++) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}

/* part of cmd mode */
void cmd_help()
{
    Serial.println();
    Serial.println(F("Commands: "));
#define X(a, b, c, d, e) \
    Serial.print('\t'); \
    Serial.print(b); \
    Serial.print(c); \
    Serial.print(d); \
    Serial.print(F(": ")); \
    Serial.println(e);
    X_CMD
#undef X
}

/* part of cmd mode */
void cmd_silence()
{
    silence = true;
}

/* part of cmd mode */
void cmd_moveOn()
{
    silence = false;
}

/* part of cmd mode */
void cmd_showAll()
{
#define X(a, b, c) \
    Serial.print(c); \
    Serial.print(F(": ")); \
    Serial.print(b); \
    Serial.print(F(" | "));
    X_POS
#undef X
    Serial.println('\n');

#define X(a, b) \
    Serial.print(b); \
    Serial.print(F(": ")); \
    Serial.print(!ERRORS[s##a]); \
    Serial.print(F(" | "));
    X_DIGIN
#undef X
    Serial.println('\n');

    Serial.print(F("VCU_VIN: "));
    Serial.print(VCU_VIN);
    Serial.print(F(" | "));
    Serial.print(F("POS_REF: "));
    Serial.println(POS_REF);

    Serial.println();
}

/* part of cmd mode */
void cmd_showTime()
{
    DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

/* part of cmd mode */
void cmd_printSD()
{
    File myFile;
    myFile = SD.open("HUB.TXT");
    if (myFile) {
        Serial.println(F("HUB.TXT:"));
        while (myFile.available()) {
            Serial.write(myFile.read());
        }
        myFile.close();
    } else {
        Serial.println(F("error opening HUB.TXT"));
    }
}

/* part of cmd mode */
void cmd_delSD()
{
    Serial.println(F("Removing ..."));
    SD.remove("HUB.TXT");
}

/* part of cmd mode */
void cmd_clearERR()
{
    Serial.println(F("Resetting errors ..."));
    HWError = false;
    EEPROM.write(0, 0);
    digitalWrite(FB, LOW);
}

void processBuff()
{
    /* process the data in normal (not cmd) mode */
    const byte st_mode[] = { 1, 1, 1 };
    if (arrCmp((buff + 1), st_mode, CMDLEN)) {
        /* read VCU_VIN */
        if (VCU_VIN != buff[POS_BUFF_VCU]) {
            if (VCU_VIN) {
                writeToSD(true, F("Nabe AUS"), false, true);
            } else {
                writeToSD(true, F("Nabe AN"), false, true);
            }
        }
        VCU_VIN = buff[POS_BUFF_VCU];
        
        /* read POS_REF (4 bytes) */
        POS_REF = 0;
        for (int i = 0; i < 4; i++) {
            POS_REF *= 10;
            POS_REF += buff[POS_BUFF_PREF + i];
        }
    
        lastTime = millis();
        linkOK = true;
        quickBlink(1,50,0);
        return;
    }

    /* cmd mode */
#define X(a, b, c, d, e) \
    const byte CMD_##a[] = {b, c, d}; \
    if (arrCmp((buff + 1), CMD_##a, CMDLEN)) { \
        cmd_##a(); \
        return; \
    }
    X_CMD
#undef X
}

void calculate_analog()
{
#define X(a,b,c) \
    b = total_##b / SMOOTHING;
    X_POS
#undef X
}

void sendData()
{
/**
 * protocol HUB->A3 :
 * <START> VCU  P107    PosA      PosB      PosC     <END>
 *   ...  [0,1] [0,1] [4 bytes] [4 bytes] [4 bytes]   ...
 *   0     1     2     3,4,5,6   7,8,9,10 11,12,13,14  15
 */
    byte protocol[HUBtoA3] = {0};
    protocol[0]  =  START_SEQ;
    protocol[HUBtoA3 - 1] =  END_SEQ;

    protocol[1] = VCU_OK;
    protocol[2] = P107_OK;

    calculate_analog();
    
    fill4Bytes(&protocol[3], POS_A);
    fill4Bytes(&protocol[7], POS_B);
    fill4Bytes(&protocol[11], POS_C);

    Serial.write(protocol, HUBtoA3);
}

/* check for silence/ broken connection */
void checkTime()
{
    unsigned long nowTime = millis();
    if (nowTime - lastTime >= MAX_SILENCE) {
        linkOK = false;
    }
}

void receiveData()
{
/**
 * protocol A3->HUB :
 * <START> CMD   VCU_VIN  POS_REF  <END>
 *   ...   1,1,1 [0,1]  [4 bytes]  ...
 *   0    CMDLEN   1     2,3,4,5    6
 */
    int inByte;

    while (Serial.available()) {
        inByte = Serial.read();
        buff[inCount++] = inByte;

        /* long enough? */
        if (inCount == A3toHUB) {

            /* properly formated? */
            if ((inByte == END_SEQ) &&
                    (buff[0] == START_SEQ)) {
                inCount = 0;
                processBuff();
                return;

            /* not allowed state */
            } else {
                inByte = 0;
            }

        /* starting over */
        } else if (inByte == START_SEQ) {
            buff[0] = START_SEQ;
            inCount = 1;
        }
    }
}

void writeData()
{
    /* error */
    if (!linkOK) {
        digitalWrite(VCU_OUT, LOW);
        dac.analogWrite(0, 0, 0, 0);
        return;
    }

    digitalWrite(VCU_OUT, VCU_OK && VCU_VIN);

    float mA = POS_REF / 100.0;
    int mcp = 178.03 * mA - 2.02;
    dac.analogWrite(mcp, 0, 0, 0);
}

void setup()
{
    /* external AREF */
    analogReference(EXTERNAL);

    Serial.begin(115200);
    while (!Serial) ;

    /* IN-/OUTPUTS */
#define X(a, b) \
    pinMode(a, INPUT);
    X_DIGIN
#undef X
    pinMode(VCU_OUT, OUTPUT);
    pinMode(FB, OUTPUT);
    digitalWrite(VCU_OUT, LOW);
    digitalWrite(FB, LOW);

    /* keep showing errors after reboot */
    if (checkEEPROM()) {
        showError();
    }

    /* SD Card */
    if (!SD.begin(10)) {
        showError();
    }

    /* RTC */
    if (! rtc.begin()) {
        writeToSD(false, F("RTC nicht gefunden!"), true, true);
        showError();
    }  
    if (rtc.lostPower()) {
        writeToSD(false, F("Die Uhr ist verstellt!"), true, true);
        showError();
    }

    /* DAC - internal reference with gain = 2 */
    dac.begin();
    dac.vdd(4096);
    dac.setVref(1,1,1,1);
    dac.setGain(1,1,1,1);

    /* keep timestaps of newstarts */
    writeToSD(true, F("Neustart."), false, true);

    /* Init Okay */
    quickBlink(2, 100, 100);
}

void loop()
{
    readBladesPos();
    if (readIndex == SMOOTHING) {
        readIndex = 0;

        readDigIN();
        if (!silence) {
            sendData();
        }
        receiveData();
        checkTime();
        writeData();
    }
    delay(HEARTBEAT);
}
