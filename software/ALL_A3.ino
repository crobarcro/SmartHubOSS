/**
 * ---------------------------
 *     >>> SmartHub <<<
 * Author: Vladimir Zhelezarov
 * 
 *      Software for A3
 * ---------------------------
 *  
 * - Initial build (18.06.18)
 * - Add CMDs in the protocoll (20-21.06.18)
 * - Smoothing (01.07.18)
 * - cleanup (08.07.18)
 */
#define VERSION 180708

#include "mcp4728.h"
static mcp4728 dac = mcp4728(0x60);

/* communication cycle rate */
#define HEARTBEAT 100
#define SMOOTHING 6
#define MAX_SILENCE 1000
static bool linkOK = false;

/* https://www.arduino.cc/en/Tutorial/Smoothing */
static int readIndex = 0;
static short readings_POS_REF[SMOOTHING] = {0};
static int total_POS_REF = 0;

/* inputs - digital */
#define VCU_IN   2

/* outputs - digital */
#define VCU_OUT  3
#define P107_OUT 4
#define FB       9

/* inputs - analog */
#define POS_REF_IN A0

/* for talking with HUB */
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

/* assigned but currently unused in A3 */
static bool ERRORS = false;

/* length of the protocolls */
#define CMDLEN 3
/* START + DATA + END */
#define HUBtoA3 16
/* START + CMD + DATA + END */
#define A3toHUB (7 + CMDLEN)

/* buffer for communication */
static byte buff[HUBtoA3] = {0};
static int inCount = 0;

/* VCU, POS_REF start at pos .. */
#define POS_BUFF_VCU 1
#define POS_BUFF_P107 2
#define POS_BUFF_POSA 3
#define POS_BUFF_POSB 7
#define POS_BUFF_POSC 11

/* watch for stable link */
static unsigned long lastTime = 0;

/* X-Macros */
#define X_POS \
    X(A) \
    X(B) \
    X(C)

void quickBlink(int cycles, int timeA, int timeB)
{
    for (int i = 0; i < cycles; i++) {
        digitalWrite(FB, HIGH);
        delay(timeA);
        digitalWrite(FB, LOW);
        delay(timeB);
    }
}

/* not used now in A3 */
void panic()
{
    /* TODO: maybe show the errors from the hub here too */
    return;
}

void fill4Bytes(byte *pr, short num)
{
    for (int i = 3; i >= 0; i--) {
        *(pr + i) = num % 10;
        num /= 10;
    }
}

void readPosREF()
{
    int inRead;
    float mA;
    inRead = analogRead(POS_REF_IN);
    mA = 2.226 * inRead + 1.378;
    total_POS_REF -= readings_POS_REF[readIndex];
    readings_POS_REF[readIndex] = (short) mA;
    total_POS_REF += readings_POS_REF[readIndex];
    if ((readings_POS_REF[readIndex] < MIN_POS) || (readings_POS_REF[readIndex] > MAX_POS)) {
        if (ERRORS == false) {
            ERRORS = true;
        }
    } else {
        ERRORS = false;
    }
    readIndex++;
}

void readDigIN()
{
    VCU_VIN = digitalRead(VCU_IN) == 0;
}

void calculate_analog()
{
    POS_REF = total_POS_REF / SMOOTHING;
}

void sendData()
{
/**
 * protocol A3->HUB :
 * <START> CMD   VCU_VIN  POS_REF  <END>
 *   ...   1,1,1 [0,1]  [4 bytes]  ...
 *   0    CMDLEN   1     2,3,4,5    6
 */
    byte protocol[A3toHUB] = {0};
    protocol[0]  =  START_SEQ;
    for (int i = 0; i < CMDLEN; i++) {
        /* CMD_normal = {1} */
        protocol[1 + i] = 1;
    }
    protocol[A3toHUB - 1] =  END_SEQ;

    protocol[CMDLEN + 1] = VCU_VIN;

    calculate_analog();

    fill4Bytes(&protocol[CMDLEN + 2], POS_REF);

    Serial.write(protocol, A3toHUB);
}

void processBuff()
{
    /* read VCU/P107_OK */
    VCU_OK = buff[POS_BUFF_VCU];
    P107_OK = buff[POS_BUFF_P107];

    /* read POS_[A,B,C] (Total: 3 x 4 bytes) */
#define X(a) \
    POS_##a = 0; \
    for (int i = 0; i < 4; i++) { \
        POS_##a *= 10; \
        POS_##a += buff[POS_BUFF_POS##a + i]; \
    }
    X_POS
#undef X

    lastTime = millis();
    linkOK = true;
    quickBlink(1,50,0);
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
 * protocol HUB->A3 :
 * <START> VCU  P107    PosA      PosB      PosC     <END>
 *   ...  [0,1] [0,1] [4 bytes] [4 bytes] [4 bytes]   ...
 *   0     1     2     3,4,5,6   7,8,9,10 11,12,13,14  15
 */
    int inByte;

    while (Serial.available()) {
        inByte = Serial.read();
        buff[inCount++] = inByte;

        /* long enough? */
        if (inCount == HUBtoA3) {

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
        digitalWrite(P107_OUT, LOW);
        dac.analogWrite(0, 0, 0, 0);
        return;
    }

    digitalWrite(VCU_OUT, VCU_OK);
    digitalWrite(P107_OUT, P107_OK);

    float mA; int mcpA, mcpB, mcpC;

#define X(a) \
    mA = POS_##a / 100.0; \
    mcp##a = 178.03 * mA - 2.02;
    X_POS
#undef X
    dac.analogWrite(mcpA, mcpB, mcpC, 0);
    
}

void setup()
{
    /* external AREF */
    analogReference(EXTERNAL);

    Serial.begin(115200);
    while (!Serial) ;

    /* IN-/OUTPUTS */
    pinMode(VCU_IN, INPUT);
    pinMode(VCU_OUT, OUTPUT);
    pinMode(P107_OUT, OUTPUT);
    pinMode(FB, OUTPUT);
    digitalWrite(VCU_OUT, LOW);
    digitalWrite(P107_OUT, LOW);
    digitalWrite(FB, LOW);

    /* DAC - internal reference with gain = 2 */
    dac.begin();
    dac.vdd(4096);
    dac.setVref(1,1,1,1);
    dac.setGain(1,1,1,1);

    /* Init Okay */
    quickBlink(2, 100, 100);
}

void loop()
{
    readPosREF();
    if (readIndex == SMOOTHING) {
        readIndex = 0;

        readDigIN();
        sendData();
        receiveData();
        checkTime();
        writeData();
    }
    delay(HEARTBEAT);
}
