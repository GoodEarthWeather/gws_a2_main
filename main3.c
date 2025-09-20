
#include "main.h"
#include "hal_rf.h"

// define state machine
uint8_t state;  // holds current state

#define STATE_SLEEP 1
#define STATE_WAKEUP 2
#define STATE_EXECUTE_COMMAND 3
#define STATE_LISTEN 4

#define INCLUDING_PRESSURE 1
#define NOT_INCLUDING_PRESSURE 0


// define instance of results structure
struct sysResults wxOut;
struct sysResultsTime wxOutTime;
struct systemStatus sysState;
uint8_t commandBuffer[30];  // to hold command from console
volatile uint8_t validCommand;  // true when command assembly is complete
extern uint8_t dataChannel;

void main(void)
{
    extern uint8_t timeOut;
    extern uint8_t uartPresent;

    WDT_A_hold(WDT_A_BASE);

    sysState.upTimer = 0; // reset up time counter on power up or reset
    initGPIO();
    initClocks();
    initRain();
    initWind();
    initRTC(); // finish interrupt handling for 5 minute flag first
    initADC();
    initPressure();
    Ref_A_disableTempSensor(REF_A_BASE);  // disable internal temperature sensor
    initRadio();

    // now go to sleep and wait for 1 minute flag to wake up
    state = STATE_SLEEP;

    while (1)
    {
        switch (state)
        {
        case STATE_SLEEP :
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);  // set TP1 low when radio turns off
            halRfRadioSleep();  // put radio to sleep
            __bis_SR_register(LPM3_bits + GIE);
            state = STATE_WAKEUP;
            break;

        case STATE_WAKEUP :
            // woke up from timer, so request console for any commands to execute
            // Before waking up radio, do measurements
            measureI2C(INCLUDING_PRESSURE);
            measureRain(0); // get rain count, but don't clear count yet
            measureADC();  // measure voltages
            recordWind();
            getNextChannel();  // get next data channel to use - sets dataChannel to channel to use
            buildPacket(CMD_NODE_READY);  // prepare simple packet results
            GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);  // set TP1 high when radio turns on
            halRfWakeUpChip();  // wake up radio
            sendPacket(SYNC_RADIO_CHANNEL);
            // now listen for response
            state = STATE_LISTEN;
            break;

        case STATE_LISTEN :
            commandBuffer[2] = 0;  // set to CMD_NULL
            validCommand = 0;
            startTimeOut();
            rxRecvPacket(commandBuffer,dataChannel);  // radio mode receive packet
            stopTimeOut();
            timeOut ? (state = STATE_SLEEP) : (state = STATE_EXECUTE_COMMAND);
            break;

        case STATE_EXECUTE_COMMAND :
            if (validCommand) // only execute if there was a valid command received and there was no timeout
            {
                executeCommand();
            }
            else
            {
                state = STATE_SLEEP;  // no commands, so go back to sleep
            }

            break;
        default :
            break;
        }
    }
}

// Function to execute command in the commandBuffer
void executeCommand(void)
{
    extern uint32_t epochTime;
    extern uint16_t wakeUpTimePeriod;

    switch ( commandBuffer[2] )  // switch on command ID
    {
    case CMD_SET_SLEEP :
        // receiving the sleep command is confirmation that rain count was received
        // so go ahead and clear rain count
        clearRain();
        state = STATE_SLEEP;
        break;

    case CMD_NULL :
        state = STATE_SLEEP;
        break;


    case CMD_SET_WAKEUP :
        wakeUpTimePeriod = commandBuffer[3];
        state = STATE_SLEEP;
        break;

    case CMD_REQ_HT :  // request for humidity, temperature
        // don't need to measure - it was done on wakeup
        //measureI2C(NOT_INCLUDING_PRESSURE);  // this measures HT - need to exclude pressure
        buildPacket(CMD_REQ_HT);  // builds response packet
        sendPacket(dataChannel);  // send response
        state = STATE_LISTEN;  // wait for next command
        break;

    case CMD_SET_NODE_ETIME :  // set epoch time
        epochTime = (uint32_t)(commandBuffer[3]+((uint32_t)(commandBuffer[4])<<8)+((uint32_t)(commandBuffer[5])<<16)+((uint32_t)(commandBuffer[6])<<24));
        buildPacket(CMD_ACK);  // acknowledge
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_NODE_ETIME :  // query epoch time
        buildPacket(CMD_REQ_NODE_ETIME);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_WIND :
        buildPacket(CMD_REQ_WIND); // wind values have already been stored in wxOut
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_RAIN :
    case CMD_REQ_CLR_RAIN :
        measureRain(commandBuffer[3]);
        buildPacket(CMD_REQ_RAIN);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_POWER :
        measureADC();
        buildPacket(CMD_REQ_POWER);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_HTP :
        buildPacket(CMD_REQ_HTP);  // builds response packet
        sendPacket(dataChannel);  // send response
        state = STATE_LISTEN;  // wait for next command
        break;

    case CMD_REQ_SIMPLE :
        buildPacket(CMD_REQ_SIMPLE);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_PRESSURE :
        buildPacket(CMD_REQ_PRESSURE);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_SET_RFID :
        setRFID(commandBuffer[3], commandBuffer[4]);
        buildPacket(CMD_ACK);  // acknowledge
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_NODE_RSSI :
        getRSSI();
        buildPacket(CMD_REQ_NODE_RSSI);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_REQ_NODE_SYS :
        buildPacket(CMD_REQ_NODE_SYS);
        sendPacket(dataChannel);
        state = STATE_LISTEN;
        break;

    case CMD_CLR_NODE_SYS : // clear system status structure (except for time)
        sysState.timeOutCount = 0;
        sysState.crcErrorCount = 0;
        sysState.fifoErrorCount = 0;
        sysState.lengthErrorCount = 0;
        sysState.pktRXCount = 0;
        sysState.pktTXCount = 0;
        sysState.reTryCount = 0;
        state = STATE_LISTEN;
        break;

    case CMD_SET_NODE_RESET :
        // send command to do a software power on reset
        PMM_trigPOR();
        break;

    default:
        /*
        buildPacket(CMD_ACK);  // for now just ack - but should ack with error here
        sendPacket();
        state = STATE_LISTEN;
        */
        state = STATE_SLEEP;
        break;
    }
}
