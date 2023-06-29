// 1 Phase V3 PZEM reader for MPP System ,  3 Phase version also  possible (it need 3 of PZEM and one ESP).
// PZEM004V3 is a small module intended for power monitoring by "non-invasively" way (you don't need break,terminate current wires, just put on current transformator on the wire the current you would check,
// connect to wall electricity and to ESP8266 or to computer directly , over serial-usb connector. Due to it "non-invasively" implementation it can read quite wide range of current values - from 0.1A up to 100A.
// Aside the quite accurate figures of  current and voltage , it can accurately calculate the power , power factor ,total energy and frequency of AC. 
// I/ve used multiple of them for quite long time , either in 3 Phase vesion for Home electricicty control or as 1 Phase version for country home.
// By default I use Wemos D1 mini + Pzem module. This implementation uses SoftwareSerial , connected to GPIO 12(RX) and GPIO 14(TX) (D5,D6) accordingly.  Additionaly to Ground wire you have to connect 
// +5V wire to PZEM board, used for PZEM's optocoupler feeding.
// Parameters for setup form main page menu : "RX_PIN", "TX_PIN" - GPIO for TX and RX on PZEM, "COMMAND_MS" - how often the ESP will ask the PZEM for values (200ms minimum). "TIMEOUT_MS" - timeout for waiting the PZEM response.(300ms)
// "Period"- how often the ESP will report to AM server (max 1 sec).
// "Reset Energy" - once set to true will reset accumulated energy value.
// Since started the device will arrange four analog devices - voltage, current,power ,energy ( I decided not to create PF and Freq , if someone need it , I can add).

#include <MppServer.h>
#include <MppDevice.h>

#include <SoftwareSerial.h>
#include <Arduino.h>
#include <ESP.h>
#include <ESP8266WiFi.h>


//
// * Pin 12 Rx (Connects to the Tx pin on the PZEM)
// * Pin 14 Tx (Connects to the Rx pin on the PZEM)

#include <stdio.h>

#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42

#define info_bytes      25

#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

#define UPDATE_TIME     200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 70

#define INVALID_ADDRESS 0x00

#define PZEM_DEFAULT_ADDR   0x1
#define PZEM_BAUD_RATE      9600

#define PZEM_RX_PIN 12
#define PZEM_TX_PIN 14


const char *DeviceVersion = "MppPZEM 3.3.1"; // New Algorithm of reading PZEM
static const char *P_PERIOD = "Period"; // Period - the frequency to read the analog input in sconds
static const char *RX_PIN = "RX_PIN";
static const char *TX_PIN = "TX_PIN";
static const char *COMMAND_MS = "COMMAND_MS";
static const char *TIMEOUT_MS = "TIMEOUT_MS";
static const char *Reset_En = "Reset Energy";

static const char *properties[] = { //
        P_PERIOD, //   how often reporting to AM server, ms
        RX_PIN, // RX pin GPIO
        TX_PIN, // TX  pin GPIO
        COMMAND_MS,//delay in sending commands to PZEM
        TIMEOUT_MS, // timeout after sending commands to PZEM
        Reset_En,
        NULL };

MppServer mppServer(DeviceVersion, properties);
class MppDevice volt;
class MppDevice curr;
class MppDevice powr;
class MppDevice ener;

unsigned long next = millis();
unsigned long last_command=millis();
boolean stop_comm=false;
static uint8_t _response[25];


static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};  

class PZEM004Tv30    {


public:


    uint8_t _addr;   // Device address

    bool _isConnected; // Flag set on successful communication

    struct {
        float voltage;
        float current;
        float power;
        float energy;
        float frequency;
        float pf;
        uint16_t alarms;
    }  _currentValues; // Measured values

    uint64_t _lastRead; // Last time values were updated
      bool _isSoft;    // Is serial interface software
      SoftwareSerial* localSWserial = nullptr; // Pointer to the Local SW serial object
      Stream* _serial; // Serial interface
   
PZEM004Tv30(uint8_t receivePin, uint8_t transmitPin, uint8_t addr)
{
    localSWserial = new SoftwareSerial(receivePin, transmitPin); // We will need to clean up in destructor
    localSWserial->begin(PZEM_BAUD_RATE);

    init((Stream *)localSWserial, true, addr);
}

PZEM004Tv30(SoftwareSerial& port, uint8_t addr)
{
    port.begin(PZEM_BAUD_RATE);
    init((Stream *)&port, true, addr);
}

PZEM004Tv30(Stream& port, uint8_t addr)
{
    init(&port, true, addr);
}

~PZEM004Tv30()
{
    // TODO: Remove local SW serial
    // This is not the correct way to do it. 
    // Best solution would be to completely remove local SW serial instance and not deal with it.
#if defined(PZEM004_SOFTSERIAL)
    if(this->localSWserial != nullptr){
        delete this->localSWserial;
    }
#endif
}



bool setAddress(uint8_t addr)
{
    if(addr < 0x01 || addr > 0xF7) // sanity check
        return false;

    // Write the new address to the address register
    if(!sendCmd8(CMD_WSR, WREG_ADDR, addr, true,addr)){
               return false;
    }
    _addr = addr; // If successful, update the current slave address

    return true;
}

uint8_t readAddress(bool update)
{
    static uint8_t response[7];
    uint8_t addr = 0;
    // Read 1 register
    if (!sendCmd8(CMD_RHR, WREG_ADDR, 0x01, false,addr)) {
             return INVALID_ADDRESS;}

delay(250);
    if(new_receive(response, 7) != 7){ // Something went wrong
 //     Serial.printf("\nBuffer of addr to read: %01X %01X %01X %01X %01X %01X %01X \n",response[0],response[1],response[2],response[3],response[4],response[5],response[6]);
        return INVALID_ADDRESS;
    }

    // Get the current address
    addr = ((uint32_t)response[3] << 8 | // Raw address
                              (uint32_t)response[4]);

    // Update the internal address if desired
    if(update){
        _addr = addr;
    }
    return addr;
}

uint8_t getAddress()
{
    return _addr;
}

void init(Stream* port, bool isSoft, uint8_t addr){
    if(addr < 0x01 || addr > 0xF8) // Sanity check of address
        addr = PZEM_DEFAULT_ADDR;
    _addr = addr;

    this->_serial = port;
    this->_isSoft = isSoft;

    // Set initial lastRed time so that we read right away
    _lastRead = -1;

    _isConnected = false; // We have not received anything yet...
}

bool sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr){
    uint8_t sendBuffer[8]; // Send buffer
    uint8_t respBuffer[8]; // Response buffer (only used when check is true)

    if((slave_addr == 0xFFFF) ||
       (slave_addr < 0x01) ||
       (slave_addr > 0xF7)){
        slave_addr = _addr;
    }

    sendBuffer[0] = slave_addr;                   // Set slave address
    sendBuffer[1] = cmd;                     // Set command

    sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
    sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=

    sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
    sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=

    setCRC(sendBuffer, 8);                   // Set CRC of frame
    
    while (_serial->available())  _serial->read(); // clear buffer !!
 //   Serial.printf("command : %02X %02X %02X %02X \n",cmd,rAddr,val,slave_addr);
// Serial.printf("\nBuffer to send: %01X %01X %01X %01X %01X %01X %01X %01X\n",sendBuffer[0],sendBuffer[1],sendBuffer[2],sendBuffer[3],sendBuffer[4],sendBuffer[5],sendBuffer[6],sendBuffer[7]);
    _serial->write(sendBuffer, 8); // send frame

    if(check) {
        if(!new_receive(respBuffer, 8)){ // if check enabled, read the response
            return false;
        }

        // Check if response is same as send
        for(uint8_t i = 0; i < 8; i++){
            if(sendBuffer[i] != respBuffer[i])
                return false;
        }
    }
    return true;
}


uint16_t new_receive(uint8_t *resp, uint16_t len)
{
unsigned index=0;
    if(_serial->available()==0 )return 0; 
    
   
 while (_serial->available() )  {
      uint8_t c = (uint8_t)_serial->read();
//      Serial.printf("Length:%01X , Bytes available:%01X , Byte of resp[%d]:%01X    ",len,_serial->available(),index,c);
      resp[index] = c;
      index++;
 }
  
//Serial.printf("\nBuffer to read: %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X %01X, index:%d\n",
//resp[0],resp[1],resp[2],resp[3],resp[4],resp[5],resp[6],resp[7],resp[8],resp[9],resp[10],resp[11],resp[12],resp[13],resp[14],resp[15],resp[16],resp[17],resp[18],resp[19],resp[20],resp[21],resp[22],resp[23],resp[24],index);

/// else Serial.printf("Empty buffer 1st byte:%01X , index:%d Serial bytes:%d \n",resp[0],index,_serial->available());
    // Check CRC with the number of bytes read
   
    if(!checkCRC(resp, index)){
        _isConnected = false; // We are no longer connected
        return 0;
    }

     _isConnected = true; // We received a reply 
    return index;
}




bool checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}

void setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}



uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

bool resetEnergy(){
    uint8_t buffer[] = {0x01, CMD_REST, 0x00, 0x00};
    uint8_t reply[5];
    buffer[0] = _addr;

    setCRC(buffer, 4);
    _serial->write(buffer, 4);
delay(250);
    uint16_t length = new_receive(reply, 5);

    Serial.printf("Reset Energy buffer[0]:%01X[1]:%01X [2]:%01X [3]:%01X . Reply :%01X %01X %01X %01X %01X \n", buffer[0],buffer[1],buffer[2],buffer[3], reply[0], reply[1], reply[2],reply[3], reply[4]);

    if(length == 0 || length == 5){
        return false;
    }

    return true;
}

} *pzem;

void setup() {

      Serial.begin(115200);
  mppServer.setPropertyDefault(P_PERIOD, "1000");
  mppServer.setPropertyDefault(RX_PIN, "12");
  mppServer.setPropertyDefault(TX_PIN, "14");
  mppServer.setPropertyDefault(COMMAND_MS, "250");
  mppServer.setPropertyDefault(TIMEOUT_MS, "300");
  mppServer.setPropertyDefault(Reset_En, "false");
  
  pzem = new class PZEM004Tv30(mppServer.getUnsignedProperty(RX_PIN),mppServer.getUnsignedProperty(TX_PIN),0x01);
      mppServer.setPropertyDefault(P_PERIOD, "1000");
      mppServer.manageDevice(&volt, getDefaultUDN(MppAnalog)+"V");
      mppServer.manageDevice(&curr, getDefaultUDN(MppAnalog)+"I");
      mppServer.manageDevice(&powr, getDefaultUDN(MppAnalog)+"P");
      mppServer.manageDevice(&ener, getDefaultUDN(MppAnalog)+"E");

  
    bool a=false;
    uint8_t adr=pzem->readAddress(true);
    Serial.printf("\nPZEM address: %02X\n",adr);
    if(adr < 0x01 || adr >= 0xF7) {
    a=pzem->setAddress(0x01);
    if(a) Serial.printf("NEW PZEM address: %02X\n",pzem->readAddress(true));
    else Serial.printf("Wrong PZEM address! a:%d\n",a);}
    delay(2000);
  
  Serial.printf("\nMppServer booting: %s, mode=%d, version=%d\n",
      ESP.getResetReason().c_str(), ESP.getBootMode(),
      ESP.getBootVersion());


      mppServer.begin();
}

void loop() {

           unsigned long now= millis();
           float v1,i1,p1,e1,f1,pf1,alarm1;

 mppServer.handleClients(); // let the server handle any incoming requests
 mppServer.handleCommand(); // optional, handle user Serial input 


if(!stop_comm) {
  pzem->sendCmd8(CMD_RIR, 0x00, 0x0A, false, 0x01);
  last_command=millis();
  stop_comm=true;
  }


if(millis()-last_command >mppServer.getUnsignedProperty(COMMAND_MS) &&  stop_comm) 
 {
 uint16_t re= pzem->new_receive(_response, info_bytes);  
 if(re != info_bytes ){ // Something went wrong
  Serial.printf("Wrong amount of bytes read:%d at %u\n",re,millis());
 } else {
  v1=((uint32_t)_response[3] << 8 | (uint32_t)_response[4])/10.0; // Raw voltage in 0.1V
  i1= ((uint32_t)_response[5] << 8 | (uint32_t)_response[6] | (uint32_t)_response[7] << 24 | (uint32_t)_response[8] << 16) / 1000.0; // Raw current in 0.001A
  p1= ((uint32_t)_response[9] << 8 | (uint32_t)_response[10] | (uint32_t)_response[11] << 24 | (uint32_t)_response[12] << 16) / 10.0; // Raw power in 0.1W
  e1= ((uint32_t)_response[13] << 8 | (uint32_t)_response[14] | (uint32_t)_response[15] << 24 | (uint32_t)_response[16] << 16) / 1000.0; // Raw Energy in 1Wh
  f1= ((uint32_t)_response[17] << 8 | (uint32_t)_response[18]) / 10.0; // Raw Frequency in 0.1Hz
  pf1=((uint32_t)_response[19] << 8 | (uint32_t)_response[20])/100.0; // Raw pf in 0.01
  alarm1=((uint32_t)_response[21] << 8 | (uint32_t)_response[22]); // Raw alarm value
 }
 while (pzem->_serial->available())  pzem->_serial->read(); // clear buffer 
    stop_comm=false; 


}  

 if(millis()-last_command > mppServer.getUnsignedProperty(TIMEOUT_MS) && stop_comm)  stop_comm=false; 

  if (now > next &&  stop_comm!=true) {

        Serial.printf(" Voltage :%.2fV  Current:%.2fA Power:%.2fW Energy:%.2fWt Freq:%.2fHz  Pf:%.2f   Alarm:%.2f\n",v1,i1,p1,e1,f1,pf1,alarm1); 
        volt.put(VALUE,String(v1)); volt.update(STATE,"on"); 
        curr.put(VALUE,String(i1)); curr.update(STATE,"on");
        powr.put(VALUE,String(p1)); powr.update(STATE,"on");
        ener.put(VALUE,String(e1)); ener.update(STATE,"on");

if(mppServer.isProperty((Reset_En))) {
  if(pzem->resetEnergy()) {
    Serial.println("Energy counter pinned to 0!");
    mppServer.putProperty(Reset_En,"false");
  }
}
Serial.printf("heap=%d at %lus \n", ESP.getFreeHeap(), now / 1000);
 next = now + mppServer.getUnsignedProperty(P_PERIOD);

  }


}
