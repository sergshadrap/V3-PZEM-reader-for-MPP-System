# V3-PZEM-reader-for-MPP-System
V3 PZEM reader for MPP System
 1 Phase V3 PZEM reader for MPP System ,  3 Phase version also  possible (it need 3 of PZEM and one ESP).
PZEM004V3 is a small module intended for power monitoring by "non-invasively" way (you don't need break,terminate current wires, just put on current transformator on the wire the current you would check, connect to wall electricity and to ESP8266 or to computer directly , over serial-usb connector. Due to it "non-invasively" implementation it can read quite wide range of current values - from 0.1A up to 100A.
 Aside the quite accurate figures of  current and voltage , it can accurately calculate the power , power factor ,total energy and frequency of AC. 
I've used multiple of them for quite long time , either in 3 Phase vesion for Home electricicty control or as 1 Phase version for country home.
By default I use Wemos D1 mini + Pzem module. This implementation uses SoftwareSerial , connected to GPIO 12(RX) and GPIO 14(TX) (D5,D6) accordingly.  Additionaly to Ground wire you have to connect  +5V wire to PZEM board, used for PZEM's optocoupler feeding.
 Parameters for setup form main page menu : "RX_PIN", "TX_PIN" - GPIO for TX and RX on PZEM, "COMMAND_MS" - how often the ESP will ask the PZEM for values (200ms minimum). "TIMEOUT_MS" - timeout for waiting the PZEM response.(300ms)
"Period"- how often the ESP will report to AM server (max 1 sec).
 "Reset Energy" - once set to true will reset accumulated energy value.
 Since started the device will arrange four analog devices - voltage, current,power ,energy ( I decided not to create PF and Freq , if someone need it , I can add).
This versiom doesn't use any libraries. All libraries had presented in the internet badly interracted with serial port , blocking whole device for long time, becuase of that the AM server periodically reported lost connections and timeouts. My implementation uses different algorithm .
