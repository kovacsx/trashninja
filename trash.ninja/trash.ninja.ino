#include "Ultrasonic.h"
#include "DHT.h"
#include "Sodaq_N2X.h"
#include "Sodaq_wdt.h"

#define STARTUP_DELAY 5000

#define DEBUG_STREAM_BAUD 115200

#if defined(ARDUINO_SODAQ_SARA)
/* SODAQ SARA AFF*/
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1
#define powerPin SARA_ENABLE
#define enablePin SARA_TX_ENABLE
#define MODEM_ON_OFF_PIN SARA_ENABLE

#elif defined(ARDUINO_SODAQ_SFF)
/* SODAQ SARA SFF*/
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial
#define powerPin SARA_ENABLE
#define enablePin SARA_TX_ENABLE

#else
#error "Please use one of the listed boards or add your board."
#endif

#define ADC_AREF 3.3f
#define BATVOLT_R1 4.7f
#define BATVOLT_R2 10.0f
#define BATVOLT_PIN BAT_VOLT

#include <Sodaq_LSM303AGR.h>

Sodaq_LSM303AGR AccMeter;

#define ACC_INT_PIN ACCEL_INT1
// Threshold for interrupt trigger
double threshold = -0.8;
int now;

uint16_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));

    return voltage;
}

// NL LMT NB
const char* apn = "nb-iot.lmt.lv";
const char* forceOperator = "24701"; // optional - depends on SIM / network (network Local) Country 
uint8_t urat = 20;

#define DHTPIN 15 
//correct pin
//#define DHTPIN 15 
#define DHTTYPE DHT22  

int analogPin = A6;
int ADC_value = 0;
int gas_intensity_value  = 0;

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
// FIRE sensor SENSOR
#define inPin A4 
//int inPin = A0;
bool val = 0;

DHT dht(DHTPIN, DHTTYPE);
Ultrasonic ultrasonic(inPin);

Sodaq_N2X n2x;

float h = 0.0;
float t = 0.0;
long RangeInInches;
long RangeInCentimeters;

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------


void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void BLANK() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

void sendMessageThroughUDP()
{   
    DEBUG_STREAM.println();
    DEBUG_STREAM.println("Sending message through UDP");

    int localPort = 16666;
    int socketID = n2x.socketCreate(localPort);
    if (socketID >= 7 || socketID < 0) {
        DEBUG_STREAM.println("Socket:" + socketID);
        DEBUG_STREAM.println("Failed to create socket");
        return;
    }

    DEBUG_STREAM.println("Created socket!");

    uint16_t battValue = getBatteryVoltage();

    String battStr = battValue > 4000 ? "FULL" : (battValue < 3400 ? "LOW" : "NORMAL");

    if(isnan(t)) t = -1;
    if(isnan(h)) h = -1;
    
    String deviceId = "RJjshhOeWjawudtf0fG7qhYU";
    String token = "maker:4PkvfZC55WkQW1VeVm0L6D4svzUVXKplRMESr39";
    //String value = getValues(); //"{\"t\":{\"value\":" + String(getBoardTemperature()) +"}}";
    String value = "{" +
          String("\"distance1\":{\"value\":") + String(RangeInCentimeters) +"},"
          + String("\"temp\":{\"value\":") + String(t) +"},"
          + String("\"humid\":{\"value\":") + String(h) +"},"
          //+ String("\"accX\":{\"value\":") + String(AccMeter.getX()) +"},"
          //+ String("\"accY\":{\"value\":") + String(AccMeter.getY()) +"},"
          + String("\"accZ\":{\"value\":") + String(AccMeter.getZ()) +"},"
          + String("\"battLevel\":{\"value\":\"") + battStr +"\"},"
          + String("\"gas\":{\"value\":") + String(ADC_value) +"}"
          + "}";


  DEBUG_STREAM.println("payload:'" + value + "'");

    String reading = deviceId + '\n' + token + '\n' + value;

    uint8_t size = reading.length();
    int lengthSent = n2x.socketSend(socketID, "40.68.172.187", 8891, (uint8_t*)reading.c_str(), size);
    n2x.socketClose(socketID);
    
    DEBUG_STREAM.println("Sent bytes: " + lengthSent);
}

static Sodaq_SARA_N211_OnOff saraR4xxOnOff;

bool color = true;

void interrupt_event()
{
    // Do not print in an interrupt event when sleep is enabled.
    DEBUG_STREAM.println("Board flipped");
}

void setup()
{

  now = millis();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
    sodaq_wdt_safe_delay(STARTUP_DELAY);

    dht.begin();
      
    DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);

 #ifdef powerPin
    // Turn the nb-iot module on
    pinMode(powerPin, OUTPUT); 
    digitalWrite(powerPin, HIGH);
    DEBUG_STREAM.println("powerPin!");
 #endif
  
  #ifdef enablePin
    // Set state to active
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
    DEBUG_STREAM.println("enablePin!");
  #endif // enablePin

    Wire.begin();

    AccMeter.rebootAccelerometer();
    delay(1000);

    // Enable the Accelerometer
    AccMeter.enableAccelerometer();

    // Attach interrupt event fot the Accelerometer
    pinMode(ACC_INT_PIN, INPUT);
    attachInterrupt(ACC_INT_PIN, interrupt_event, RISING);

    // Enable interrupts on the SAMD
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
        GCLK_CLKCTRL_GEN_GCLK1 |
        GCLK_CLKCTRL_CLKEN;

    // If Z goes below threshold the interrupt is triggered
    AccMeter.enableInterrupt1(AccMeter.ZLow, threshold, 0, AccMeter.PositionRecognition);
  
    DEBUG_STREAM.println("Initializing and connecting... "); 

    MODEM_STREAM.begin(n2x.getDefaultBaudrate());

    n2x.setDiag(DEBUG_STREAM);
    n2x.init(&saraR4xxOnOff, MODEM_STREAM);
    if (n2x.on()) {
       DEBUG_STREAM.println("turning modem on");
    } else {
       DEBUG_STREAM.println("failed to turn modem on");
       //return;
    }
    if (!n2x.connect(apn,"0",forceOperator, urat)) {
       DEBUG_STREAM.println("FAILED TO CONNECT TO MODEM");
       return;
    } else {
      DEBUG_STREAM.println("connected!");
    }  
}


void loop()
{


    h = dht.readHumidity();
    t = dht.readTemperature();

    DEBUG_STREAM.print("Temp: ");
    DEBUG_STREAM.print(t);
    DEBUG_STREAM.print(" Celsius ");
    DEBUG_STREAM.print("Hum: ");
    DEBUG_STREAM.print(h);
    DEBUG_STREAM.println(" %RH");

    RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
    DEBUG_STREAM.print(RangeInCentimeters);//0~400cm
    DEBUG_STREAM.println(" cm");

    ADC_value = analogRead(analogPin);  
    DEBUG_STREAM.print("GAS value: ");
    DEBUG_STREAM.println(ADC_value);

    DEBUG_STREAM.print("X acc: " + String(AccMeter.getX()));
    DEBUG_STREAM.print(", Y acc: " + String(AccMeter.getY()));
    DEBUG_STREAM.print(", Z acc: " + String(AccMeter.getZ()));
    DEBUG_STREAM.println();

    DEBUG_STREAM.println("Battery: " + String(getBatteryVoltage()));
    
    sendMessageThroughUDP();

    if(color) {
      RED();
    } else {
      BLANK();
    }

    color = !color;
    delay(1000);
}
