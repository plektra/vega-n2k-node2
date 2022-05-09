#include <Arduino.h>
#include "ota.h"

#include <ReactESP.h>
using namespace reactesp;
ReactESP app;

#define CAN_TX_PIN GPIO_NUM_32
#define CAN_RX_PIN GPIO_NUM_34
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

#define LED_PIN 2
#define SDA_PIN 16
#define SCL_PIN 17
#define ONEWIRE_PIN 4

#include <Wire.h>
#include <INA226.h>
#include "DallasTemperature.h"
#include "OneWire.h"

INA226 ina(Wire);
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature* ds;
tNMEA2000* n2k;

const unsigned long TransmitMessages[] PROGMEM={
  127508L,
  61200L,
  0
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void RebootHandler(const tN2kMsg &N2kMsg);
void readINA();
void checkConfig();
void sendBatteryStatus(unsigned char instance);
double ReadBatteryTemperature(unsigned char instance);

tNMEA2000Handler NMEA2000Handlers[]={
  {61200L,&RebootHandler},
  {0,0}
};

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200); delay(500);

  // Download latest firmware and upgrade automatically
  if (!OTAUpdate()) {
    for (int i=0; i<=10; i++) {
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
    }
  }
  digitalWrite(LED_PIN, LOW);

  // Set up CAN bus and NMEA2000
  n2k = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  n2k->SetProductInformation("00000002");
  n2k->SetDeviceInformation(2, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  n2k->SetForwardStream(&Serial);
  n2k->SetForwardType(tNMEA2000::fwdt_Text);
  n2k->SetMode(tNMEA2000::N2km_ListenAndNode, 2);
  n2k->EnableForward(false);
  n2k->ExtendTransmitMessages(TransmitMessages);
  n2k->SetMsgHandler(HandleNMEA2000Msg);
  n2k->Open();

  // Set up DS18B20 temperature sensor using 1-wire
  ds = new DallasTemperature(&oneWire);
  ds->begin();

  // Set up i2c
  Wire.begin(SDA_PIN, SCL_PIN);
  
  bool success = ina.begin(0x40);

  // Check if the connection was successful, stop if not
  if(!success)
  {
    Serial.println("Connection error");
    while(1);
  }

  // Configure INA226
  ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

  // Calibrate INA226. Rshunt = 0.00015 ohm, Max expected current = 200A
  ina.calibrate(0.00015F, 200.0F);

  // Display INA226 configuration
  checkConfig();

  Serial.println("-----------------------------------------------");

  // Parse incoming messages every 1ms
  app.onRepeat(1, []() {
    n2k->ParseMessages();
  });

  // Send data every 2,5 sec
  app.onRepeat(2500, []() {
    sendBatteryStatus(1);
  });
}

void loop() {
  app.tick();
}

void sendBatteryStatus(unsigned char instance) {
  tN2kMsg n2kMsg1;
  tN2kMsg n2kMsg2;
  double voltage = ina.readBusVoltage();
  double current = ina.readShuntCurrent();
  double temperature = ReadBatteryTemperature(instance-1);
  double temperatureC = KelvinToC(temperature);
  double charge = 6.03 + -0.82 * voltage + 0.0244 * voltage * voltage;
  double capacityAh = 85.0 / (1.34 - 0.0285 * temperatureC + 0.000622 * temperatureC * temperatureC);
  double timeRemaining = (capacityAh * 3600 * charge) / current;

  SetN2kDCBatStatus(n2kMsg1, instance, voltage, current, temperature);
  SetN2kDCStatus(n2kMsg2, 1, instance, N2kDCt_Battery, round(charge*100), 1, timeRemaining);
  if (n2k->SendMsg(n2kMsg1) && n2k->SendMsg(n2kMsg2)) {
    Serial.print(millis()); Serial.println(", Battery status sent");
  }
}

double ReadBatteryTemperature(unsigned char instance) {
  ds->requestTemperatures();
  return CToKelvin(ds->getTempCByIndex(instance));
}

void readINA()
{
  Serial.print("Bus voltage:   ");
  Serial.print(ina.readBusVoltage(), 5);
  Serial.println(" V");

  Serial.print("Bus power:     ");
  Serial.print(ina.readBusPower(), 5);
  Serial.println(" W");


  Serial.print("Shunt voltage: ");
  Serial.print(ina.readShuntVoltage(), 5);
  Serial.println(" V");

  Serial.print("Shunt current: ");
  Serial.print(ina.readShuntCurrent(), 5);
  Serial.println(" A");

  Serial.println("");
}


void RebootHandler(const tN2kMsg &inboundMsg) {
  Serial.printf("Received message in PGN %ld\n", inboundMsg.PGN);
  Serial.println("Rebooting!");
  esp_restart();
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
  // Find handler
  //Serial.print("In Main Handler: "); Serial.println(N2kMsg.PGN);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}


void checkConfig()
{
  Serial.print("Mode:                  ");
  switch (ina.getMode())
  {
    case INA226_MODE_POWER_DOWN:      Serial.println("Power-Down"); break;
    case INA226_MODE_SHUNT_TRIG:      Serial.println("Shunt Voltage, Triggered"); break;
    case INA226_MODE_BUS_TRIG:        Serial.println("Bus Voltage, Triggered"); break;
    case INA226_MODE_SHUNT_BUS_TRIG:  Serial.println("Shunt and Bus, Triggered"); break;
    case INA226_MODE_ADC_OFF:         Serial.println("ADC Off"); break;
    case INA226_MODE_SHUNT_CONT:      Serial.println("Shunt Voltage, Continuous"); break;
    case INA226_MODE_BUS_CONT:        Serial.println("Bus Voltage, Continuous"); break;
    case INA226_MODE_SHUNT_BUS_CONT:  Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Samples average:       ");
  switch (ina.getAverages())
  {
    case INA226_AVERAGES_1:           Serial.println("1 sample"); break;
    case INA226_AVERAGES_4:           Serial.println("4 samples"); break;
    case INA226_AVERAGES_16:          Serial.println("16 samples"); break;
    case INA226_AVERAGES_64:          Serial.println("64 samples"); break;
    case INA226_AVERAGES_128:         Serial.println("128 samples"); break;
    case INA226_AVERAGES_256:         Serial.println("256 samples"); break;
    case INA226_AVERAGES_512:         Serial.println("512 samples"); break;
    case INA226_AVERAGES_1024:        Serial.println("1024 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus conversion time:   ");
  switch (ina.getBusConversionTime())
  {
    case INA226_BUS_CONV_TIME_140US:  Serial.println("140uS"); break;
    case INA226_BUS_CONV_TIME_204US:  Serial.println("204uS"); break;
    case INA226_BUS_CONV_TIME_332US:  Serial.println("332uS"); break;
    case INA226_BUS_CONV_TIME_588US:  Serial.println("558uS"); break;
    case INA226_BUS_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_BUS_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_BUS_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_BUS_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt conversion time: ");
  switch (ina.getShuntConversionTime())
  {
    case INA226_SHUNT_CONV_TIME_140US:  Serial.println("140uS"); break;
    case INA226_SHUNT_CONV_TIME_204US:  Serial.println("204uS"); break;
    case INA226_SHUNT_CONV_TIME_332US:  Serial.println("332uS"); break;
    case INA226_SHUNT_CONV_TIME_588US:  Serial.println("558uS"); break;
    case INA226_SHUNT_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_SHUNT_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_SHUNT_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_SHUNT_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Max possible current:  ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:           ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:     ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:             ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}
