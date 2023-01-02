// Sample usage of the DavisRFM69 library to sniff the packets from a Davis Instruments
// wireless Integrated Sensor Suite (ISS)

#include <DavisRFM69.h> // From https://github.com/dekay/DavisRFM69
#include <SPI.h>        // From standard Arduino library

// NOTE: *** One of DAVIS_FREQS_US, DAVIS_FREQS_EU, DAVIS_FREQS_AU, or
// DAVIS_FREQS_NZ MUST be defined at the top of DavisRFM69.h ***

#define IS_RFM69HW 1    // uncomment only for RFM69HW! Leave out if you have RFM69W!

#define PACKET_INTERVAL 2555
#define LOOP_INTERVAL 2500


/* ESP32 pinout */

// MISO  GPIO_NUM_12
// MOSI  GPIO_NUM_13
// SCLK  GPIO_NUM_14
#define RF69_NSS  GPIO_NUM_15
#define RF69_IRQ  GPIO_NUM_2

DavisRFM69 radio(RF69_NSS, RF69_IRQ, IS_RFM69HW);

struct __attribute__((packed)) MyData
{
  double rainRate;                  // Rain rate as number of rain clicks per hour (e.g 256 = 2.56 in/hr)
  double temperature;               // Outside temperature in tenths of degrees
  double humidity;                  // Outside relative humidity in %
  double uV;                        // UV index
  double solarRadiation;             // Solar radiation in Watts/m^2
  uint8_t windSpeed;                // Wind speed in kilometer per hour
  uint16_t windDirection;           // Wind direction from 1 to 360 degrees (0 = no wind data)

  // uint16_t dayRain;                 // Rain today sent as number of rain clicks (0.2mm or 0.01in)
  // uint16_t monthRain;               // Rain this month sent as number of rain clicks (0.2mm or 0.01in)
  // uint16_t yearRain;                // Rain this year sent as number of rain clicks (0.2mm or 0.01in)
  uint8_t transmitterBatteryStatus; // Transmitter battery status (0 or 1)
};

MyData myData;
PacketStats stats = {0, 0, 0, 0 ,0};

struct RainData {
  String state = "No rain";
  double rate = 0.0;
};

void setup()
{
  Serial.begin(115200);
  radio.initialize();
  radio.setChannel(0);  // Frequency / Channel *not* set in initialization. Do it right after.
  radio.setHighPower(); // Uncomment only for RFM69HW!
  Serial.println("Wind\nSpeed\tDir\tTemp.\tHum\tRain\nm/s\t360°\t°C\t%\tmm/h");
}

// See https://github.com/dekay/im-me/blob/master/pocketwx/src/protocol.txt
uint32_t lastRxTime = 0;
uint8_t hopCount = 0;

void loop()
{
  // Print updated data to Serial every 3 seconds
  printData();


  // The check for a zero CRC value indicates a bigger problem that will need fixing, but it needs to stay until the fix is in.
  if (radio.receiveDone())
  {
    stats.packetsReceived++;
    uint16_t crc = radio.crc16_ccitt(radio.DATA, 6);
    if ((crc == (word(radio.DATA[6], radio.DATA[7]))) && (crc != 0))
    {
      processPacket();
      stats.receivedStreak++;
      hopCount = 1;
    }
    else
    {
      stats.crcErrors++;
      stats.receivedStreak = 0;
    }

    // Whether CRC is right or not, we count that as reception and hop.
    lastRxTime = millis();
    radio.hop();
  }

  // If a packet was not received at the expected time, hop the radio anyway in an attempt to keep up.
  // Give up after 25 failed attempts. Keep track of packet stats as we go.
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * PACKET_INTERVAL + 200)))
  {
    stats.packetsMissed++;
    if (hopCount == 1)
      stats.numResyncs++;
    if (++hopCount > 25)
      hopCount = 0;
    radio.hop();
  }
}


void printData() {
    // Serial.print("UV Index: ");
    // radio.DATA[3] == 0xFF ? Serial.println(" no sensor.") : Serial.println(myData.uV);

        // Serial.print("Solar irradiation: ");
    // radio.DATA[3] == 0xFF ? Serial.println(" no sensor.") : Serial.println(myData.solarRadiation);

  static uint32_t printTime;
  if (millis() - printTime > 3000) {
    printTime = millis();
    char dataBuf[512];
    snprintf(dataBuf, sizeof(dataBuf),
      "%d\t%d\t%d.%d\t%d.%d\t%d.%d",
      myData.windSpeed, myData.windDirection, (int)myData.temperature, (int)(abs(myData.temperature)*10)%10,
      (int)myData.humidity, (int)(myData.humidity*10)%10, (int)myData.rainRate, (int)(myData.rainRate*10)%10
    );
    Serial.println(dataBuf);
  }
}

void printDataPacketValue(volatile uint8_t * data) {
  Serial.print(F("Data: "));
  for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++)
  {
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
  }
  Serial.print(F("  RSSI: "));
  Serial.println(radio.RSSI);
}

// Read the data from the ISS and figure out what to do with it
void processPacket()
{
  // Every packet has wind speed, direction, and battery status in it
  myData.windSpeed = radio.DATA[1];
  myData.windDirection = radio.DATA[2] * 360 / 255;
  // 1 for battery low
  myData.transmitterBatteryStatus = (radio.DATA[0] & 0x8) >> 3;

  // Now look at each individual packet. The high order nibble is the packet type.
  // The highest order bit of the low nibble is set high when the ISS battery is low.
  // The low order three bits of the low nibble are the station ID.
  switch (radio.DATA[0] >> 4)
  {
    case VP2P_TEMP: {
      // printDataPacketValue(radio.DATA);
      double tempF =  (double)word(radio.DATA[3], radio.DATA[4]) / 160.0 ;

      // Fahrenheit to Celsius
      myData.temperature = (tempF - 32.0) / 1.8;
      break;
    }
    case VP2P_HUMIDITY:
      // printDataPacketValue(radio.DATA);
      myData.humidity = (double)word(radio.DATA[4] >> 4, radio.DATA[3]) / 10.0;
      break;
    case VP2P_UV:
      myData.uV = (word(radio.DATA[3], radio.DATA[4]) >> 6) / 50.0 ;
      break;
    case VP2P_SOLAR:
      myData.solarRadiation = (word(radio.DATA[3], radio.DATA[4]) >> 6) * 1.757936;
      break;
    case VP2P_RAINSECS: {
      RainData rain = getRainState(radio.DATA);
      myData.rainRate = rain.rate;
      break;
    }
    case VP2P_RAIN: {
      // Serial.print("Rain bucket tips counter: ");
      // Serial.println(radio.DATA[3] && 0x7F);
      break;
    }

    default:
      break;
  }
}


RainData getRainState(volatile uint8_t* data) {
  RainData _rain;
  if (data[3] == 0xFF && data[4] >= 0x75) {
    _rain.state = "No rain";
    _rain.rate = 0.0;
    return _rain;
  }
  uint8_t factor = data[4] >> 4;
  double raw;
  if (factor < 4) {
    _rain.state = "Strong rain";
    raw =  data[3] + (factor * 256);
  }
  else {
    _rain.state = "Light rain";
    raw =  ((data[3] + (factor * 256)) * 16) - 16384;
  }
  _rain.rate =  11520 / raw;
  return _rain;
}
