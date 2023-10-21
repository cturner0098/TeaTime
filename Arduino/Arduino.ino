#include "dht_nonblocking.h"
#include "SoftwareSerial.h"

// Setup HC-06
static const int BT_RXD_PIN = 0;
static const int BT_TXD_PIN = 1;
SoftwareSerial BT(BT_RXD_PIN, BT_TXD_PIN);
char c = ' ';
String readString;

// Setup DHT11
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

void setup() {
  pinMode(BT_RXD_PIN, INPUT_PULLUP);
  pinMode(BT_TXD_PIN, OUTPUT);
  // Ready arduino
  Serial.begin(9600);

  // Ready HC-06
  BT.begin(9600);
}

void loop() {
  float humidity,temperature;

  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
  if(measure_environment(&temperature, &humidity))
  {
    
    // Check serial for availabilty
    if(Serial.available())
    {
      // Format Temperature string and send its C value
      String tempStr = "T=" + String(temperature);
      Serial.println(tempStr.c_str());

      // Format Humidity string and send its C value
      String humidStr = "H=" + String(humidity);
      Serial.println(humidStr.c_str());
    }
  }
}

/*
 * Poll for a measurement, keeping the state machine alive.  Returns
 * true if a measurement is available.
 */
static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  // Measure once every four seconds.
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}
