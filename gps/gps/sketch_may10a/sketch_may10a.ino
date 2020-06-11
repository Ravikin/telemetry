#include <NMEA_data.h>
#include <Adafruit_PMTK.h>
#include <Adafruit_GPS.h>

// this sketch will allow you to bypass the Atmega chip
// and connect the Ultimate GPS directly to the USB/Serial
// chip converter.
     
// Connect VIN to +5V
// Connect GND to Ground
// Connect GPS RX (data into GPS) to Digital 0
// Connect GPS TX (data out from GPS) to Digital 1

void setup() {
  // You can adjust which sentences to have the module emit, below
     
// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
// uncomment this line to turn on only the "minimum recommended" data for high update rates!
//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
// uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
     
// Set the update rate
// 1 Hz update rate
//GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
// 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
// 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
//GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
}
void loop() {}
