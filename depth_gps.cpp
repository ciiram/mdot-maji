#include "select_program.h"

#if PROGRAM == DEPTH_GPS

#include "mDot.h"
#include "ChannelPlans.h"
#include "MTSText.h"
#include "GPS.h"
#include "device_addresses.h"
#include "utils.h"


#define     SLEEP_S          30 * 60

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

// sub band 1 - 8 will allow the radio to use the 8 channels in that sub band
static uint8_t frequency_sub_band = 1;

// deep sleep
static bool deep_sleep = false;

// retries 0 to disable acks
static uint8_t ack = 0;

// Parameters for ABP
std::vector<uint8_t> network_address_vector(network_address, network_address + 4);
std::vector<uint8_t> network_session_key_vector(network_session_key, network_session_key + 16);
std::vector<uint8_t> data_session_key_vector(data_session_key, data_session_key + 16);



static AnalogIn range_sensor(A0); // water level sensor
static GPS gps(PA_2, PA_3);
const uint8_t DATA_RATE = 0;
const uint32_t TX_POWER = 14;




int main () {

  plan = new lora::ChannelPlan_EU868();
  assert(plan);

  mDot* dot = mDot::getInstance(plan);
  assert(dot);

  setup_connection(ack, frequency_sub_band);

  setup_abp_parameters(network_address_vector,
                     network_session_key_vector,
                     data_session_key_vector);


  // Tx power
  dot->setTxPower(TX_POWER);

  // Data rate
  dot->setTxDataRate(DATA_RATE);

  while (1) {
    //measure water level
    uint16_t height;
    height = range_sensor.read_u16();

    printf("\rLevel: %d\n", height);

    //get gps
    if (gps.sample() == 1) {
      printf("\rLongitude: %.2f\t Latitude: %.2f\n",
             gps.longitude,
             gps.latitude);
    }

    std::vector<uint8_t> tx_data;
    tx_data.push_back((height >> 8) & 0xFF);
    tx_data.push_back(height & 0xFF);

    int32_t ret = dot->send(tx_data);
    if (ret != mDot::MDOT_OK) {
      printf("\r\nData transmission failed\n\r");
      // return -1;
    }

    // sleep
    dot->sleep(SLEEP_S, mDot::RTC_ALARM, deep_sleep);
  }

  return 0;
}


#endif
