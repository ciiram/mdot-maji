#include "utils.h"



int setup_connection(uint8_t ack, uint8_t frequency_sub_band) {
  if (dot->getJoinMode() != mDot::MANUAL) {
    printf("\r\nChanging Join Mode to Manual\n\r");
    if (dot->setJoinMode(mDot::MANUAL) != mDot::MDOT_OK) {
      printf("\r\nChanging join mode failed!\n\r");
      return -1;
    }
  }
  // Set frequency subband
  if (dot->getFrequencySubBand() != frequency_sub_band) {
    if (dot->setFrequencySubBand(frequency_sub_band) != mDot::MDOT_OK) {
        printf("\r\nSetting frequency subband failed\n\r");
        return -1;
      }
  }

  // disable retries
  if (dot->setAck(ack) != mDot::MDOT_OK) {
    printf("\r\nSetting retries failed\n\r");
    return -1;
  }

  return 0;
}



int setup_abp_parameters(const std::vector<uint8_t>& network_address_vector,
                         const std::vector<uint8_t>& network_session_key_vector,
                         const std::vector<uint8_t>& data_session_key_vector) {
  // Set ABP parameters
  if (dot->setNetworkAddress(network_address_vector) != mDot::MDOT_OK) {
    printf("\r\nSetting device address failed\n\r");
    return -1;
  }

  if (dot->setNetworkSessionKey(network_session_key_vector) != mDot::MDOT_OK) {
    printf("\r\nSetting network session key failed\n\r");
    return -1;
  }

  if (dot->setDataSessionKey(data_session_key_vector) != mDot::MDOT_OK) {
    printf("\r\nSetting application session key failed\n\r");
    return -1;
  }

  return 0;
}
