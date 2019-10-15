#ifndef __UTILS_H__
#define __UTILS_H__

#include "mbed.h"
#include "mDot.h"
#include "ChannelPlans.h"
#include "MTSLog.h"
#include "MTSText.h"



extern mDot* dot;


int setup_connection(uint8_t ack, uint8_t frequency_sub_band);

int setup_abp_parameters(const std::vector<uint8_t>& network_address_vector,
                         const std::vector<uint8_t>& network_session_key_vector,
                         const std::vector<uint8_t>& data_session_key_vector);


#endif
