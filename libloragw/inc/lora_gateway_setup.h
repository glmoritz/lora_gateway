#ifndef LABSCIM_LORAGATEWAY_SETUP_H_
#define LABSCIM_LORAGATEWAY_SETUP_H_

#include <stdint.h>

struct lora_gateway_setup
{
    uint8_t mac_addr[8];
    uint64_t startup_time;
    uint64_t labscim_log_master;
    uint8_t output_logs;
}__attribute__((packed));


#endif /* LABSCIM_LORAGATEWAY_SETUP_H_ */
