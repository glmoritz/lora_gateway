#!/bin/bash
rm ./libloragw/src/shared_mutex.c
rm ./libloragw/src/labscim_socket.c
rm ./libloragw/src/parson.c
rm ./libloragw/src/labscim_linked_list.c
rm ./libloragw/src/base64.c
rm ./libloragw/inc/lr_fhss_mac.h
rm ./libloragw/inc/parson.h
rm ./libloragw/inc/shared_mutex.h
rm ./libloragw/inc/labscim_linked_list.h
rm ./libloragw/inc/labscim_protocol.h
rm ./libloragw/inc/labscim_socket.h
rm ./libloragw/inc/sx126x_labscim.h
rm ./libloragw/inc/labscim_sx126x.h
rm ./libloragw/inc/labscim-lora-radio-protocol.h
rm ./libloragw/inc/labscim_sx126x_lr_fhss.h
rm ./libloragw/inc/base64.h
rm ./libloragw/inc/lr_fhss_v1_base_types.h
rm ./libloragw/inc/omnet_radio_mode.h
ln -s $HOME/packet_forwarder/lora_pkt_fwd/inc/base64.h ./libloragw/inc/base64.h
ln -s $HOME/contiki-ng/arch/platform/labscim/labscim_linked_list.h ./libloragw/inc/labscim_linked_list.h
ln -s $HOME/LoRaMac-node/src/radio/labscim-lora-radio-protocol.h ./libloragw/inc/labscim-lora-radio-protocol.h
ln -s $HOME/contiki-ng/arch/platform/labscim/labscim_protocol.h ./libloragw/inc/labscim_protocol.h
ln -s $HOME/contiki-ng/arch/platform/labscim/labscim_socket.h ./libloragw/inc/labscim_socket.h
ln -s $HOME/LoRaMac-node/src/radio/labscim_sx126x_driver/src/labscim_sx126x_lr_fhss.h ./libloragw/inc/lr_fhss_mac.h
ln -s $HOME/LoRaMac-node/src/radio/labscim_sx126x_driver/src/lr_fhss_mac.h ./libloragw/inc/lr_fhss_mac.h
ln -s $HOME/LoRaMac-node/src/radio/labscim_sx126x_driver/src/lr_fhss_v1_base_types.h ./libloragw/inc/lr_fhss_v1_base_types.h
ln -s $HOME/labscim/src/common/omnet_radio_mode.h ./libloragw/inc/omnet_radio_mode.h
ln -s $HOME/packet_forwarder/lora_pkt_fwd/inc/parson.h ./libloragw/inc/parson.h
ln -s $HOME/contiki-ng/arch/platform/labscim/shared_mutex.h ./libloragw/inc/shared_mutex.h
ln -s $HOME/LoRaMac-node/src/radio/labscim_sx126x_driver/src/labscim_sx126x.h ./libloragw/inc/labscim_sx126x.h
ln -s $HOME/packet_forwarder/lora_pkt_fwd/src/base64.c ./libloragw/src/base64.c
ln -s $HOME/contiki-ng/arch/platform/labscim/labscim_linked_list.c ./libloragw/src/labscim_linked_list.c
ln -s $HOME/contiki-ng/arch/platform/labscim/labscim_socket.c ./libloragw/src/labscim_socket.c
ln -s $HOME/packet_forwarder/lora_pkt_fwd/src/parson.c ./libloragw/src/parson.c
ln -s $HOME/contiki-ng/arch/platform/labscim/shared_mutex.c ./libloragw/src/shared_mutex.c
