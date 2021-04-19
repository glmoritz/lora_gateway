#ifndef __LABSCIM_PROTOCOL_SOCKET_H
#define __LABSCIM_PROTOCOL_SOCKET_H

#include "labscim_linked_list.h"
#include "labscim_protocol.h"
#include "labscim_socket.h"

extern struct labscim_ll gCommands;
extern buffer_circ_t* gNodeInputBuffer;
extern buffer_circ_t* gNodeOutputBuffer;
extern uint8_t* gNodeName;
extern uint8_t* gServerAddress;
extern uint64_t gServerPort;
extern uint64_t gIsMaster;
extern uint64_t gBufferSize;
extern uint32_t gBootReceived;
extern uint32_t gProcessing;


int socket_process_command(struct labscim_protocol_header *hdr);
void socket_process_all_commands();
void platform_process_args(int argc, char**argv);
void* lgw_pop_command(uint32_t command, uint32_t sequence_number);
void* lgw_wait_for_command(uint32_t command, uint32_t sequence_number);


#endif //__LABSCIM_PROTOCOL_SOCKET_H