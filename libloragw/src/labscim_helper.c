#include "labscim_socket.h"
#include "labscim_helper.h"

extern buffer_circ_t* gNodeOutputBuffer;
void* lgw_wait_for_command(uint32_t command, uint32_t sequence_number);

uint64_t LabscimSignalRegister(uint8_t* signal_name)
{
	uint64_t ret = 0;
	struct labscim_protocol_header* resp;
	uint32_t sequence_number = signal_register(gNodeOutputBuffer, signal_name);
	do{
		resp =  (struct labscim_protocol_header*)lgw_wait_for_command(0, sequence_number);
		if(resp->request_sequence_number == sequence_number)
		{
			ret =  ((struct labscim_signal_register_response*)resp)->signal_id;
			free(resp);
			break;
		}
		else
		{
			//should never happen
		}		
	}while(1); //ugly?
	return ret;
}

void LabscimSignalEmitDouble(uint64_t id, double value)
{
	signal_emit_double(gNodeOutputBuffer, id, value);
}

void LabscimSignalEmitChar(uint64_t id, char* value, uint64_t size)
{
	signal_emit_char(gNodeOutputBuffer, id, value,size);
}

void LabscimSignalSubscribe(uint64_t id)
{
	signal_subscribe(gNodeOutputBuffer,id);	
}