//labscim related structures
#include <stdarg.h>
#include <sys/time.h>
#include "labscim_linked_list.h"
#include "labscim_protocol.h"
#include "labscim_socket.h"
#include "lora_gateway_setup.h"

#define SERVER_PORT (9608)
#define SERVER_ADDRESS "127.0.0.1"
#define SOCK_BUFFER_SIZE (512)

struct labscim_ll gCommands;

buffer_circ_t *gNodeInputBuffer;
buffer_circ_t *gNodeOutputBuffer;
uint8_t* gNodeName;
uint8_t* gServerAddress;
uint64_t gCommandLabscimLog;
uint64_t gServerPort;
uint64_t gBufferSize;
uint32_t gBootReceived=0;
uint32_t gProcessing=0;
uint8_t gMQTTAddress[48];
uint8_t gMQTTTopic[128];

double gGPSLatitude_deg=0;
double gGPSLongitude_deg=0;
double gGPSAltitude_m=0;

struct timeval gTVStart = {0,0};

uint8_t mac_addr[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

extern pthread_mutex_t gWaitingCommandsMutex;
extern pthread_cond_t gWaitingCommandsMore;
extern struct labscim_ll gThreadCommands;

#define DBG_PRINT_BUFFER_SIZE (256)
uint8_t gByteBuffer[DBG_PRINT_BUFFER_SIZE];

void labscim_set_time(uint64_t time);
void labscim_signal_arrived(struct labscim_signal* sig);


void labscim_protocol_boot(struct labscim_protocol_boot* msg)
{
	struct lora_gateway_setup* cns = (struct lora_gateway_setup*)msg->message;
	memcpy((void*)mac_addr,(void*)cns->mac_addr,sizeof(uint8_t)*8);
	labscim_set_time(cns->startup_time);
	gBootReceived = 1;
    gCommandLabscimLog = cns->labscim_log_master;
    gGPSAltitude_m = cns->alt_m;
    gGPSLatitude_deg = cns->lat_deg;
    gGPSLongitude_deg = cns->lon_deg;

    gTVStart.tv_sec = cns->TimeReference / 1000000;
    gTVStart.tv_usec = cns->TimeReference % 1000000;

    strcpy(gMQTTAddress,cns->MQTTLoggerAddress);
    strcpy(gMQTTTopic,cns->MQTTLoggerApplicationTopic);
	free(msg);
	return;
}


int socket_process_command(struct labscim_protocol_header *hdr)
{
#ifdef LABSCIM_LOG_COMMANDS
    char log[128];
#endif
    switch (hdr->labscim_protocol_code)
    {
    case LABSCIM_PROTOCOL_BOOT:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tPROTOCOL_BOOT\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        labscim_protocol_boot((struct labscim_protocol_boot *)(hdr));
        break;
    }    
    case LABSCIM_TIME_EVENT:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tTIME_EVENT\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif        
        labscim_set_time(((struct labscim_time_event *)(hdr))->current_time_us);
        labscim_ll_insert_at_back(&gThreadCommands,(void*)hdr);  
        break;
    }
    case LABSCIM_SIGNAL_REGISTER_RESPONSE:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tSIGNAL_REGISTER_RESPONSE\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif        
        labscim_ll_insert_at_back(&gThreadCommands, (void *)hdr);
        break;
    }
    case LABSCIM_RADIO_RESPONSE:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tRADIO_RESPONSE\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        labscim_set_time(((struct labscim_radio_response *)(hdr))->current_time);
        labscim_radio_incoming_response((struct labscim_radio_response *)(hdr));
        break;
    }
    case LABSCIM_SIGNAL:
    {
        labscim_set_time(((struct labscim_signal *)(hdr))->current_time);
        labscim_signal_arrived((struct labsim_signal *)(hdr));
        break;
    }
    case LABSCIM_END:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tEND\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        exit(0);
        break;
    }
    default:
    {
        perror("Unhandled Labscim Command\n");
        free(hdr);
    }
    }
    return 1;
}

void* lgw_process_all_commands()
{
    pthread_mutex_lock(&gWaitingCommandsMutex); 
    uint64_t count = gThreadCommands.count;   
    for(uint64_t i=0;i<count;i++)
    {
        struct labscim_protocol_header *hdr = labscim_ll_pop_front(&gThreadCommands);
        if (!socket_process_command(hdr))
        {
            labscim_ll_insert_at_back(&gThreadCommands,(void*)hdr);
        }       
    } 
    pthread_mutex_unlock(&gWaitingCommandsMutex);     
}


void *lgw_pop_command(uint32_t command, uint32_t sequence_number)
{
    void *cmd=NULL;
    pthread_mutex_lock(&gWaitingCommandsMutex); 
    struct labscim_ll_node *it = gThreadCommands.head;
    do
    {
        if (it != NULL)
        {
            struct labscim_protocol_header *hdr = (struct labscim_protocol_header *)it->data;
            if (((hdr->labscim_protocol_code == command) && (sequence_number == 0)) || (hdr->request_sequence_number == sequence_number) || ((command == 0) && (sequence_number == 0)))
            {
                cmd = labscim_ll_pop_node(&gThreadCommands, it);
                break;
            }
            it = it->next;
        }

    } while (it != NULL);
    pthread_mutex_unlock(&gWaitingCommandsMutex); 
    return cmd;
}

void* lgw_wait_for_command(uint32_t command, uint32_t sequence_number)
{
    struct labscim_protocol_header* resp;    
    uint8_t waiting = 1;
    void* cmd;
    uint32_t cmd_seq;    
    pthread_mutex_lock(&gWaitingCommandsMutex);                        
    while (waiting)
    {        
        if (gThreadCommands.count > 0)
        {
            struct labscim_ll_node *iter = gThreadCommands.head;
            while (iter != NULL)
            {
                if (((struct labscim_protocol_header *)iter->data)->request_sequence_number == sequence_number)
                {
                    cmd =  labscim_ll_pop_node(&gThreadCommands, iter);                                                           
                    waiting = 0;
                    break;
                }
                iter = iter->next;
            }
        }
        if(waiting)
        {
            pthread_cond_wait(&gWaitingCommandsMore, &gWaitingCommandsMutex);
        }
    }
    pthread_mutex_unlock(&gWaitingCommandsMutex);
    return cmd;
}

