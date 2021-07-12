/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    LoRa concentrator Hardware Abstraction Layer

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>  /* C99 types */
#include <stdbool.h> /* bool type */
#include <stdio.h>   /* printf fprintf */
#include <string.h>  /* memcpy */

#include <sys/time.h>
#include <math.h>    /* pow, cell */
#include <stdarg.h>

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_spi.h"
#include "loragw_radio.h"
#include "loragw_fpga.h"
#include "loragw_lbt.h"
#include "sx126x_labscim.h"

/*----------------
LABSCIM
-----------------*/
#include "labscim_linked_list.h"
#include "labscim_protocol.h"
#include "labscim_socket.h"
#include "labscim_platform_socket.h"
#include "labscim-lora-radio-protocol.h"
#include "base64.h"
#include "parson.h"
#include "labscim_helper.h"

extern uint64_t gLabscimTime;
extern uint64_t gCommandLabscimLog;
extern uint8_t mac_addr[];

/*LabSCim MQTT result collector*/
#include "MQTTAsync.h"

void mqtt_onSubscribeFailure(void* context, MQTTAsync_failureData* response);
void mqtt_onSubscribe(void* context, MQTTAsync_successData* response);
void mqtt_onDisconnect(void* context, MQTTAsync_successData* response);
void mqtt_onConnectFailure(void* context, MQTTAsync_failureData* response);
void mqtt_connlost(void *context, char *cause);
int mqtt_msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message);
void mqtt_onSend(void* context, MQTTAsync_successData* response);
void mqtt_onConnect(void* context, MQTTAsync_successData* response);



extern uint8_t gMQTTAddress[48];
extern uint8_t gMQTTTopic[128];

#define CLIENTID    "LabSCim gateway"
#define QOS         1
#define TIMEOUT     10000L

uint64_t gPacketGeneratedSignal;
uint64_t gPacketLatencySignal;
uint64_t gPacketErrorSignal;

pthread_mutex_t gEmitListMutex = PTHREAD_MUTEX_INITIALIZER;
struct labscim_ll gEmitSignalList;




volatile bool lib_exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool lib_quit_sig = false;


/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_HAL == 1
#define DEBUG_MSG(str) fprintf(stderr, str)
#define DEBUG_PRINTF(fmt, args...) fprintf(stderr, "%s:%d: " fmt, __FUNCTION__, __LINE__, args)
#define DEBUG_ARRAY(a, b, c)          \
    for (a = 0; a < b; ++a)           \
        fprintf(stderr, "%x.", c[a]); \
    fprintf(stderr, "end\n")
#define CHECK_NULL(a)                                                                        \
    if (a == NULL)                                                                           \
    {                                                                                        \
        fprintf(stderr, "%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__); \
        return LGW_HAL_ERROR;                                                                \
    }
#else
#define DEBUG_MSG(str)
#define DEBUG_PRINTF(fmt, args...)
#define DEBUG_ARRAY(a, b, c) \
    for (a = 0; a != 0;)     \
    {                        \
    }
#define CHECK_NULL(a)         \
    if (a == NULL)            \
    {                         \
        return LGW_HAL_ERROR; \
    }
#endif

#define IF_HZ_TO_REG(f) (f << 5) / 15625
#define SET_PPM_ON(bw, dr) (((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))
#define TRACE() fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

#define MCU_ARB 0
#define MCU_AGC 1
#define MCU_ARB_FW_BYTE 8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define MCU_AGC_FW_BYTE 8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define FW_VERSION_ADDR 0x20 /* Address of firmware version in data memory */
#define FW_VERSION_CAL 2     /* Expected version of calibration firmware */
#define FW_VERSION_AGC 4     /* Expected version of AGC firmware */
#define FW_VERSION_ARB 1     /* Expected version of arbiter firmware */

#define TX_METADATA_NB 16
#define RX_METADATA_NB 16

#define AGC_CMD_WAIT 16
#define AGC_CMD_ABORT 17

#define MIN_LORA_PREAMBLE 6
#define STD_LORA_PREAMBLE 8
#define MIN_FSK_PREAMBLE 3
#define STD_FSK_PREAMBLE 5

#define RSSI_MULTI_BIAS -35 /* difference between "multi" modem RSSI offset and "stand-alone" modem RSSI offset */
#define RSSI_FSK_POLY_0 60  /* polynomiam coefficients to linearize FSK RSSI */
#define RSSI_FSK_POLY_1 1.5351
#define RSSI_FSK_POLY_2 0.003

/* Useful bandwidth of SX125x radios to consider depending on channel bandwidth */
/* Note: the below values come from lab measurements. For any question, please contact Semtech support */
#define LGW_RF_RX_BANDWIDTH_125KHZ 925000  /* for 125KHz channels */
#define LGW_RF_RX_BANDWIDTH_250KHZ 1000000 /* for 250KHz channels */
#define LGW_RF_RX_BANDWIDTH_500KHZ 1100000 /* for 500KHz channels */

#define TX_START_DELAY_DEFAULT 1497 /* Calibrated value for 500KHz BW and notch filter disabled */

/* constant arrays defining hardware capability */
const uint8_t ifmod_config[LGW_IF_CHAIN_NB] = LGW_IFMODEM_CONFIG;

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "arb_fw.var" /* external definition of the variable */
#include "agc_fw.var" /* external definition of the variable */
#include "cal_fw.var" /* external definition of the variable */

/*
The following static variables are the configuration set that the user can
modify using rxrf_setconf, rxif_setconf and txgain_setconf functions.
The functions _start and _send then use that set to configure the hardware.

Parameters validity and coherency is verified by the _setconf functions and
the _start and _send functions assume they are valid.
*/

static bool lgw_is_started;

static bool rf_enable[LGW_RF_CHAIN_NB];
static uint32_t rf_rx_freq[LGW_RF_CHAIN_NB]; /* absolute, in Hz */
static float rf_rssi_offset[LGW_RF_CHAIN_NB];
static bool rf_tx_enable[LGW_RF_CHAIN_NB];
static uint32_t rf_tx_notch_freq[LGW_RF_CHAIN_NB];
static enum lgw_radio_type_e rf_radio_type[LGW_RF_CHAIN_NB];

static bool if_enable[LGW_IF_CHAIN_NB];
static bool if_rf_chain[LGW_IF_CHAIN_NB]; /* for each IF, 0 -> radio A, 1 -> radio B */
static int32_t if_freq[LGW_IF_CHAIN_NB];  /* relative to radio frequency, +/- in Hz */

static uint8_t lora_multi_sfmask[LGW_MULTI_NB]; /* enables SF for LoRa 'multi' modems */

static uint8_t lora_rx_bw; /* bandwidth setting for LoRa standalone modem */
static uint8_t lora_rx_sf; /* spreading factor setting for LoRa standalone modem */
static bool lora_rx_ppm_offset;

static uint8_t fsk_rx_bw;                 /* bandwidth setting of FSK modem */
static uint32_t fsk_rx_dr;                /* FSK modem datarate in bauds */
static uint8_t fsk_sync_word_size = 3;    /* default number of bytes for FSK sync word */
static uint64_t fsk_sync_word = 0xC194C1; /* default FSK sync word (ALIGNED RIGHT, MSbit first) */

static bool lorawan_public = false;
static uint8_t rf_clkout = 0;

static struct lgw_tx_gain_lut_s txgain_lut = {
    .size = 2,
    .lut[0] = {
        .dig_gain = 0,
        .pa_gain = 2,
        .dac_gain = 3,
        .mix_gain = 10,
        .rf_power = 14},
    .lut[1] = {.dig_gain = 0, .pa_gain = 3, .dac_gain = 3, .mix_gain = 14, .rf_power = 27}};

/* TX I/Q imbalance coefficients for mixer gain = 8 to 15 */
static int8_t cal_offset_a_i[8]; /* TX I offset for radio A */
static int8_t cal_offset_a_q[8]; /* TX Q offset for radio A */
static int8_t cal_offset_b_i[8]; /* TX I offset for radio B */
static int8_t cal_offset_b_q[8]; /* TX Q offset for radio B */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int load_firmware(uint8_t target, uint8_t *firmware, uint16_t size);

void lgw_constant_adjust(void);

int32_t lgw_sf_getval(int x);
int32_t lgw_bw_getval(int x);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

void EmitAndYield()
{
    struct signal_emit* em;
    pthread_mutex_lock(&gEmitListMutex);
    em = labscim_ll_pop_front(&gEmitSignalList);
    while(em!=NULL)
    {
        LabscimSignalEmit(em->id,em->value);
        free(em);
        em = labscim_ll_pop_front(&gEmitSignalList);
    }
    pthread_mutex_unlock(&gEmitListMutex);
    protocol_yield(gNodeOutputBuffer);
}


/* size is the firmware size in bytes (not 14b words) */
int load_firmware(uint8_t target, uint8_t *firmware, uint16_t size)
{
    int reg_rst;
    int reg_sel;
    uint8_t fw_check[8192];
    int32_t dummy;

    /* check parameters */
    CHECK_NULL(firmware);
    if (target == MCU_ARB)
    {
        if (size != MCU_ARB_FW_BYTE)
        {
            DEBUG_MSG("ERROR: NOT A VALID SIZE FOR MCU ARG FIRMWARE\n");
            return -1;
        }
        reg_rst = LGW_MCU_RST_0;
        reg_sel = LGW_MCU_SELECT_MUX_0;
    }
    else if (target == MCU_AGC)
    {
        if (size != MCU_AGC_FW_BYTE)
        {
            DEBUG_MSG("ERROR: NOT A VALID SIZE FOR MCU AGC FIRMWARE\n");
            return -1;
        }
        reg_rst = LGW_MCU_RST_1;
        reg_sel = LGW_MCU_SELECT_MUX_1;
    }
    else
    {
        DEBUG_MSG("ERROR: NOT A VALID TARGET FOR LOADING FIRMWARE\n");
        return -1;
    }

    /* reset the targeted MCU */
    lgw_reg_w(reg_rst, 1);

    /* set mux to access MCU program RAM and set address to 0 */
    lgw_reg_w(reg_sel, 0);
    lgw_reg_w(LGW_MCU_PROM_ADDR, 0);

    /* write the program in one burst */
    lgw_reg_wb(LGW_MCU_PROM_DATA, firmware, size);

    /* Read back firmware code for check */
    lgw_reg_r(LGW_MCU_PROM_DATA, &dummy); /* bug workaround */
    lgw_reg_rb(LGW_MCU_PROM_DATA, fw_check, size);
    if (memcmp(firmware, fw_check, size) != 0)
    {
        printf("ERROR: Failed to load fw %d\n", (int)target);
        return -1;
    }

    /* give back control of the MCU program ram to the MCU */
    lgw_reg_w(reg_sel, 1);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lgw_constant_adjust(void)
{

    /* I/Q path setup */
    // lgw_reg_w(LGW_RX_INVERT_IQ,0); /* default 0 */
    // lgw_reg_w(LGW_MODEM_INVERT_IQ,1); /* default 1 */
    // lgw_reg_w(LGW_CHIRP_INVERT_RX,1); /* default 1 */
    // lgw_reg_w(LGW_RX_EDGE_SELECT,0); /* default 0 */
    // lgw_reg_w(LGW_MBWSSF_MODEM_INVERT_IQ,0); /* default 0 */
    // lgw_reg_w(LGW_DC_NOTCH_EN,1); /* default 1 */
    lgw_reg_w(LGW_RSSI_BB_FILTER_ALPHA, 6);      /* default 7 */
    lgw_reg_w(LGW_RSSI_DEC_FILTER_ALPHA, 7);     /* default 5 */
    lgw_reg_w(LGW_RSSI_CHANN_FILTER_ALPHA, 7);   /* default 8 */
    lgw_reg_w(LGW_RSSI_BB_DEFAULT_VALUE, 23);    /* default 32 */
    lgw_reg_w(LGW_RSSI_CHANN_DEFAULT_VALUE, 85); /* default 100 */
    lgw_reg_w(LGW_RSSI_DEC_DEFAULT_VALUE, 66);   /* default 100 */
    lgw_reg_w(LGW_DEC_GAIN_OFFSET, 7);           /* default 8 */
    lgw_reg_w(LGW_CHAN_GAIN_OFFSET, 6);          /* default 7 */

    /* Correlator setup */
    // lgw_reg_w(LGW_CORR_DETECT_EN,126); /* default 126 */
    // lgw_reg_w(LGW_CORR_NUM_SAME_PEAK,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_MAC_GAIN,5); /* default 5 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF6,0); /* default 0 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF7,1); /* default 1 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF8,1); /* default 1 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF9,1); /* default 1 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF10,1); /* default 1 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF11,1); /* default 1 */
    // lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF12,1); /* default 1 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF6,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF7,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF8,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF9,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF10,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF11,4); /* default 4 */
    // lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF12,4); /* default 4 */

    /* LoRa 'multi' demodulators setup */
    // lgw_reg_w(LGW_PREAMBLE_SYMB1_NB,10); /* default 10 */
    // lgw_reg_w(LGW_FREQ_TO_TIME_INVERT,29); /* default 29 */
    // lgw_reg_w(LGW_FRAME_SYNCH_GAIN,1); /* default 1 */
    // lgw_reg_w(LGW_SYNCH_DETECT_TH,1); /* default 1 */
    // lgw_reg_w(LGW_ZERO_PAD,0); /* default 0 */
    lgw_reg_w(LGW_SNR_AVG_CST, 3); /* default 2 */
    if (lorawan_public)
    {                                            /* LoRa network */
        lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS, 3); /* default 1 */
        lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS, 4); /* default 2 */
    }
    else
    {                                            /* private network */
        lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS, 1); /* default 1 */
        lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS, 2); /* default 2 */
    }

    // lgw_reg_w(LGW_PREAMBLE_FINE_TIMING_GAIN,1); /* default 1 */
    // lgw_reg_w(LGW_ONLY_CRC_EN,1); /* default 1 */
    // lgw_reg_w(LGW_PAYLOAD_FINE_TIMING_GAIN,2); /* default 2 */
    // lgw_reg_w(LGW_TRACKING_INTEGRAL,0); /* default 0 */
    // lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_RDX8,0); /* default 0 */
    // lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_SF12_RDX4,4092); /* default 4092 */
    // lgw_reg_w(LGW_MAX_PAYLOAD_LEN,255); /* default 255 */

    /* LoRa standalone 'MBWSSF' demodulator setup */
    // lgw_reg_w(LGW_MBWSSF_PREAMBLE_SYMB1_NB,10); /* default 10 */
    // lgw_reg_w(LGW_MBWSSF_FREQ_TO_TIME_INVERT,29); /* default 29 */
    // lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_GAIN,1); /* default 1 */
    // lgw_reg_w(LGW_MBWSSF_SYNCH_DETECT_TH,1); /* default 1 */
    // lgw_reg_w(LGW_MBWSSF_ZERO_PAD,0); /* default 0 */
    if (lorawan_public)
    {                                                   /* LoRa network */
        lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS, 3); /* default 1 */
        lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS, 4); /* default 2 */
    }
    else
    {
        lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS, 1); /* default 1 */
        lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS, 2); /* default 2 */
    }
    // lgw_reg_w(LGW_MBWSSF_ONLY_CRC_EN,1); /* default 1 */
    // lgw_reg_w(LGW_MBWSSF_PAYLOAD_FINE_TIMING_GAIN,2); /* default 2 */
    // lgw_reg_w(LGW_MBWSSF_PREAMBLE_FINE_TIMING_GAIN,1); /* default 1 */
    // lgw_reg_w(LGW_MBWSSF_TRACKING_INTEGRAL,0); /* default 0 */
    // lgw_reg_w(LGW_MBWSSF_AGC_FREEZE_ON_DETECT,1); /* default 1 */

    /* Improvement of reference clock frequency error tolerance */
    lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_RDX4, 1);         /* default 0 */
    lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_SF12_RDX4, 4094); /* default 4092 */
    lgw_reg_w(LGW_CORR_MAC_GAIN, 7);                          /* default 5 */

    /* FSK datapath setup */
    lgw_reg_w(LGW_FSK_RX_INVERT, 1);       /* default 0 */
    lgw_reg_w(LGW_FSK_MODEM_INVERT_IQ, 1); /* default 0 */

    /* FSK demodulator setup */
    lgw_reg_w(LGW_FSK_RSSI_LENGTH, 4); /* default 0 */
    lgw_reg_w(LGW_FSK_PKT_MODE, 1);    /* variable length, default 0 */
    lgw_reg_w(LGW_FSK_CRC_EN, 1);      /* default 0 */
    lgw_reg_w(LGW_FSK_DCFREE_ENC, 2);  /* default 0 */
    // lgw_reg_w(LGW_FSK_CRC_IBM,0); /* default 0 */
    lgw_reg_w(LGW_FSK_ERROR_OSR_TOL, 10); /* default 0 */
    lgw_reg_w(LGW_FSK_PKT_LENGTH, 255);   /* max packet length in variable length mode */
    // lgw_reg_w(LGW_FSK_NODE_ADRS,0); /* default 0 */
    // lgw_reg_w(LGW_FSK_BROADCAST,0); /* default 0 */
    // lgw_reg_w(LGW_FSK_AUTO_AFC_ON,0); /* default 0 */
    lgw_reg_w(LGW_FSK_PATTERN_TIMEOUT_CFG, 128); /* sync timeout (allow 8 bytes preamble + 8 bytes sync word, default 0 */

    /* TX general parameters */
    lgw_reg_w(LGW_TX_START_DELAY, TX_START_DELAY_DEFAULT); /* default 0 */

    /* TX LoRa */
    // lgw_reg_w(LGW_TX_MODE,0); /* default 0 */
    lgw_reg_w(LGW_TX_SWAP_IQ, 1); /* "normal" polarity; default 0 */
    if (lorawan_public)
    {                                               /* LoRa network */
        lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK1_POS, 3); /* default 1 */
        lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK2_POS, 4); /* default 2 */
    }
    else
    {                                               /* Private network */
        lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK1_POS, 1); /* default 1 */
        lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK2_POS, 2); /* default 2 */
    }

    /* TX FSK */
    // lgw_reg_w(LGW_FSK_TX_GAUSSIAN_EN,1); /* default 1 */
    lgw_reg_w(LGW_FSK_TX_GAUSSIAN_SELECT_BT, 2); /* Gaussian filter always on TX, default 0 */
    // lgw_reg_w(LGW_FSK_TX_PATTERN_EN,1); /* default 1 */
    // lgw_reg_w(LGW_FSK_TX_PREAMBLE_SEQ,0); /* default 0 */

    return;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t lgw_bw_getval(int x)
{
    switch (x)
    {
    case BW_500KHZ:
        return 500000;
    case BW_250KHZ:
        return 250000;
    case BW_125KHZ:
        return 125000;
    case BW_62K5HZ:
        return 62500;
    case BW_31K2HZ:
        return 31200;
    case BW_15K6HZ:
        return 15600;
    case BW_7K8HZ:
        return 7800;
    default:
        return -1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t lgw_sf_getval(int x)
{
    switch (x)
    {
    case DR_LORA_SF7:
        return 7;
    case DR_LORA_SF8:
        return 8;
    case DR_LORA_SF9:
        return 9;
    case DR_LORA_SF10:
        return 10;
    case DR_LORA_SF11:
        return 11;
    case DR_LORA_SF12:
        return 12;
    default:
        return -1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t lgw_get_tx_start_delay(bool tx_notch_enable, uint8_t bw)
{
    float notch_delay_us = 0.0;
    float bw_delay_us = 0.0;
    float tx_start_delay;

    /* Notch filtering performed by FPGA adds a constant delay (group delay) that we need to compensate */
    if (tx_notch_enable)
    {
        notch_delay_us = lgw_fpga_get_tx_notch_delay();
    }

    /* Calibrated delay brought by SX1301 depending on signal bandwidth */
    switch (bw)
    {
    case BW_125KHZ:
        bw_delay_us = 1.5;
        break;
    case BW_500KHZ:
        /* Intended fall-through: it is the calibrated reference */
    default:
        break;
    }

    tx_start_delay = (float)TX_START_DELAY_DEFAULT - bw_delay_us - notch_delay_us;

    printf("INFO: tx_start_delay=%u (%f) - (%u, bw_delay=%f, notch_delay=%f)\n", (uint16_t)tx_start_delay, tx_start_delay, TX_START_DELAY_DEFAULT, bw_delay_us, notch_delay_us);

    return (uint16_t)tx_start_delay; /* keep truncating instead of rounding: better behaviour measured */
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_board_setconf(struct lgw_conf_board_s conf)
{

    /* check if the concentrator is running */
    if (lgw_is_started == true)
    {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    lorawan_public = conf.lorawan_public;
    rf_clkout = conf.clksrc;

    DEBUG_PRINTF("Note: board configuration; lorawan_public:%d, clksrc:%d\n", lorawan_public, rf_clkout);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_lbt_setconf(struct lgw_conf_lbt_s conf)
{
    int x;

    /* check if the concentrator is running */
    if (lgw_is_started == true)
    {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    x = lbt_setconf(&conf);
    if (x != LGW_LBT_SUCCESS)
    {
        DEBUG_MSG("ERROR: Failed to configure concentrator for LBT\n");
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf)
{

    /* check if the concentrator is running */
    if (lgw_is_started == true)
    {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* check input range (segfault prevention) */
    if (rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* check if radio type is supported */
    if ((conf.type != LGW_RADIO_TYPE_SX1255) && (conf.type != LGW_RADIO_TYPE_SX1257))
    {
        DEBUG_MSG("ERROR: NOT A VALID RADIO TYPE\n");
        return LGW_HAL_ERROR;
    }

    /* check if TX notch filter frequency is supported */
    if ((conf.tx_enable == true) && ((conf.tx_notch_freq < LGW_MIN_NOTCH_FREQ) || (conf.tx_notch_freq > LGW_MAX_NOTCH_FREQ)))
    {
        DEBUG_PRINTF("WARNING: NOT A VALID TX NOTCH FILTER FREQUENCY [%u..%u]Hz\n", LGW_MIN_NOTCH_FREQ, LGW_MAX_NOTCH_FREQ);
        conf.tx_notch_freq = 0;
    }

    /* set internal config according to parameters */
    rf_enable[rf_chain] = conf.enable;
    rf_rx_freq[rf_chain] = conf.freq_hz;
    rf_rssi_offset[rf_chain] = conf.rssi_offset;
    rf_radio_type[rf_chain] = conf.type;
    rf_tx_enable[rf_chain] = conf.tx_enable;
    rf_tx_notch_freq[rf_chain] = conf.tx_notch_freq;

    DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d rssi_offset:%f radio_type:%d tx_enable:%d tx_notch_freq:%u\n", rf_chain, rf_enable[rf_chain], rf_rx_freq[rf_chain], rf_rssi_offset[rf_chain], rf_radio_type[rf_chain], rf_tx_enable[rf_chain], rf_tx_notch_freq[rf_chain]);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf)
{
    int32_t bw_hz;
    uint32_t rf_rx_bandwidth;

    /* check if the concentrator is running */
    if (lgw_is_started == true)
    {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* check input range (segfault prevention) */
    if (if_chain >= LGW_IF_CHAIN_NB)
    {
        DEBUG_PRINTF("ERROR: %d NOT A VALID IF_CHAIN NUMBER\n", if_chain);
        return LGW_HAL_ERROR;
    }

    /* if chain is disabled, don't care about most parameters */
    if (conf.enable == false)
    {
        if_enable[if_chain] = false;
        if_freq[if_chain] = 0;
        DEBUG_PRINTF("Note: if_chain %d disabled\n", if_chain);
        return LGW_HAL_SUCCESS;
    }

    /* check 'general' parameters */
    if (ifmod_config[if_chain] == IF_UNDEFINED)
    {
        DEBUG_PRINTF("ERROR: IF CHAIN %d NOT CONFIGURABLE\n", if_chain);
    }
    if (conf.rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN\n");
        return LGW_HAL_ERROR;
    }
    /* check if IF frequency is optimal based on channel and radio bandwidths */
    switch (conf.bandwidth)
    {
    case BW_250KHZ:
        rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_250KHZ; /* radio bandwidth */
        break;
    case BW_500KHZ:
        rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_500KHZ; /* radio bandwidth */
        break;
    default:
        /* For 125KHz and below */
        rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_125KHZ; /* radio bandwidth */
        break;
    }
    bw_hz = lgw_bw_getval(conf.bandwidth); /* channel bandwidth */
    if ((conf.freq_hz + ((bw_hz == -1) ? LGW_REF_BW : bw_hz) / 2) > ((int32_t)rf_rx_bandwidth / 2))
    {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO HIGH\n", conf.freq_hz);
        return LGW_HAL_ERROR;
    }
    else if ((conf.freq_hz - ((bw_hz == -1) ? LGW_REF_BW : bw_hz) / 2) < -((int32_t)rf_rx_bandwidth / 2))
    {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO LOW\n", conf.freq_hz);
        return LGW_HAL_ERROR;
    }

    /* check parameters according to the type of IF chain + modem,
    fill default if necessary, and commit configuration if everything is OK */
    switch (ifmod_config[if_chain])
    {
    case IF_LORA_STD:
        /* fill default parameters if needed */
        if (conf.bandwidth == BW_UNDEFINED)
        {
            conf.bandwidth = BW_250KHZ;
        }
        if (conf.datarate == DR_UNDEFINED)
        {
            conf.datarate = DR_LORA_SF9;
        }
        /* check BW & DR */
        if (!IS_LORA_BW(conf.bandwidth))
        {
            DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_STD_DR(conf.datarate))
        {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        /* set internal configuration  */
        if_enable[if_chain] = conf.enable;
        if_rf_chain[if_chain] = conf.rf_chain;
        if_freq[if_chain] = conf.freq_hz;
        lora_rx_bw = conf.bandwidth;
        lora_rx_sf = (uint8_t)(DR_LORA_MULTI & conf.datarate); /* filter SF out of the 7-12 range */
        if (SET_PPM_ON(conf.bandwidth, conf.datarate))
        {
            lora_rx_ppm_offset = true;
        }
        else
        {
            lora_rx_ppm_offset = false;
        }

        DEBUG_PRINTF("Note: LoRa 'std' if_chain %d configuration; en:%d freq:%d bw:%d dr:%d\n", if_chain, if_enable[if_chain], if_freq[if_chain], lora_rx_bw, lora_rx_sf);
        break;

    case IF_LORA_MULTI:
        /* fill default parameters if needed */
        if (conf.bandwidth == BW_UNDEFINED)
        {
            conf.bandwidth = BW_125KHZ;
        }
        if (conf.datarate == DR_UNDEFINED)
        {
            conf.datarate = DR_LORA_MULTI;
        }
        /* check BW & DR */
        if (conf.bandwidth != BW_125KHZ)
        {
            DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_MULTI_DR(conf.datarate))
        {
            DEBUG_MSG("ERROR: DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        /* set internal configuration  */
        if_enable[if_chain] = conf.enable;
        if_rf_chain[if_chain] = conf.rf_chain;
        if_freq[if_chain] = conf.freq_hz;
        lora_multi_sfmask[if_chain] = (uint8_t)(DR_LORA_MULTI & conf.datarate); /* filter SF out of the 7-12 range */

        DEBUG_PRINTF("Note: LoRa 'multi' if_chain %d configuration; en:%d freq:%d SF_mask:0x%02x\n", if_chain, if_enable[if_chain], if_freq[if_chain], lora_multi_sfmask[if_chain]);
        break;

    case IF_FSK_STD:
        /* fill default parameters if needed */
        if (conf.bandwidth == BW_UNDEFINED)
        {
            conf.bandwidth = BW_250KHZ;
        }
        if (conf.datarate == DR_UNDEFINED)
        {
            conf.datarate = 64000; /* default datarate */
        }
        /* check BW & DR */
        if (!IS_FSK_BW(conf.bandwidth))
        {
            DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_FSK_DR(conf.datarate))
        {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        /* set internal configuration  */
        if_enable[if_chain] = conf.enable;
        if_rf_chain[if_chain] = conf.rf_chain;
        if_freq[if_chain] = conf.freq_hz;
        fsk_rx_bw = conf.bandwidth;
        fsk_rx_dr = conf.datarate;
        if (conf.sync_word > 0)
        {
            fsk_sync_word_size = conf.sync_word_size;
            fsk_sync_word = conf.sync_word;
        }
        DEBUG_PRINTF("Note: FSK if_chain %d configuration; en:%d freq:%d bw:%d dr:%d (%d real dr) sync:0x%0*llX\n", if_chain, if_enable[if_chain], if_freq[if_chain], fsk_rx_bw, fsk_rx_dr, LGW_XTAL_FREQU / (LGW_XTAL_FREQU / fsk_rx_dr), 2 * fsk_sync_word_size, fsk_sync_word);
        break;

    default:
        DEBUG_PRINTF("ERROR: IF CHAIN %d TYPE NOT SUPPORTED\n", if_chain);
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_txgain_setconf(struct lgw_tx_gain_lut_s *conf)
{
    int i;

    /* Check LUT size */
    if ((conf->size < 1) || (conf->size > TX_GAIN_LUT_SIZE_MAX))
    {
        DEBUG_PRINTF("ERROR: TX gain LUT must have at least one entry and  maximum %d entries\n", TX_GAIN_LUT_SIZE_MAX);
        return LGW_HAL_ERROR;
    }

    txgain_lut.size = conf->size;

    for (i = 0; i < txgain_lut.size; i++)
    {
        /* Check gain range */
        if (conf->lut[i].dig_gain > 3)
        {
            DEBUG_MSG("ERROR: TX gain LUT: SX1301 digital gain must be between 0 and 3\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].dac_gain != 3)
        {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 DAC gains != 3 are not supported\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].mix_gain > 15)
        {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gain must not exceed 15\n");
            return LGW_HAL_ERROR;
        }
        else if (conf->lut[i].mix_gain < 8)
        {
            DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gains < 8 are not supported\n");
            return LGW_HAL_ERROR;
        }
        if (conf->lut[i].pa_gain > 3)
        {
            DEBUG_MSG("ERROR: TX gain LUT: External PA gain must not exceed 3\n");
            return LGW_HAL_ERROR;
        }

        /* Set internal LUT */
        txgain_lut.lut[i].dig_gain = conf->lut[i].dig_gain;
        txgain_lut.lut[i].dac_gain = conf->lut[i].dac_gain;
        txgain_lut.lut[i].mix_gain = conf->lut[i].mix_gain;
        txgain_lut.lut[i].pa_gain = conf->lut[i].pa_gain;
        txgain_lut.lut[i].rf_power = conf->lut[i].rf_power;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

pthread_t gThridIf;
pthread_t gThridMQTT;


pthread_mutex_t gWaitingCommandsMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t gWaitingCommandsMore = PTHREAD_COND_INITIALIZER;
struct labscim_ll gThreadCommands;

struct labscim_ll gReceivedPackets;

void lgw_labscim_get_command_queue(struct labscim_ll** Commands, pthread_mutex_t** Mutex, pthread_cond_t** More)
{
    *Commands = &gCommands;
    *Mutex = &gWaitingCommandsMutex;
    *More = &gWaitingCommandsMore;    
}



void lgw_labscim_sleep(uint64_t ms, uint64_t yield)
{
    struct labscim_protocol_header* resp;
    uint8_t ctrue = 1;
    uint8_t waiting = 1;
    uint32_t cmd_seq;
    uint32_t seq_no = set_time_event(gNodeOutputBuffer, 0, ctrue, ms * 1000);        
    if(yield)
    {
        EmitAndYield();        
    }
    pthread_mutex_lock(&gWaitingCommandsMutex);        
    while (waiting)
    {
        if (gThreadCommands.count > 0)
        {
            struct labscim_ll_node *iter = gThreadCommands.head;
            while (iter != NULL)
            {
                if (((struct labscim_protocol_header *)iter->data)->request_sequence_number == seq_no)
                {
                    free(labscim_ll_pop_node(&gThreadCommands, iter));
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
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 6: LabSCim - Send statistics back to Omnet++ --------- */
volatile MQTTAsync_token deliveredtoken;

volatile uint32_t finished = 0;
volatile uint32_t subscribed = 0;

void mqtt_onConnect(void* context, MQTTAsync_successData* response)
{
        MQTTAsync client = (MQTTAsync)context;
        MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
        
        int rc;
        printf("Successful connection\n");

        printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n", gMQTTTopic, CLIENTID, QOS);
        
        opts.onSuccess = mqtt_onSubscribe;
        opts.onFailure = mqtt_onSubscribeFailure;
        opts.context = client;

        deliveredtoken = 0;
        finished = 0;
        subscribed = 0;
        if ((rc = MQTTAsync_subscribe(client, gMQTTTopic, QOS, &opts)) != MQTTASYNC_SUCCESS)
        {
            printf("Failed to start subscribe, return code %d\n", rc);
            exit(EXIT_FAILURE);
        }
}

void mqtt_onSend(void* context, MQTTAsync_successData* response)
{
    printf("Message with token value %d delivery confirmed\n", response->token);
}

int mqtt_msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
{
    JSON_Value *root_val;
    JSON_Array *fields;
    JSON_Object *obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    unsigned long long ull = 0;

    MQTTAsync client = (MQTTAsync)context;
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    int rc;

    /* try to parse JSON */    

    const char s[2] = "/";
    const max_topics = 8;
    char *token[max_topics];
    uint8_t payload[255];
    uint8_t enc_payload[255];
    uint8_t return_topic[255];
    int8_t* data;
    uint32_t i;
    struct signal_emit* sig;
    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("%lu, Message arrived\n", tv.tv_sec*1000000+tv.tv_usec);
    printf("   topic: %s\n", topicName);
    printf("   message: %.*s\n", message->payloadlen, (char*)message->payload);   
   
   /* walk through other tokens */
   i = 0;
   token[0] = strtok(topicName, s);
   while(( token[i] != NULL )&&(i<max_topics))
   {
       i++;
       token[i] = strtok(NULL, s);       
   }
   

   if (strcmp("error", token[5])==0)
   {
       root_val = json_parse_string_with_comments(message->payload);
       if (root_val != NULL)
       {
           obj = json_value_get_object(root_val);
           data = json_object_get_string(obj, "error");
           if (data != NULL)
           {               
               sig = (struct signal_emit *)malloc(sizeof(struct signal_emit));
               sig->id = gPacketErrorSignal;
               sig->value = data[0];
               pthread_mutex_lock(&gEmitListMutex);
               labscim_ll_insert_at_back(&gEmitSignalList, (void *)sig);
               pthread_mutex_unlock(&gEmitListMutex);
           }
       }
   }

   if(strcmp("up",token[5])==0)
   {
       root_val = json_parse_string_with_comments(message->payload);
       if (root_val != NULL)
       {
           obj = json_value_get_object(root_val);                                 
           data = json_object_get_string(obj, "data");
           if (data != NULL)
           {               
               b64_to_bin(data, strlen(data), payload, 255);               
               uint64_t *values = (uint64_t *)payload;
               sig = (struct signal_emit*)malloc(sizeof(struct signal_emit));
               sig->id = gPacketLatencySignal;
               sig->value = (gLabscimTime - values[1])/1e6;
               pthread_mutex_lock(&gEmitListMutex);
               labscim_ll_insert_at_back(&gEmitSignalList,(void*)sig);
               pthread_mutex_unlock(&gEmitListMutex);
               if (client != NULL)
               {
                  values[2] = gLabscimTime;
                   bin_to_b64(payload, sizeof(uint64_t) * 3, enc_payload, 255);
                   sprintf(payload, "{\"confirmed\": false, \"fPort\": 2, \"data\": \"%s\"}", enc_payload);
                   opts.onSuccess = mqtt_onSend;
                   opts.context = client;
                   pubmsg.payload = payload;
                   pubmsg.payloadlen = strlen(payload);
                   pubmsg.qos = QOS;
                   pubmsg.retained = 0;
                   deliveredtoken = 0;
                   //application/[ApplicationID]/device/[DevEUI]/command/down
                //    sprintf(return_topic, "application/%s/%s/%s/command/down", token[1], token[2], token[3]);
                //    if ((rc = MQTTAsync_sendMessage(client, return_topic, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
                //    {
                //        printf("Failed to start sendMessage, return code %d\n", rc);
                //        exit(EXIT_FAILURE);
                //    }
                   sig = (struct signal_emit *)malloc(sizeof(struct signal_emit));
                   sig->id = gPacketGeneratedSignal;
                   sig->value = gLabscimTime / 1e6;
                   pthread_mutex_lock(&gEmitListMutex);
                   labscim_ll_insert_at_back(&gEmitSignalList, (void *)sig);
                   pthread_mutex_unlock(&gEmitListMutex);
               }
           }
       }
   }
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
}

void mqtt_connlost(void *context, char *cause)
{
    MQTTAsync client = (MQTTAsync)context;
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    int rc;
    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
    printf("Reconnecting\n");
    conn_opts.keepAliveInterval = 10;
    conn_opts.cleansession = 1;
    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to start connect, return code %d\n", rc);
        finished = 1;
    }
}

void mqtt_onConnectFailure(void* context, MQTTAsync_failureData* response)
{
        printf("Connect failed, rc %d\n", response ? response->code : 0);
        finished = 1;
}

void mqtt_onDisconnect(void* context, MQTTAsync_successData* response)
{
        printf("Successful disconnection\n");
        finished = 1;
}

void mqtt_onSubscribe(void* context, MQTTAsync_successData* response)
{
        printf("Subscribe succeeded\n");
        subscribed = 1;
}

void mqtt_onSubscribeFailure(void* context, MQTTAsync_failureData* response)
{
        printf("Subscribe failed, rc %d\n", response ? response->code : 0);
        finished = 1;
        subscribed = 0;
}


void thread_labscim_mqtt(void) 
{
    MQTTAsync client;
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
    MQTTAsync_token token;
    int rc;

    pthread_mutex_lock(&gEmitListMutex);
    labscim_ll_init_list(&gEmitSignalList);
    pthread_mutex_unlock(&gEmitListMutex);

    MQTTAsync_create(&client, gMQTTAddress, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTAsync_setCallbacks(client, (void*)client, mqtt_connlost, mqtt_msgarrvd, NULL);     

    conn_opts.keepAliveInterval = 10;
    conn_opts.cleansession = 1;
    conn_opts.onSuccess = mqtt_onConnect;
    conn_opts.onFailure = mqtt_onConnectFailure;
    conn_opts.context = client;
    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to start connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    
    /* main loop task */
    while (!lib_exit_sig && !lib_quit_sig)
    {
        wait_ms(500);
        if(finished)
        {
            mqtt_connlost(client, " reconnect");
        }
    }
    
    disc_opts.onSuccess = mqtt_onDisconnect;
    if ((rc = MQTTAsync_disconnect(client, &disc_opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to start sendMessage, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    
destroy_exit:
    MQTTAsync_destroy(&client);
exit:
    printf("\nINFO: End of validation thread\n");
    return rc;
}

void labscim_interface_thread(void) 
{
    uint64_t ret;
    labscim_ll_init_list(&gReceivedPackets);    
    
    while (!lib_exit_sig && !lib_quit_sig) 
    {     
        pthread_mutex_lock(gNodeInputBuffer->mutex.mutex);                
        labscim_socket_handle_input(gNodeInputBuffer, &gCommands);        
        while (gCommands.count == 0)
        {
            pthread_cond_wait(gNodeInputBuffer->mutex.more, gNodeInputBuffer->mutex.mutex);            
            labscim_socket_handle_input(gNodeInputBuffer, &gCommands);            
        }
        pthread_cond_signal(gNodeInputBuffer->mutex.less);
        pthread_mutex_unlock(gNodeInputBuffer->mutex.mutex);
        //socket_process_all_commands();
        void *cmd;

        pthread_mutex_lock(&gWaitingCommandsMutex);
        uint64_t processed = 0;
        //process returned commands (if any)
        do
        {
            cmd = labscim_ll_pop_front(&gCommands);
            if (cmd != NULL)
            {
                struct labscim_protocol_header *hdr = (struct labscim_protocol_header *)cmd;
                ret = socket_process_command(hdr);
                processed++;
            }
        } while (cmd != NULL);
        
        if(gThreadCommands.count > 0)
        {
            pthread_cond_broadcast(&gWaitingCommandsMore);
        } 
        else 
        {
            EmitAndYield();            
        }
        pthread_mutex_unlock(&gWaitingCommandsMutex);
    }
}




void labscim_radio_incoming_response(struct labscim_radio_response *cmd)
{
    switch (cmd->radio_response_code)
    {
    case LORA_RADIO_PACKET_RECEIVED:    
        labscim_ll_insert_at_back(&gReceivedPackets,cmd);
        break;    
    case LORA_RADIO_GET_STATE_RESULT:
        labscim_ll_insert_at_back(&gThreadCommands,cmd);
        break;
    default:
        free(cmd);        
        break;
    }
}

void lgw_labscim_get_time(struct timeval *time)
{
    struct timespec utc;
    lgw_gps_get(&utc, NULL, NULL, NULL);   
    time->tv_sec = utc.tv_sec;
    time->tv_usec = utc.tv_nsec/1000;
    //time->tv_sec = gLabscimTime/1000000;
    //time->tv_usec = gLabscimTime%1000000;
}


int lgw_start(uint8_t* NodeName, uint32_t BufferSize, uint64_t* GatewayMac)
{
    int i;    

    if (lgw_is_started == true)
    {
        DEBUG_MSG("Note: LoRa concentrator already started, restarting it now\n");
    }

    lib_exit_sig = false;
    lib_quit_sig = false;

    uint8_t *inbuffername, *outbuffername;
    inbuffername = (uint8_t *)malloc(sizeof(uint8_t) * strlen(NodeName) + 4);
    if (inbuffername == NULL)
    {
        perror("\nMalloc\n");
        return;
    }
    memcpy(inbuffername + 1, NodeName, strlen(NodeName));
    memcpy(inbuffername + strlen(NodeName) + 1, "in", 2);
    inbuffername[0] = '/';
    inbuffername[strlen(NodeName) + 3] = 0;

    outbuffername = (uint8_t *)malloc(sizeof(uint8_t) * strlen(NodeName) + 5);
    if (outbuffername == NULL)
    {
        perror("\nMalloc\n");
        return;
    }
    memcpy(outbuffername + 1, NodeName, strlen(NodeName));
    memcpy(outbuffername + strlen(NodeName) + 1, "out", 3);
    outbuffername[0] = '/';
    outbuffername[strlen(NodeName) + 4] = 0;

    gNodeOutputBuffer = (buffer_circ_t *)malloc(sizeof(buffer_circ_t));
    if (gNodeOutputBuffer == NULL)
    {
        perror("\nMalloc\n");
        return;
    }
    labscim_buffer_init(gNodeOutputBuffer, outbuffername, BufferSize, 0);
    labscim_ll_init_list(&gCommands);
    gNodeInputBuffer = (buffer_circ_t *)malloc(sizeof(buffer_circ_t));
    if (gNodeInputBuffer == NULL)
    {
        perror("\nMalloc\n");
        return;
    }
    labscim_buffer_init(gNodeInputBuffer, inbuffername, BufferSize, 1);
    node_is_ready(gNodeOutputBuffer);
    while (!gBootReceived)
    {
        //shared memory communication
        pthread_mutex_lock(gNodeInputBuffer->mutex.mutex);
        labscim_socket_handle_input(gNodeInputBuffer, &gCommands);

        while (gCommands.count == 0)
        {
            pthread_cond_wait(gNodeInputBuffer->mutex.more, gNodeInputBuffer->mutex.mutex);
            labscim_socket_handle_input(gNodeInputBuffer, &gCommands);
        }
        pthread_cond_signal(gNodeInputBuffer->mutex.less);
        pthread_mutex_unlock(gNodeInputBuffer->mutex.mutex);
        pthread_mutex_lock(&gWaitingCommandsMutex);
        void* cmd;
        do
        {
            cmd = labscim_ll_pop_front(&gCommands);
            if(cmd!=NULL)
            {                
                labscim_ll_insert_at_back(&gThreadCommands, cmd);                
            }            
        } while (cmd!=NULL);
        pthread_mutex_unlock(&gWaitingCommandsMutex);
        lgw_process_all_commands();
    }
    *GatewayMac = *((uint64_t*)mac_addr);   



    
    i = pthread_create(&gThridIf, NULL, (void *(*)(void *))labscim_interface_thread, NULL);
    if (i != 0)
    {
        DEBUG_MSG("ERROR: [main] impossible to create library thread\n");
        return LGW_HAL_ERROR;
    }
    lgw_is_started = true;

    if (gCommandLabscimLog)
    {
		gPacketGeneratedSignal = LabscimSignalRegister("LoRaDownstreamPacketGenerated");
		gPacketLatencySignal = LabscimSignalRegister("LoRaUpstreamPacketLatency");
        gPacketErrorSignal = LabscimSignalRegister("LoRaUpstreamPacketError");

        i = pthread_create(&gThridMQTT, NULL, (void *(*)(void *))thread_labscim_mqtt, NULL);
        if (i != 0)
        {
            DEBUG_MSG("ERROR: [main] impossible to labscim mqtt thread\n");
            exit(EXIT_FAILURE);
        }
    } 


    //
    int64_t max_freq=rf_rx_freq[(uint8_t)if_rf_chain[0]] + if_freq[0];
    int64_t min_freq=rf_rx_freq[(uint8_t)if_rf_chain[0]] + if_freq[0];

    for (int64_t i = 0; i < LGW_IF_CHAIN_NB; i++)
    {

        if (rf_rx_freq[(uint8_t)if_rf_chain[i]] + if_freq[i] > max_freq)
        {
            max_freq = rf_rx_freq[(uint8_t)if_rf_chain[i]] + if_freq[i];
        }

        if (rf_rx_freq[(uint8_t)if_rf_chain[i]] + if_freq[i] < min_freq)
        {
            min_freq = rf_rx_freq[(uint8_t)if_rf_chain[i]] + if_freq[i];
        }
    }

    //set channel and bandwidth - this has no practical effect since gateway radio always receive all packets, but this way, spectrum plotter plots the right listening window
    struct lora_set_frequency frequency_setup;    
    frequency_setup.Frequency_Hz = (max_freq-min_freq)/2 + min_freq;
    radio_command(gNodeOutputBuffer, LORA_RADIO_SET_CHANNEL, (void *)&frequency_setup, sizeof(struct lora_set_frequency));

    struct lora_set_modulation_params mp;
    ModulationParams_t radio_setup;    
    mp.TransmitPower_dBm = 27;
    mp.ModulationParams.Params.LoRa.CodingRate = 1;
    mp.ModulationParams.Params.LoRa.SpreadingFactor = 7;
    radio_setup.Params.LoRa.Bandwidth = (max_freq-min_freq)+200000;
    memcpy(&(mp.ModulationParams), &radio_setup, sizeof(ModulationParams_t));
    radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODULATION_PARAMS, (void *)&mp, sizeof(struct lora_set_modulation_params));

    //set gateway to RX mode
    struct lora_set_rx srx;
    srx.Timeout_us = ~((uint64_t)0); //continuous RX    
    radio_command(gNodeOutputBuffer, LORA_RADIO_SET_RX, (void *)&srx, sizeof(struct lora_set_rx));

    //then set modulation to lora
    struct lora_set_modem modem_type;
    uint32_t sequence_number;
    modem_type.Modem = 1;//(uint32_t)MODEM_LORA;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_SET_MODEM, (void *)&modem_type, sizeof(struct lora_set_modem));
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void)
{
    lgw_soft_reset();
    lgw_disconnect();
    lib_exit_sig = true;
    lib_quit_sig = true;

    lgw_is_started = false;
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data)
{
    int nb_pkt_fetch;                     /* loop variable and return value */
    struct lgw_pkt_rx_s *p;               /* pointer to the current structure in the struct array */
    uint8_t buff[255 + RX_METADATA_NB];   /* buffer to store the result of SPI read bursts */
    unsigned sz;                          /* size of the payload, uses to address metadata */
    int ifmod;                            /* type of if_chain/modem a packet was received by */
    int stat_fifo;                        /* the packet status as indicated in the FIFO */
    uint32_t raw_timestamp;               /* timestamp when internal 'RX finished' was triggered */
    uint32_t delay_x, delay_y, delay_z;   /* temporary variable for timestamp offset calculation */
    uint32_t timestamp_correction;        /* correction to account for processing delay */
    uint32_t sf, cr, bw_pow, crc_en, ppm; /* used to calculate timestamp correction */
    struct labscim_radio_response *cmd;
    struct lora_radio_payload* payload;

    /* check if the concentrator is running */
    if (lgw_is_started == false)
    {
        DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE RECEIVING\n");
        return LGW_HAL_ERROR;
    }

    /* check input variables */
    if ((max_pkt <= 0) || (max_pkt > LGW_PKT_FIFO_SIZE))
    {
        DEBUG_PRINTF("ERROR: %d = INVALID MAX NUMBER OF PACKETS TO FETCH\n", max_pkt);
        return LGW_HAL_ERROR;
    }
    CHECK_NULL(pkt_data);

    /* Initialize buffer */
    memset(buff, 0, sizeof buff);

    /* iterate max_pkt times at most */
    nb_pkt_fetch = 0;
    while (nb_pkt_fetch < max_pkt) 
    {
        /* point to the proper struct in the struct array */
        p = &pkt_data[nb_pkt_fetch];

        /* fetch all the RX FIFO data */
        //lgw_reg_rb(LGW_RX_PACKET_DATA_FIFO_NUM_STORED, buff, 5);
        /* 0:   number of packets available in RX data buffer */
        /* 1,2: start address of the current packet in RX data buffer */
        /* 3:   CRC status of the current packet */
        /* 4:   size of the current packet payload in byte */

        /* how many packets are in the RX buffer ? Break if zero */
        //if (buff[0] == 0)
        //{
            //    break; /* no more packets to fetch, exit out of FOR loop */
        //}

        /* sanity check */
        //if (buff[0] > LGW_PKT_FIFO_SIZE)
        //{
        //    DEBUG_PRINTF("WARNING: %u = INVALID NUMBER OF PACKETS TO FETCH, ABORTING\n", buff[0]);
        //    break;
        //}

        //DEBUG_PRINTF("FIFO content: %x %x %x %x %x\n", buff[0], buff[1], buff[2], buff[3], buff[4]);
        if(gReceivedPackets.count > 0)
        {
            cmd = (struct labscim_radio_response*)labscim_ll_pop_front(&gReceivedPackets);            
            payload = (struct lora_radio_payload*)(cmd->radio_struct);
            p->size = payload->MessageSize_bytes;
            //sz = p->size;
        

            /* get payload + metadata */
            //lgw_reg_rb(LGW_RX_DATA_BUF_DATA, buff, sz + RX_METADATA_NB);

            /* process metadata  TODO: set the ifchain properly*/
            p->if_chain = 0xFF;

            for (int64_t i = 0; i < LGW_IF_CHAIN_NB; i++)
            {
                if (rf_rx_freq[(uint8_t)if_rf_chain[i]] + if_freq[i] == payload->CenterFrequency_Hz)
                {
                    p->if_chain = i;
                    break;
                }
            }

            if (p->if_chain == 0xFF)
            {
                //this packet would not be received if our gateway model were more accurate - ignoring
                free(cmd);
                continue;
            }
            else
            {
                /* copy payload to result struct */
                memcpy((void *)p->payload, (void *)payload->Message, payload->MessageSize_bytes);

                //if (p->if_chain >= LGW_IF_CHAIN_NB)
                //{
                //   DEBUG_PRINTF("WARNING: %u NOT A VALID IF_CHAIN NUMBER, ABORTING\n", p->if_chain);
                //    break;
                //}
                ifmod = ifmod_config[p->if_chain];
                //DEBUG_PRINTF("[%d %d]\n", p->if_chain, ifmod);
                p->rf_chain = (uint8_t)if_rf_chain[p->if_chain];
                p->freq_hz = (uint32_t)((int32_t)rf_rx_freq[p->rf_chain] + if_freq[p->if_chain]);
                p->rssi = payload->RSSI_dbm-30;
                if (payload->CRCError)
                {
                    p->status = STAT_CRC_BAD;
                }
                else
                {
                    p->status = STAT_CRC_OK;
                }
                if ((ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD))
                {
                    DEBUG_MSG("Note: LoRa packet\n");

                    p->modulation = MOD_LORA;
                    p->snr = payload->SNR_db;
                    p->snr_min = payload->SNR_db;
                    p->snr_max = payload->SNR_db;
                    if (ifmod == IF_LORA_MULTI)
                    {
                        p->bandwidth = BW_125KHZ; /* fixed in hardware */
                    }
                    else
                    {
                        p->bandwidth = lora_rx_bw; /* get the parameter from the config variable */
                    }
                    switch (payload->LoRaSF)
                    {
                    case 7:
                        p->datarate = DR_LORA_SF7;
                        break;
                    case 8:
                        p->datarate = DR_LORA_SF8;
                        break;
                    case 9:
                        p->datarate = DR_LORA_SF9;
                        break;
                    case 10:
                        p->datarate = DR_LORA_SF10;
                        break;
                    case 11:
                        p->datarate = DR_LORA_SF11;
                        break;
                    case 12:
                        p->datarate = DR_LORA_SF12;
                        break;
                    default:
                        p->datarate = DR_UNDEFINED;
                    }

                    switch (payload->LoRaCR)
                    {
                    case 1:
                        p->coderate = CR_LORA_4_5;
                        break;
                    case 2:
                        p->coderate = CR_LORA_4_6;
                        break;
                    case 3:
                        p->coderate = CR_LORA_4_7;
                        break;
                    case 4:
                        p->coderate = CR_LORA_4_8;
                        break;
                    default:
                        p->coderate = CR_UNDEFINED;
                    }
                }
                else if (ifmod == IF_FSK_STD)
                {
                    DEBUG_MSG("Note: FSK packet\n");
                    p->modulation = MOD_FSK;
                    p->snr = -128.0;
                    p->snr_min = -128.0;
                    p->snr_max = -128.0;
                    p->bandwidth = fsk_rx_bw;
                    p->datarate = fsk_rx_dr;
                    p->coderate = CR_UNDEFINED;
                }
                else
                {
                    DEBUG_MSG("ERROR: UNEXPECTED PACKET ORIGIN\n");
                    p->status = STAT_UNDEFINED;
                    p->modulation = MOD_UNDEFINED;
                    p->rssi = -128.0;
                    p->snr = -128.0;
                    p->snr_min = -128.0;
                    p->snr_max = -128.0;
                    p->bandwidth = BW_UNDEFINED;
                    p->datarate = DR_UNDEFINED;
                    p->coderate = CR_UNDEFINED;
                    timestamp_correction = 0;
                }
                free(cmd);
            }
            p->count_us = payload->RX_timestamp_us;
            p->crc = 0;
            nb_pkt_fetch++;
            //TODO: p->crc = (uint16_t)buff[sz + 10] + ((uint16_t)buff[sz + 11] << 8);
        }
        else
        {
            break;
        }
    }
    return nb_pkt_fetch;
}

/* ~~ */

char gBuffer[256];
int labscim_printf(const char *fmt, ...)
{
	#define LOGLEVEL_INFO (3)    
    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(gBuffer, sizeof(gBuffer), fmt, args);	
	if(rc>sizeof(gBuffer))
	{
		strcpy(gBuffer+strlen(gBuffer)-4,"...");		
		rc = sizeof(gBuffer);
	}
	gBuffer[sizeof(gBuffer)-1]=0;
    printf("%s",gBuffer);
	print_message(gNodeOutputBuffer,LOGLEVEL_INFO,gBuffer,strlen(gBuffer)+1);
	va_end(args);
    return rc;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s pkt_data)
{
    int i, x; 
    struct lora_radio_payload* msg;
    uint8_t labscim_buff[FIXED_SIZEOF_LORA_RADIO_PAYLOAD + 256];   
    uint8_t buff[TX_METADATA_NB]; /* buffer to prepare the packet to send + metadata before SPI write burst */
    uint32_t part_int = 0;              /* integer part for PLL register value calculation */
    uint32_t part_frac = 0;             /* fractional part for PLL register value calculation */
    uint16_t fsk_dr_div;                /* divider to configure for target datarate */
    int transfer_size = 0;              /* data to transfer from host to TX databuffer */
    int payload_offset = 0;             /* start of the payload content in the databuffer */
    uint8_t pow_index = 0;              /* 4-bit value to set the firmware TX power */
    uint8_t target_mix_gain = 0;        /* used to select the proper I/Q offset correction */
    uint32_t count_trig = 0;            /* timestamp value in trigger mode corrected for TX start delay */
    bool tx_allowed = false;
    uint16_t tx_start_delay;
    bool tx_notch_enable = false;

    msg = (struct lora_radio_payload*)labscim_buff;    

    /* check if the concentrator is running */
    if (lgw_is_started == false)
    {
        DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
        return LGW_HAL_ERROR;
    }

    /* check input range (segfault prevention) */
    if (pkt_data.rf_chain >= LGW_RF_CHAIN_NB)
    {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
        return LGW_HAL_ERROR;
    }

    /* check input variables */
    if (rf_tx_enable[pkt_data.rf_chain] == false)
    {
        DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
        return LGW_HAL_ERROR;
    }
    if (rf_enable[pkt_data.rf_chain] == false)
    {
        DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
        return LGW_HAL_ERROR;
    }
    if (!IS_TX_MODE(pkt_data.tx_mode))
    {
        DEBUG_MSG("ERROR: TX_MODE NOT SUPPORTED\n");
        return LGW_HAL_ERROR;
    }
    if (pkt_data.modulation == MOD_LORA)
    {
        if (!IS_LORA_BW(pkt_data.bandwidth))
        {
            DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_STD_DR(pkt_data.datarate))
        {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_CR(pkt_data.coderate))
        {
            DEBUG_MSG("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data.size > 255)
        {
            DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
            return LGW_HAL_ERROR;
        }
    }
    else if (pkt_data.modulation == MOD_FSK)
    {
        if ((pkt_data.f_dev < 1) || (pkt_data.f_dev > 200))
        {
            DEBUG_MSG("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_FSK_DR(pkt_data.datarate))
        {
            DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data.size > 255)
        {
            DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
            return LGW_HAL_ERROR;
        }
    }
    else
    {
        DEBUG_MSG("ERROR: INVALID TX MODULATION\n");
        return LGW_HAL_ERROR;
    }

    

    /* Enable notch filter for LoRa 125kHz */
    if ((pkt_data.modulation == MOD_LORA) && (pkt_data.bandwidth == BW_125KHZ))
    {
        tx_notch_enable = true;
    }

    /* Get the TX start delay to be applied for this TX */
    tx_start_delay = lgw_get_tx_start_delay(tx_notch_enable, pkt_data.bandwidth);

    /* interpretation of TX power */
    for (pow_index = txgain_lut.size - 1; pow_index > 0; pow_index--)
    {
        if (txgain_lut.lut[pow_index].rf_power <= pkt_data.rf_power)
        {
            break;
        }
    }

    
    msg->TxPower_dbm = pkt_data.rf_power;


    /* loading TX imbalance correction */
    target_mix_gain = txgain_lut.lut[pow_index].mix_gain;
    // if (pkt_data.rf_chain == 0)
    // { /* use radio A calibration table */
    //     lgw_reg_w(LGW_TX_OFFSET_I, cal_offset_a_i[target_mix_gain - 8]);
    //     lgw_reg_w(LGW_TX_OFFSET_Q, cal_offset_a_q[target_mix_gain - 8]);
    // }
    // else
    // { /* use radio B calibration table */
    //     lgw_reg_w(LGW_TX_OFFSET_I, cal_offset_b_i[target_mix_gain - 8]);
    //     lgw_reg_w(LGW_TX_OFFSET_Q, cal_offset_b_q[target_mix_gain - 8]);
    // }

    /* Set digital gain from LUT */
    //lgw_reg_w(LGW_TX_GAIN, txgain_lut.lut[pow_index].dig_gain);

    /* fixed metadata, useful payload and misc metadata compositing */
    transfer_size = TX_METADATA_NB + pkt_data.size; /*  */
    payload_offset = TX_METADATA_NB;                /* start the payload just after the metadata */
    
    msg->MessageSize_bytes = pkt_data.size;

    /* metadata 0 to 2, TX PLL frequency */
    switch (rf_radio_type[0])
    { /* we assume that there is only one radio type on the board */
    case LGW_RADIO_TYPE_SX1255:
        part_int = pkt_data.freq_hz / (SX125x_32MHz_FRAC << 7);                               /* integer part, gives the MSB */
        part_frac = ((pkt_data.freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
        break;
    case LGW_RADIO_TYPE_SX1257:
        part_int = pkt_data.freq_hz / (SX125x_32MHz_FRAC << 8);                               /* integer part, gives the MSB */
        part_frac = ((pkt_data.freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
        break;
    default:
        DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE\n", rf_radio_type[0]);
        break;
    }

    buff[0] = 0xFF & part_int;         /* Most Significant Byte */
    buff[1] = 0xFF & (part_frac >> 8); /* middle byte */
    buff[2] = 0xFF & part_frac;        /* Least Significant Byte */

    /* metadata 3 to 6, timestamp trigger value */
    /* TX state machine must be triggered at (T0 - lgw_i_tx_start_delay_us) for packet to start being emitted at T0 */
    if (pkt_data.tx_mode == TIMESTAMPED)
    {
        count_trig = pkt_data.count_us - (uint32_t)tx_start_delay;
        buff[3] = 0xFF & (count_trig >> 24);
        buff[4] = 0xFF & (count_trig >> 16);
        buff[5] = 0xFF & (count_trig >> 8);
        buff[6] = 0xFF & count_trig;
    }

    msg->Tx_Delay_us = pkt_data.count_us-gLabscimTime;
    msg->CenterFrequency_Hz = pkt_data.freq_hz;

    /* parameters depending on modulation  */
    if (pkt_data.modulation == MOD_LORA)
    {
        
        /* metadata 7, modulation type, radio chain selection and TX power */
        buff[7] = (0x20 & (pkt_data.rf_chain << 5)) | (0x0F & pow_index); /* bit 4 is 0 -> LoRa modulation */

        buff[8] = 0; /* metadata 8, not used */

        /* metadata 9, CRC, LoRa CR & SF */
        switch (pkt_data.datarate)
        {
        case DR_LORA_SF7:
            buff[9] = 7;            
            break;
        case DR_LORA_SF8:
            buff[9] = 8;
            break;
        case DR_LORA_SF9:
            buff[9] = 9;
            break;
        case DR_LORA_SF10:
            buff[9] = 10;
            break;
        case DR_LORA_SF11:
            buff[9] = 11;
            break;
        case DR_LORA_SF12:
            buff[9] = 12;
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.datarate);
        }
        msg->LoRaSF = buff[9];
        switch (pkt_data.coderate)
        {
        case CR_LORA_4_5:
            buff[9] |= 1 << 4;
            break;
        case CR_LORA_4_6:
            buff[9] |= 2 << 4;
            break;
        case CR_LORA_4_7:
            buff[9] |= 3 << 4;
            break;
        case CR_LORA_4_8:
            buff[9] |= 4 << 4;
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.coderate);
        }
        msg->LoRaCR = pkt_data.coderate;
        if (pkt_data.no_crc == false)
        {
            buff[9] |= 0x80; /* set 'CRC enable' bit */
        }
        else
        {
            DEBUG_MSG("Info: packet will be sent without CRC\n");
        }

        /* metadata 10, payload size */
        buff[10] = pkt_data.size;

        /* metadata 11, implicit header, modulation bandwidth, PPM offset & polarity */
        switch (pkt_data.bandwidth)
        {
        case BW_125KHZ:
            buff[11] = 0;
            msg->LoRaBandwidth_Hz = 125000;
            break;
        case BW_250KHZ:
            msg->LoRaBandwidth_Hz = 250000;
            buff[11] = 1;
            break;
        case BW_500KHZ:
            msg->LoRaBandwidth_Hz = 500000;
            buff[11] = 2;
            break;
        default:
            DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.bandwidth);
        }
        if (pkt_data.no_header == true)
        {
            buff[11] |= 0x04; /* set 'implicit header' bit */
        }
        if (SET_PPM_ON(pkt_data.bandwidth, pkt_data.datarate))
        {
            buff[11] |= 0x08; /* set 'PPM offset' bit at 1 */
        }
        if (pkt_data.invert_pol == true)
        {
            buff[11] |= 0x10; /* set 'TX polarity' bit at 1 */
        }

        /* metadata 12 & 13, LoRa preamble size */
        if (pkt_data.preamble == 0)
        { /* if not explicit, use recommended LoRa preamble size */
            pkt_data.preamble = STD_LORA_PREAMBLE;
        }
        else if (pkt_data.preamble < MIN_LORA_PREAMBLE)
        { /* enforce minimum preamble size */
            pkt_data.preamble = MIN_LORA_PREAMBLE;
            DEBUG_MSG("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
        }
        buff[12] = 0xFF & (pkt_data.preamble >> 8);
        buff[13] = 0xFF & pkt_data.preamble;

        /* metadata 14 & 15, not used */
        buff[14] = 0;
        buff[15] = 0;

        /* MSB of RF frequency is now used in AGC firmware to implement large/narrow filtering in SX1257/55 */
        buff[0] &= 0x3F; /* Unset 2 MSBs of frequency code */
        if (pkt_data.bandwidth == BW_500KHZ)
        {
            buff[0] |= 0x80; /* Set MSB bit to enlarge analog filter for 500kHz BW */
        }

        /* Set MSB-1 bit to enable digital filter if required */
        if (tx_notch_enable == true)
        {
            DEBUG_MSG("INFO: Enabling TX notch filter\n");
            buff[0] |= 0x40;
        }
    }
    else if (pkt_data.modulation == MOD_FSK)
    {
        /* metadata 7, modulation type, radio chain selection and TX power */
        buff[7] = (0x20 & (pkt_data.rf_chain << 5)) | 0x10 | (0x0F & pow_index); /* bit 4 is 1 -> FSK modulation */

        buff[8] = 0; /* metadata 8, not used */

        /* metadata 9, frequency deviation */
        buff[9] = pkt_data.f_dev;

        /* metadata 10, payload size */
        buff[10] = pkt_data.size;
        /* TODO: how to handle 255 bytes packets ?!? */

        /* metadata 11, packet mode, CRC, encoding */
        buff[11] = 0x01 | (pkt_data.no_crc ? 0 : 0x02) | (0x02 << 2); /* always in variable length packet mode, whitening, and CCITT CRC if CRC is not disabled  */

        /* metadata 12 & 13, FSK preamble size */
        if (pkt_data.preamble == 0)
        { /* if not explicit, use LoRa MAC preamble size */
            pkt_data.preamble = STD_FSK_PREAMBLE;
        }
        else if (pkt_data.preamble < MIN_FSK_PREAMBLE)
        { /* enforce minimum preamble size */
            pkt_data.preamble = MIN_FSK_PREAMBLE;
            DEBUG_MSG("Note: preamble length adjusted to respect minimum FSK preamble size\n");
        }
        buff[12] = 0xFF & (pkt_data.preamble >> 8);
        buff[13] = 0xFF & pkt_data.preamble;

        /* metadata 14 & 15, FSK baudrate */
        fsk_dr_div = (uint16_t)((uint32_t)LGW_XTAL_FREQU / pkt_data.datarate); /* Ok for datarate between 500bps and 250kbps */
        buff[14] = 0xFF & (fsk_dr_div >> 8);
        buff[15] = 0xFF & fsk_dr_div;

        /* insert payload size in the packet for variable mode */
        buff[16] = pkt_data.size;
        ++transfer_size;  /* one more byte to transfer to the TX modem */
        ++payload_offset; /* start the payload with one more byte of offset */

        /* MSB of RF frequency is now used in AGC firmware to implement large/narrow filtering in SX1257/55 */
        buff[0] &= 0x7F; /* Always use narrow band for FSK (force MSB to 0) */
    }
    else
    {
        DEBUG_MSG("ERROR: INVALID TX MODULATION..\n");
        return LGW_HAL_ERROR;
    }

    /* Configure TX start delay based on TX notch filter */
    //lgw_reg_w(LGW_TX_START_DELAY, tx_start_delay);

    /* copy payload from user struct to buffer containing metadata */
    //memcpy((void *)(buff + payload_offset), (void *)(pkt_data.payload), pkt_data.size);

    /* reset TX command flags */
    //lgw_abort_tx();

    msg->SNR_db = -200.0;
    msg->RSSI_dbm = -200.0;
    msg->RX_timestamp_us = 0;    

    /* put metadata + payload in the TX data buffer */
    //lgw_reg_w(LGW_TX_DATA_BUF_ADDR, 0);
    //lgw_reg_wb(LGW_TX_DATA_BUF_DATA, buff, transfer_size);
    //DEBUG_ARRAY(i, transfer_size, buff);

    uint8_t code;
    x = lbt_is_channel_free(&pkt_data, tx_start_delay, &tx_allowed);
    //x = lgw_status(RX_STATUS , &code);
    
    if (x != LGW_HAL_SUCCESS)
    {
        DEBUG_MSG("ERROR: Failed to check channel availability for TX\n");
        return LGW_HAL_ERROR;
    }
    if (tx_allowed == true)
    {
        // switch (pkt_data.tx_mode)
        // {
        // case IMMEDIATE:
        //     lgw_reg_w(LGW_TX_TRIG_IMMEDIATE, 1);
        //     break;

        // case TIMESTAMPED:
        //     lgw_reg_w(LGW_TX_TRIG_DELAYED, 1);
        //     break;

        // case ON_GPS:
        //     lgw_reg_w(LGW_TX_TRIG_GPS, 1);
        //     break;

        // default:
        //     DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.tx_mode);
        //     return LGW_HAL_ERROR;
        // }
        memcpy((void*)(msg->Message),(void *)(pkt_data.payload), pkt_data.size);
        radio_command(gNodeOutputBuffer, LORA_RADIO_SEND, (uint8_t *)msg, FIXED_SIZEOF_LORA_RADIO_PAYLOAD + msg->MessageSize_bytes);
    }
    else
    {
        DEBUG_MSG("ERROR: Cannot send packet, channel is busy (LBT)\n");
        return LGW_LBT_ISSUE;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t select, uint8_t *code)
{
    int32_t read_value;
    /* check input variables */
    CHECK_NULL(code);
    uint32_t ChannelIsFree;
    struct labscim_radio_response *resp;
    struct lora_radio_status radio_status;
    uint32_t sequence_number;
    sequence_number = radio_command(gNodeOutputBuffer, LORA_RADIO_GET_STATE, (void *)&radio_status, sizeof(struct lora_radio_status));
    resp = (struct labscim_radio_response *)lgw_wait_for_command(LABSCIM_RADIO_RESPONSE, sequence_number);
    if (select == TX_STATUS)
    {
        if (lgw_is_started == false)
        {
            *code = TX_OFF;
        }
        else
        {
            if (resp->radio_response_code == LORA_RADIO_GET_STATE_RESULT)
            {
                switch ((RadioMode)((struct lora_radio_status *)resp->radio_struct)->RadioMode)
                {
                case RADIO_MODE_TRANSMITTER:
                {
                    *code = TX_EMITTING;
                    break;
                }
                case RADIO_MODE_RECEIVER:
                case RADIO_MODE_OFF:
                case RADIO_MODE_SLEEP:
                {
                    *code = TX_FREE;
                    break;
                }
                default:
                    //something very wrong
                    while (1)
                        ;
                }                
            }
            else
            {
                //something very wrong happened
                while (1)
                    ;
            }
        }            
    }
    else if (select == RX_STATUS)
    {
        if (resp->radio_response_code == LORA_RADIO_GET_STATE_RESULT)
        {
            if (((RadioMode)((struct lora_radio_status *)resp->radio_struct)->RadioMode) == RADIO_MODE_RECEIVER)
            {
                if ((RadioMode)((struct lora_radio_status *)resp->radio_struct)->ChannelIsFree)
                {
                    *code = RX_OFF;
                }
                else
                {
                    *code = RX_ON;
                }
            }
            else
            {
                *code = RX_STATUS_UNKNOWN;
            }
        }
        else
        {
            //something very wrong happened
            while (1)
                ;
        }
    }
    else
    {
        DEBUG_MSG("ERROR: SELECTION INVALID, NO STATUS TO RETURN\n");
        free(resp);
        return LGW_HAL_ERROR;
    }
    free(resp);
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(void)
{
    int i;

    i = lgw_reg_w(LGW_TX_TRIG_ALL, 0);

    if (i == LGW_REG_SUCCESS)
        return LGW_HAL_SUCCESS;
    else
        return LGW_HAL_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint64_t *trig_cnt_us)
{
    //int i;
    //int32_t val;

    //i = lgw_reg_r(LGW_TIMESTAMP, &val);
    //if (i == LGW_REG_SUCCESS)
    //{
        *trig_cnt_us = (uint64_t)(gLabscimTime);
        return LGW_HAL_SUCCESS;
    //}
    //else
    //{
    //    return LGW_HAL_ERROR;
    //}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char *lgw_version_info()
{
    return lgw_version_string;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet)
{
    int32_t val;
    uint8_t SF, H, DE;
    uint16_t BW;
    uint32_t payloadSymbNb, Tpacket;
    double Tsym, Tpreamble, Tpayload, Tfsk;

    if (packet == NULL)
    {
        DEBUG_MSG("ERROR: Failed to compute time on air, wrong parameter\n");
        return 0;
    }

    if (packet->modulation == MOD_LORA)
    {
        /* Get bandwidth */
        val = lgw_bw_getval(packet->bandwidth);
        if (val != -1)
        {
            BW = (uint16_t)(val / 1E3);
        }
        else
        {
            DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported bandwidth (0x%02X)\n", packet->bandwidth);
            return 0;
        }

        /* Get datarate */
        val = lgw_sf_getval(packet->datarate);
        if (val != -1)
        {
            SF = (uint8_t)val;
        }
        else
        {
            DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported datarate (0x%02X)\n", packet->datarate);
            return 0;
        }

        /* Duration of 1 symbol */
        Tsym = pow(2, SF) / BW;

        /* Duration of preamble */
        Tpreamble = ((double)(packet->preamble) + 4.25) * Tsym;

        /* Duration of payload */
        H = (packet->no_header == false) ? 0 : 1; /* header is always enabled, except for beacons */
        DE = (SF >= 11) ? 1 : 0;                  /* Low datarate optimization enabled for SF11 and SF12 */

        payloadSymbNb = 8 + (ceil((double)(8 * packet->size - 4 * SF + 28 + 16 - 20 * H) / (double)(4 * (SF - 2 * DE))) * (packet->coderate + 4)); /* Explicitely cast to double to keep precision of the division */

        Tpayload = payloadSymbNb * Tsym;

        /* Duration of packet */
        Tpacket = Tpreamble + Tpayload;
    }
    else if (packet->modulation == MOD_FSK)
    {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        Tfsk = (8 * (double)(packet->preamble + fsk_sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate) * 1E3;

        /* Duration of packet */
        Tpacket = (uint32_t)Tfsk + 1; /* add margin for rounding */
    }
    else
    {
        Tpacket = 0;
        DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    return Tpacket;
}

/* --- EOF ------------------------------------------------------------------ */
