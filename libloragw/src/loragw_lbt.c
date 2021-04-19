/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Functions used to handle the Listen Before Talk feature

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* abs, labs, llabs */
#include <string.h>     /* memset */

#include "loragw_radio.h"
#include "loragw_aux.h"
#include "loragw_lbt.h"
#include "loragw_fpga.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_LBT == 1
    #define DEBUG_MSG(str)              fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_REG_ERROR;}
#endif

#define LBT_TIMESTAMP_MASK  0x007FF000 /* 11-bits timestamp */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

extern void *lgw_spi_target; /*! generic pointer to the SPI device */
extern uint8_t lgw_spi_mux_mode; /*! current SPI mux mode used */
extern uint16_t lgw_i_tx_start_delay_us;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static bool lbt_enable;
static uint8_t lbt_nb_active_channel;
static int8_t lbt_rssi_target_dBm;
static int8_t lbt_rssi_offset_dB;
static uint32_t lbt_start_freq;
static struct lgw_conf_lbt_chan_s lbt_channel_cfg[LBT_CHANNEL_FREQ_NB];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

bool is_equal_freq(uint32_t a, uint32_t b);

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lbt_setconf(struct lgw_conf_lbt_s * conf) {
    int i;

    /* Check input parameters */
    if (conf == NULL) {
        return LGW_LBT_ERROR;
    }
    if ((conf->nb_channel < 1) || (conf->nb_channel > LBT_CHANNEL_FREQ_NB)) {
        DEBUG_PRINTF("ERROR: Number of defined LBT channels is out of range (%u)\n", conf->nb_channel);
        return LGW_LBT_ERROR;
    }

    /* Initialize LBT channels configuration */
    memset(lbt_channel_cfg, 0, sizeof lbt_channel_cfg);

    /* Set internal LBT config according to parameters */
    lbt_enable = conf->enable;
    lbt_nb_active_channel = conf->nb_channel;
    lbt_rssi_target_dBm = conf->rssi_target;
    lbt_rssi_offset_dB = conf->rssi_offset;

    for (i=0; i<lbt_nb_active_channel; i++) {
        lbt_channel_cfg[i].freq_hz = conf->channels[i].freq_hz;
        lbt_channel_cfg[i].scan_time_us = conf->channels[i].scan_time_us;
    }

    return LGW_LBT_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lbt_setup(void) {
    int x, i;
    int32_t val;
    uint32_t freq_offset;

    DEBUG_MSG("ERROR: No support for LBT in FPGA\n");
    return LGW_LBT_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lbt_start(void) {
    int x;
    DEBUG_MSG("ERROR: Failed to start LBT FSM\n");
    return LGW_LBT_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lbt_is_channel_free(struct lgw_pkt_tx_s * pkt_data, uint16_t tx_start_delay, bool * tx_allowed) 
{
    *tx_allowed = true;
    return LGW_LBT_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

bool lbt_is_enabled(void) {
    return lbt_enable;
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* As given frequencies have been converted from float to integer, some aliasing
issues can appear, so we can't simply check for equality, but have to take some
margin */
bool is_equal_freq(uint32_t a, uint32_t b) {
    int64_t diff;
    int64_t a64 = (int64_t)a;
    int64_t b64 = (int64_t)b;

    /* Calculate the difference */
    diff = llabs(a64 - b64);

    /* Check for acceptable diff range */
    if( diff <= 10000 )
    {
        return true;
    }

    return false;
}

/* --- EOF ------------------------------------------------------------------ */
