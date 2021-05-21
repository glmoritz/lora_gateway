/*
 * labscim_helper.h
 *
 *  Created on: 20 de ago de 2020
 *      Author: root
 */

#ifndef EXAMPLES_6TISCH_SIMPLE_NODE_LABSCIM_HELPER_H_
#define EXAMPLES_6TISCH_SIMPLE_NODE_LABSCIM_HELPER_H_

#include <stdint.h>

struct signal_emit
{
    uint64_t id;
    double value;
};

uint64_t LabscimSignalRegister(uint8_t* signal_name);

void LabscimSignalEmit(uint64_t id, double value);


#endif /* EXAMPLES_6TISCH_SIMPLE_NODE_LABSCIM_HELPER_H_ */
