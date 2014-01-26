/*! \file 
 * File:        cell.h
 * Author:      martin
 * Created:     August 5, 2013, 5:18 PM
 */
#pragma once

#include <inttypes.h>
#include "ltc6804_regs.h"



/***                Definitions                 ***/

/***          Public Global Variables           ***/

/* States and parameters of a battery cell.
 * Based on model from University of Colorado ECEN5017 
 */
struct batt_cell_s {
    // cell states
    double soc;             ///< State of charge (0-1)
    double voc;             ///< Open Circuit Voltage (Volts)
    double termv;           ///< Terminal Voltage (Volts)
    double i;               ///< Cell current (Amps - positive current discharges cell)
    // cell parameters
    double eta_c;           ///< Coulomb Efficiency (0-1)
    double rs;              ///< Series Resistance (ohms))
    double r1;              ///< Diffusion model resistance (ohms)
    double tau1;            ///< Diffusion time constant
    double vm;              ///< Hysteresis voltage (Volts)
    double taum;            ///< Hysteresis time constant
    double capacity;        ///< Cell capacity (A*h)
    enum ltc6804_cell_flags_e flags;    ///< Cell flags
    // cell LPF filter states
    double lpf_vr;          ///< diffusion LPF output
    double lpf_vh;          ///< hysteresis LPF output
};



/***              Public Functions              ***/

double cell_compute_soc(double voc);
double cell_integrate_i(struct batt_cell_s *c, double dt);
void cell_compute_voc(struct batt_cell_s *c, double dt);

