/*! \file 
 * File:        cell.c
 * Author:      martin
 * Created:     August 5, 2013, 5:19 PM
 */

#include "cell.h"




/***                Definitions                 ***/

/***         Private Global Variables           ***/

/***          Public Global Variables           ***/

/***        Private Function Prototypes         ***/

/***                   ISRs                     ***/

/***             Private Functions              ***/

/*! Recursive single order low-pass filter.
 *  \param dt Delta t, should remain the same between function calls.
 *  \param tau Filter time constant.
 *  \param in Filter input value.
 *  \param prev Prev filter output (t-1).
 *  \return Filter output.
 */
static double lpf_s(double dt, double tau, double in, double prev) {
    
    double alpha = dt / (tau + dt);     // could be pre-computed if constant.
    return prev + alpha * (in - prev);
}

/*! Compute instantaneous Vs
 *  \param rs Series resistance (ohms).
 *  \param icell Cell current (Amps).
 *  Note: Cell current discharging cell is positive.
 *  \return Returns series voltage drop from ideal, Vs.
 */
static double v_s(double rs, double icell) {
    
    return rs * icell;
}

/*! Compute instantaneous Vr
 *  \param r1 Diffusion model resistance (ohms).
 *  \param icell Cell current (Amps).
 *  Note: Cell current discharging cell is positive.
 *  \return Returns diffusion voltage drop from ideal, Vr.
 */
static double v_r(double r1, double icell) {
    
    return r1 * icell;
}

/*! Compute instantaneous Vm
 *  \param vm Hysteresis model voltage (volts).
 *  \param icell Cell current (Amps).
 *  Note: Cell current discharging cell is positive.
 *  \return Returns hysteresis voltage drop from ideal, Vr.
 */
static double v_m(double vm, double icell) {
    
    return (icell > 0) ? vm : vm * -1;
}


/***              Public Functions              ***/

/*! Perform LUT lookup of cell SOC based on cell open circuit voltage.
 *  \param voc Cell open circuit voltage, Volts.
 *  \return Function returns SOC from 0-1.
 *  Note: Function interpolates between LUT points to provide continuous output.
 */
double cell_compute_soc(double voc) {
    
    /* SOC based on OCV lookup table.
     * SOC as a function of open cell voltage is non=linear, a lookup table seems to
     * be the best way to map between these two values.  
     */
    /*const double t_offset = 3.00205;
    const double t_step = 0.01;
    const double t_gain = 1000;
    u_int16_t tbl[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,17,18,19,21,22,23,25,26,28,30,31,33,35,37,39,41,44,46,49,52,55,58,61,65,69,73,78,84,90,97,105,115,126,141,159,182,210,242,277,314,352,389,427,465,503,541,579,616,654,692,730,768,806,843,878,905,923,935,943,949,954,958,961,964,967,969,971,973,975,977,978,979,981,982,983,984,985,986,987,988,989,990,991,991,992,993,994,994,995,996,996,997,997,998,};*/
    // offset: 2.41498V, step size:0.01V
    const double t_offset = 2.415;
    const double t_step = 0.01;
    const double t_gain = 1000;
    u_int16_t tbl[] = {0,1,2,3,5,6,7,8,10,11,12,14,15,17,18,20,22,24,25,27,29,32,34,36,39,42,45,48,51,55,59,63,68,73,79,87,95,105,117,133,153,181,216,258,303,349,396,443,490,537,584,632,679,726,773,820,865,902,925,938,947,953,958,962,966,969,971,974,976,978,979,981,983,984,985,986,988,989,990,991,992,993,994,994,995,996,997,997,998,999,999,1000,};


    // number of elements in lut
    if (voc <= t_offset) {
        return 0.0;
    }
    // table size
    const unsigned int n = sizeof (tbl) / sizeof (tbl[0]);
    // compute index
    unsigned int index = (unsigned int) ((voc - t_offset) / t_step);
    // compute fractional index
    double f_index = ((voc - t_offset) / t_step) - index;
    // limit to valid table index, -1
    index = (index < n-2) ? index : n-2;
    f_index = (f_index < 1.0) ? f_index : 1.0;  // don't extrapolate beyond LUT limits
    // compute local slope
    const double delta = (tbl[index+1] - tbl[index]) / t_gain;
    return tbl[index] / t_gain + f_index * delta;
}

/*! Integrate Icell to find new cell SOC.
 *  \param c Pointer to battery cell struct with cell states/parameters.
 *  \param dt time elapsed since last integration (s).
 *  \return Function returns new SOC.
 */
double cell_integrate_i(struct batt_cell_s *c, double dt) {
    
    double delta_c = c->i * dt;
    if (delta_c < 0.0) {
        // battery is being charged
        delta_c *= c->eta_c;
    }
    // note: battery capacity is in A*h, delta_c is in A*s.
    const double new_soc = (c->capacity * c->soc - delta_c / 3600) / c->capacity;
    if (new_soc > 1.0) {
        return 1.0;
    }
    if (new_soc < 0.0) {
        return 0.0;
    }
    return new_soc;
}

/*! Compute cell OCV based on cell state and parameters.
 *  Note; Function should be called at regular intervals separated by time dt.
 *  This allows internal digital filters to operate properly.
 *  Function updates voc field based on computed value.
 *  \param c Pointer to battery cell struct with cell states/parameters.
 *  \param dt Delta t since function was last called. 
 */
void cell_compute_voc(struct batt_cell_s *c, double dt) {
    
    const double vs = v_s(c->rs, c->i);
    const double vr_inst = v_r(c->r1, c->i);
    const double vm_inst = v_m(c->vm, c->i);
    c->lpf_vr = lpf_s(dt, c->tau1, vr_inst, c->lpf_vr);
    c->lpf_vh = lpf_s(dt, c->taum, vm_inst, c->lpf_vh);
    c->voc = c->termv + vs + c->lpf_vr + c->lpf_vh;
}


