/* 
 * File:   beagle_bms.c
 * Author: martin
 *
 * Created on July 27, 2013, 1:26 AM
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <ncurses.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include "ltc_6804.h"
#include "ltc6804_util.h"
#include "ltc6804_regs.h"
#include "cell.h"


/***                Definitions                 ***/

// number of ICs in BMS
#define     IC_COUNT            2
// total cell count
#define     CELL_COUNT          (CELL_COUNT_PER_IC * IC_COUNT)

/***              Global Variables              ***/

/*! LTC6804 IC status
 */
struct ic_status_s {
    double vsoc;                    ///< sum of cells measurement, Volts
    double itmp;                    ///< IC die temperature, deg. C
    double va;                      ///< Analog supply voltage, Volts
    double vd;                      ///< Digital supply voltage, Volts
    u_int8_t rev;                   ///< IC revision
    bool muxfail;                   ///< IC MUX fail flag
    bool thsd;                      ///< IC thermal shutdown flag
    struct {
        enum ltc6804_PEC_e cv;      ///< Cell Voltage register groups PEC status
        enum ltc6804_PEC_e auxa;    ///< Aux A register group PEC status
        enum ltc6804_PEC_e stat;    ///< Status register group PEC status
    } pec;
};

/*! Parameters for each RT task thread
 */
struct thread_param_s {
    pthread_t                       thread;                 ///< thread data
    const char *                    task_name;              ///< a task name for debugging
    struct ltc6804_cell_voltage_s   *sVoltage;              ///< |sampled cell voltages
    struct ltc6804_aux_reg_a_s      *sAux;                  ///< |sampled GPIO
    struct ltc6804_stat_reg_a_s     *sStatA;                ///< |LTC6804 status A struct
    struct ltc6804_stat_reg_b_s     *sStatB;                ///< |LTC6804 status B struct
    sem_t                           *sample_sem;            ///< ^ sampled data semaphore
    struct batt_cell_s              *cells;                 ///< |BMS cell data
    struct ic_status_s              *status;                ///< |BMS IC status struct
    sem_t                           *bms_sem;               ///< ^ BMS data semaphore
    sem_t                           *sampleComplete_sem;    ///< sampling of cell data is complete
    sem_t                           *updateComplete_sem;    ///< update of BMS data is complete
    sem_t                           *term_sem;              ///< indicates thread should terminate
};

/***              Private Functions             ***/

/*! Convert raw ADC V_GPIO value to current.
 *  \param v V_GPIO ADC value.
 *  \return Analog current sensed by ADC/Hall sensor (Amps).
 */
double convertI(u_int16_t v) {
    
    const double gain = 100.0 / 1.5;            // 100A per 1.5V
    //const double v_offset = 2.5225;             // Vref = 2.5V
    const double v_offset = 2.53;             // Vref = 2.5V
    double v_adc = ltc6804util_convertV(v);
    return (v_adc - v_offset) * gain; 
}

/*! Initialize and configure a pthread with FIFO scheduling class and priority p
 *  \param attr Pointer to un-initialized thread attributes.
 *  \param p Thread RT priority level
 *  \return Function returns 0 on success, -1 on error.
 */
static int initFifoThread(pthread_attr_t *attr, int p)
{
    struct sched_param param;
    int rv;
    
    rv = pthread_attr_init(attr);
    if (rv != 0) {
        perror("pthread_attr_init");
        return -1;
    }
    rv = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    if (rv != 0) {
        perror("pthread_attr_setinheritsched");
        return -1;
    }
    rv = pthread_attr_setschedpolicy(attr, SCHED_FIFO);
    if (rv != 0) {
        perror("pthread_attr_setschedpolicy");
        return -1;
    }
    param.sched_priority = p;
    rv = pthread_attr_setschedparam(attr, &param);
    if (rv != 0) {
        perror("pthread_attr_setschedparam");
        return -1;
    }
    return 0;
}

/*! Sample cell data - RT task 1
 *  \param p Pointer to thread parameters for task.
 *  \return function calls pthread_exit() and returns p.
 */
static void *sampleCellData(void *p) {
    
    struct thread_param_s *t = p;
    timer_t sampleTimer;                // interval timer for sampling ADC
    sigset_t timerSig;
    sigemptyset(&timerSig);
    sigaddset(&timerSig, SIGRTMIN + 1);
    struct sigevent sev = {
        .sigev_notify = SIGEV_SIGNAL,   // send signal to process
        .sigev_signo = SIGRTMIN + 1,    // signal number
    };
    // 10ms time interval for sampling ADC at 100Hz
    struct itimerspec timInt = {
        .it_value = {
            .tv_sec = 0,
            .tv_nsec = 10000000,
        },
        .it_interval = {
            .tv_sec = 0,
            .tv_nsec = 10000000,
        },
    };
    printf("task %s starting.\n", t->task_name);
    int s = timer_create(CLOCK_MONOTONIC, &sev, &sampleTimer);
    if (s < 0) {
        perror("timer_create");
        pthread_exit(p);
    }
    // arm interval timer
    timer_settime(&sampleTimer, 0, &timInt, NULL);
    
    while (1) {
        struct ltc6804_cell_voltage_s   cv[IC_COUNT];       // cell voltage register data
        struct ltc6804_stat_reg_a_s     sa[IC_COUNT];       // status register A group
        struct ltc6804_stat_reg_b_s     sb[IC_COUNT];       // status register B group
        struct ltc6804_aux_reg_a_s      xa[IC_COUNT];       // aux register A group
        
        // check if termination semaphore is posted
        int s;
        sem_getvalue(t->term_sem, &s);
        if (s > 0) {
            break;
        }
        // wait for signal from sampling timer
        siginfo_t info;
        sigwaitinfo(&timerSig, &info);
        //syslog(LOG_USER, "sampling ADC");
        int addr;
        for (addr=0; addr<IC_COUNT; addr++) {
            // for each '6804 IC, assuming continuous address space
            // start all cell voltage groups and GPIO 1&2 ADC conversions
            ltc6804_startCellGpioVoltageConversion(addr, ADC_MD_NORM_gc);
        }
        // wait for ADC to complete conversion
        usleep(2000);
        // start status group ADC conversion
        for (addr=0; addr<IC_COUNT; addr++) {
            ltc6804_startStatVoltageConversion(addr, ADC_MD_NORM_gc, CHST_ALL_gc);
        }
        // read cell voltage & aux registers
        for (addr=0; addr<IC_COUNT; addr++) {
            ltc6804_readCellVoltageGroups(addr, &cv[addr]);
            ltc6804_readAuxGroupA(addr, &xa[addr]);
        }
        /* we assume here that the time it takes to read the 4 cell voltage
         * register groups and the singe auxA register group is sufficient for
         * the status ADC conversion to complete. This conversion takes 1.6ms
         * in the "NORMAL" ADC mode.
         */
        for (addr=0; addr<IC_COUNT; addr++) {
            ltc6804_readStatA(addr, &sa[addr]);
            ltc6804_readStatB(addr, &sb[addr]);
        }
        // obtain a lock on shared sampled data
        sem_wait(t->sample_sem);
        // copy cell voltages
        for (addr=0; addr<IC_COUNT; addr++) {
            // for each IC
            // copy sampled data to shared structs
            memcpy(&t->sVoltage[addr], &cv[addr], sizeof cv[0]);    // cell voltage
            memcpy(&t->sAux[addr], &xa[addr], sizeof xa[0]);        // GPIO voltage
            memcpy(&t->sStatA[addr], &sa[addr], sizeof sa[0]);      // Status A
            memcpy(&t->sStatB[addr], &sb[addr], sizeof sb[0]);      // Status B
        }
        sem_post(t->sample_sem);
        // post sample complete semaphore
        sem_post(t->sampleComplete_sem);
    }
    printf("task %s exiting.\n", t->task_name);
    pthread_exit(p);
}

/*! Update BMS state estimates - RT task 2
 *  \param p Pointer to thread parameters for task.
 *  \return function calls pthread_exit() and returns p.
 */
static void *updateBmsState(void *p) {
    
    struct thread_param_s *t = p;
    
    printf("task %s starting.\n", t->task_name);
    
    while (1) {
        // check if termination semaphore is posted
        int s;
        sem_getvalue(t->term_sem, &s);
        if (s > 0) {
            break;
        }
        // wait for updated samples
        sem_wait(t->sampleComplete_sem);
        // update BMS data
        sem_wait(t->sample_sem);
        sem_wait(t->bms_sem);
        /* Convert and copy cell voltages and current from sampled data to
         * BMS data. Verify PEC for all voltage registers and for first AUX
         * register. This way samples are updated atomically and only if all
         * PECs are OK. If a PEC error is detected, cell state is not updated
         * for any cells. It seems better to use stale data for recursive state
         * estimation than to use erroneous data.
         * PEC states in BMS data are always updated, so even if cell state or
         * IC state is not it is possible to detect PEC errors using the BMS
         * state data.
         * PECs for IC status are checked per IC.
         */
        // check PECs
        int cellStatePec = PEC_OK;
        int ic;
        for (ic=0; ic<IC_COUNT; ic++) {
            t->status[ic].pec.auxa = t->sAux[ic].pecOk;
            t->status[ic].pec.cv = t->sVoltage[ic].pecOk;
            t->status[ic].pec.stat = t->sStatA[ic].pecOk & t->sStatB[ic].pecOk;
            //cellStatePec &= t->status[ic].pec.auxa;
            //cellStatePec &= t->status[ic].pec.cv;
        }
        // copy and decode cell state data
        if (cellStatePec == PEC_OK) {
            int ic_cell;        // cells within ic
            int c = 0;          // total cells in BMS
            // current sensing is only on first ic, and is the same for all cells.
            double i_sense = convertI(t->sAux[0].g1v);
            for (ic=0; ic<IC_COUNT; ic++) {
                // for each IC
                // copy and convert cell voltage data from this IC
                for (ic_cell=0; ic_cell<CELL_COUNT_PER_IC; ic_cell++) {
                    // for each cell in ic
                    t->cells[c].termv = ltc6804util_convertV(t->sVoltage[ic].cell[ic_cell]); // terminal voltage
                    t->cells[c].i = i_sense;
                    t->cells[c].flags = t->sStatB[ic].cell_flags[ic_cell];
                    c++;
                }
            }
        }
        // copy and decode IC state data
        for (ic=0; ic<IC_COUNT; ic++) {
            if (t->sStatA[ic].pecOk == PEC_OK && t->sStatB[ic].pecOk == PEC_OK) {
                // for each IC
                // copy and convert IC status
                t->status[ic].vsoc = ltc6804util_convertSoc(t->sStatA[ic].soc);
                t->status[ic].va = ltc6804util_convertVa_Vd(t->sStatA[ic].va);
                t->status[ic].vd = ltc6804util_convertVa_Vd(t->sStatB[ic].vd);
                t->status[ic].itmp = ltc6804util_convertItmp(t->sStatA[ic].itmp);
                t->status[ic].rev = t->sStatB[ic].rev;
                t->status[ic].muxfail = t->sStatB[ic].muxfail;
                t->status[ic].thsd = t->sStatB[ic].thsd;
            }
        }
        sem_post(t->sample_sem);

        // cell state calculations
        int i;
        for (i=0; i<CELL_COUNT; i++) {
            cell_compute_voc(&t->cells[i], 1/100.0);
            // SOC estimate based on cell voltage
            double soc_v = cell_compute_soc(t->cells[i].voc);
            // SOC estimate based on integrating cell current
            double soc_i = cell_integrate_i(&t->cells[i], 1/100.0);
            // fixed gain fusion of SOC estimates:
            const double a = 0.999;
            t->cells[i].soc = a * soc_i + (1-a) * soc_v;
        }
        
        // cell state update complete
        sem_post(t->bms_sem);
        sem_post(t->updateComplete_sem);
    }
    printf("task %s exiting.\n", t->task_name);
    pthread_exit(p);
}

static void logBmsData(struct thread_param_s *t) {
    
    WINDOW *ic_win[IC_COUNT];       // IC data windows
    WINDOW *cell_leg_win[IC_COUNT]; // cell legend (one for each cell row, IC)
    WINDOW *cell_win[CELL_COUNT];   // cell data windows
    WINDOW *status_win;             // status display
    bool logToFile = false;
    FILE *logFile = 0;
    
    // initialize ncurses interface
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    // fix (ncurses??) display issue by getting dummy input here.
    int ch = getch();
    int row, col, i;
    // initialize IC windows
    for (i=0; i<IC_COUNT; i++) {
        ic_win[i] = newwin(10, 32, 0, i*32);
        box(ic_win[i], 0 , 0);
    }
    // initialize cell legend
    for (i=0; i<IC_COUNT; i++) {
        cell_leg_win[i] = newwin(9, 10, i*9 + 10, 0);
        box(cell_leg_win[i], 0 , 0);
    }
    // initialize cell windows
    i = 0;
    for (row=0; row<IC_COUNT; row++) {
        for (col=0; col<CELL_COUNT_PER_IC; col++) {
            cell_win[i] = newwin(9, 8, row*9 + 10, col*8 + 10);
            box(cell_win[i], 0 , 0);
            i++;
        }
    }
    // initialize status window
    status_win = newwin(3, 90, 10 + IC_COUNT * 9 , 0);
    // print cell legend
    for (i=0; i<IC_COUNT; i++) {
        mvwprintw(cell_leg_win[i], 1, 1, "BMS Cell");
        mvwprintw(cell_leg_win[i], 2, 1, "SOC   %");
        mvwprintw(cell_leg_win[i], 3, 1, "V_oc  V");
        mvwprintw(cell_leg_win[i], 4, 1, "V_trm V");
        mvwprintw(cell_leg_win[i], 5, 1, "I     A");
        mvwprintw(cell_leg_win[i], 6, 1, "flags");
        mvwprintw(cell_leg_win[i], 7, 1, "PEC");
        wrefresh(cell_leg_win[i]);
    }
    // print commands
    mvwprintw(status_win, 1, 1, "(l)og  (s)top log  (q)uit");
    wrefresh(status_win);
    // print updates to windows as necessary
    unsigned int iter = 0;
    int loop = true;
    while (loop) {
        
        // wait for update task to complete
        sem_wait(t->updateComplete_sem);
        // log every 25th update (4Hz)
        if (++iter % 25 != 0) {
            continue;
        }
        // wait for access to shared data
        sem_wait(t->bms_sem);
        // log IC status data
        for (i=0; i<IC_COUNT; i++) {
            mvwprintw(ic_win[i], 1, 1, "IC revision %u address %u", t->status[i].rev, (unsigned int)i);
            mvwprintw(ic_win[i], 2, 1, "Sum of Cells (SOC): %#.2f V", t->status[i].vsoc);
            mvwprintw(ic_win[i], 3, 1, "Die Temperature:    %#.2f C", t->status[i].itmp);
            mvwprintw(ic_win[i], 4, 1, "Analog Supply (Va): %#.3f V", t->status[i].va);
            mvwprintw(ic_win[i], 5, 1, "Digital Supply (Vd):%#.3f V", t->status[i].vd);
            mvwprintw(ic_win[i], 6, 1, "Mux Test:           %s", (t->status[i].muxfail == true) ? "Failed" : "Passed");
            mvwprintw(ic_win[i], 7, 1, "Thermal Shutdown:   %s", (t->status[i].thsd == true) ? "yes" : "no ");
            mvwprintw(ic_win[i], 8, 1, "PEC:                %s", (t->status[i].pec.stat == PEC_OK) ? "Ok" : "Fail");
        }
        // log cell status data
        for (i=0; i<CELL_COUNT; i++) {
            mvwprintw(cell_win[i], 1, 1, "%u", (unsigned int) i+1);
            mvwprintw(cell_win[i], 2, 1, "%#.1f%%", t->cells[i].soc * 100);
            mvwprintw(cell_win[i], 3, 1, "%#.3f", t->cells[i].voc);
            mvwprintw(cell_win[i], 4, 1, "%#.3f", t->cells[i].termv);
            mvwprintw(cell_win[i], 5, 1, "%#.1f", t->cells[i].i);
            mvwprintw(cell_win[i], 6, 1, "%c%c", 
                      (t->cells[i].flags & CF_OVERVOLTAGE) ? 'o':'-',
                      (t->cells[i].flags & CF_UNDERVOLTAGE) ? 'u':'-');
            mvwprintw(cell_win[i], 7, 1, "%s",
                      (t->status[i/CELL_COUNT_PER_IC].pec.cv == PEC_OK) ? "Ok" : "Fail");
        }
        // log to file
        if (logToFile) {
            time_t rawtime;
            struct tm *timeinfo;
            char timeString[128];
            time(&rawtime);
            timeinfo = localtime(&rawtime);
            strftime(timeString, sizeof timeString,"%F-%H:%M:%S", timeinfo);
            fprintf(logFile, "%s,", timeString);
            for (i=0; i<IC_COUNT; i++) {
                fprintf(logFile, "%#.2f,", t->status[i].vsoc);
                fprintf(logFile, "%#.2f,", t->status[i].itmp);
                fprintf(logFile, "%#.3f,", t->status[i].va);
                fprintf(logFile, "%#.3f,", t->status[i].vd);
                fprintf(logFile, "%s,", (t->status[i].pec.stat == PEC_OK) ? "Ok" : "Fail");
            }
            for (i=0; i<CELL_COUNT; i++) {
                fprintf(logFile, "%u,", (unsigned int) i+1);
                fprintf(logFile, "%#.3f,", t->cells[i].soc);
                fprintf(logFile, "%#.3f,", t->cells[i].voc);
                fprintf(logFile, "%#.3f,", t->cells[i].termv);
                fprintf(logFile, "%#.1f,", t->cells[i].i);
                fprintf(logFile, "%c%c,", 
                          (t->cells[i].flags & CF_OVERVOLTAGE) ? 'o':'-',
                          (t->cells[i].flags & CF_UNDERVOLTAGE) ? 'u':'-');
                fprintf(logFile, "%s,", (t->status[i/CELL_COUNT_PER_IC].pec.cv == PEC_OK) ? "Ok" : "Fail");
            }
            fprintf(logFile, "\n");
        }
        sem_post(t->bms_sem);
        // refresh ncurses windows
        for (i=0; i<IC_COUNT; i++) {
            box(ic_win[i], 0 , 0);
            wrefresh(ic_win[i]);
        }
        for (i=0; i<CELL_COUNT; i++) {
            box(cell_win[i], 0 , 0);
            wrefresh(cell_win[i]);
        }
        // get user input
        ch = getch();
        switch (tolower(ch)) {
            case 'q':
                // quit program
                loop = false;
                // stop logging data
                if (logToFile) {
                    logToFile = false;
                    fclose(logFile);
                }
                mvwprintw(status_win, 2, 1, "exiting...");
                wrefresh(status_win);
                break;
            case 'l': {
                // begin logging data to file
                if (logToFile) {
                    // already logging
                    break;
                }
                time_t rawtime;
                struct tm *timeinfo;
                char filename [128];
                time(&rawtime);
                timeinfo = localtime(&rawtime);
                strftime(filename, sizeof filename,"log_%F-%H-%M-%S.txt", timeinfo);
                // open logging stream file
                logFile = fopen(filename, "w");
                if (logFile == 0) {
                    perror("fopen");
                    break;
                }
                logToFile = true;
                mvwprintw(status_win, 2, 1, "Started logging to file: %s.", filename);
                wrefresh(status_win);
            } break;
            case 's':
                // stop logging data
                if (!logToFile) {
                    // not currently logging
                    break;
                }
                logToFile = false;
                fclose(logFile);
                mvwprintw(status_win, 2, 1, "Logging Complete.\n");
                wrefresh(status_win);
                break;
            default:
                break;
        }
    }
    
    // end ncurses session, free windows
    endwin();
    
    for (i=0; i<IC_COUNT; i++) {
        delwin(ic_win[i]);
        delwin(cell_leg_win[i]);
    }
    for (i=0; i<CELL_COUNT; i++) {
        delwin(cell_win[i]);
    }
    delwin(status_win);
}

/*! Initialize and start RT tasks.
 *  \param t1 Pointer to uninitialized parameters for task 1.
 *  \param t2 Pointer to uninitialized parameters for task 2.
 *  Note: Task 1 is assigned SCHED_FIFO min priority + 2,
 *  task 2 is assigned SCHED_FIFO min priority + 1.
 *  \return Function returns 0 on success, -1 on failure.
 */
static int startRtTasks(struct thread_param_s *t1, struct thread_param_s *t2) {
    
    // shared state variables
    static struct ltc6804_cell_voltage_s sv[IC_COUNT];
    static struct ltc6804_aux_reg_a_s xa[IC_COUNT];
    static struct ltc6804_stat_reg_a_s sStata[IC_COUNT];
    static struct ltc6804_stat_reg_b_s sStatb[IC_COUNT];
    static struct ic_status_s stat[IC_COUNT];
    static struct batt_cell_s cells[CELL_COUNT];
    // semaphores
    sem_t sampleSem;
    sem_t bmsSem;
    sem_t sampleCompleteSem;
    sem_t updateCompleteSem;
    sem_t termSem;
    
    // thread data for each RT task, task 1
    t1->task_name = "sample data";
    t1->sVoltage = sv;
    t1->sAux = xa;
    t1->sStatA = sStata;
    t1->sStatB = sStatb;
    t1->status = stat;
    t1->sample_sem = &sampleSem;
    t1->cells = cells;
    t1->bms_sem = &bmsSem;
    t1->sampleComplete_sem = &sampleCompleteSem;
    t1->updateComplete_sem = &updateCompleteSem;
    t1->term_sem = &termSem;
    // task 2
    memcpy(t2, t1, sizeof (struct thread_param_s));
    t2->task_name = "update BMS state";
    // initialize cell parameters
    int i;
    for (i=0; i<CELL_COUNT; i++) {
        cells[i].soc = 0.99;         // initial SOC
        cells[i].eta_c = 0.995;
        cells[i].rs = 2.5E-3;
        cells[i].r1 = 2.5E-3;
        cells[i].tau1 = 240;
        cells[i].vm = 7E-3;
        cells[i].taum = 40;
        cells[i].capacity = 40.0;
    }
            
    // initialize semaphores
    int s = sem_init(&sampleSem, 0, 1);
    if (s != 0) {
        perror("sem_init");
        return -1;
    }
    s = sem_init(&bmsSem, 0, 1);
    if (s != 0) {
        perror("sem_init");
        return -1;
    }
    s = sem_init(&sampleCompleteSem, 0, 0);
    if (s != 0) {
        perror("sem_init");
        return -1;
    }
    s = sem_init(&updateCompleteSem, 0, 0);
    if (s != 0) {
        perror("sem_init");
        return -1;
    }
    s = sem_init(&termSem, 0, 0);
    if (s != 0) {
        perror("sem_init");
        return -1;
    }
    // initialize threads
    int min = sched_get_priority_min(SCHED_FIFO);
    pthread_attr_t t1Attr;
    pthread_attr_t t2Attr;
    initFifoThread(&t1Attr, min + 2);   // task 1 has higher priority
    initFifoThread(&t2Attr, min + 1);
    
    // create threads
    s = pthread_create(&t1->thread, &t1Attr, sampleCellData, t1);
    if (s != 0) {
        perror("pthread_create");
        return -1;
    }
    s = pthread_create(&t2->thread, &t2Attr, updateBmsState, t2);
    if (s != 0) {
        perror("pthread_create");
        return -1;
    }
    return 0;
}

/***              Public Functions              ***/

/*
 * 
 */
int main(int argc, char** argv) {
    
    // thread data for each RT task
    static struct thread_param_s sampleTask;
    static struct thread_param_s updateTask;
    
    // initialize interface and ICs
    int i;
    for (i=0; i<IC_COUNT; i++) {
        ltc6804_init(i, "/dev/spidev4.0", 800000);
    }
    
    /* Block ADC sampling timer signal.
     * This needs to be blocked for the process in general here so only the task
     * (thread) waiting on that signal receives the signal. This must be
     * accomplished before the task threads are created.
     */
    sigset_t tMask;
    sigemptyset(&tMask);
    sigaddset(&tMask, SIGRTMIN + 1);    // the signal chosen for the timer
    sigprocmask(SIG_BLOCK, &tMask, NULL);
    
    int s = startRtTasks(&sampleTask, &updateTask);
    if (s < 0) {
        puts("failed to create threads for RT tasks.");
        return (EXIT_FAILURE);
    }
    
    logBmsData(&sampleTask);
    
    // terminate RT task threads
    sem_post(sampleTask.term_sem);
    pthread_join(sampleTask.thread, NULL);
    pthread_join(updateTask.thread, NULL);
    ltc6804_close();
    puts("main() exiting.");
    
    return (EXIT_SUCCESS);
}

