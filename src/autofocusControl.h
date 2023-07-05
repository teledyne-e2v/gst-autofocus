#pragma once

#include <gst/gst.h>
#include <stdbool.h>

#include "i2c_control.h"
#include "logger.h"

#define NB_THREADS 4

typedef enum
{
    NONE,
    MINIMAL,
    FULL
} DebugLevel;

typedef enum
{
    PHASE_1 = 1,
    PHASE_2
} AlgorithmPhase;

typedef enum
{
    NAIVE,
    TWO_PHASES,
    WEIGHTED_MEAN,
    GAUSSIAN_PREDICTION,
    BENCHMARK
} AutofocusStrategy;

typedef struct ROI
{
    int x, y;           // The ROI top left coordinates
    int width, height;  // The ROI size
} ROI;

typedef struct sharpnessParameters
{
    unsigned char *imgMat;  // The frame data
    ROI threadsROI;         // The ROI to be processed
    int width;              // The width of the whole frame 
    long int result;        // The sharpness value
    long int average;       // The average of the pixels on the ROI
} SharpnessParameters;


__attribute__((unused)) static int important_PDA_arg = 0;
__attribute__((unused)) static int waiting_frames = 0;
__attribute__((unused)) static int frame = 0;
__attribute__((unused)) static int proc = 0;
__attribute__((unused)) static long int sharpness_saved[100];

typedef struct autofocusConf
{
    DebugLevel debugLvl;
    AlgorithmPhase phase;
    AutofocusStrategy currentStrategy;
    int bestPdaValue;
    int pdaValue;
    int pdaMin;         // The pda minimum value
    int pdaMax;         // The pda maximum value 
    int pdaSmallStep;   // The pda step used for the naive autofocus technique
    int pdaBigStep;     // The pda step used for the first pass of the 2 phase autofocus technique
    int pdaStep;
    int maxDec;         // The maximum number of consecutive frame being less focused before stoping the autofocus  
    int offset;
    int tmpOffset;
} AutofocusConf;

__attribute__((unused)) static int calculated_sharpness;
__attribute__((unused)) static int iteration = 0;
__attribute__((unused)) static int sum_of_sharpness=0;
__attribute__((unused)) static int frame_between_autofocus=0;
__attribute__((unused)) static double bench_accuracy[1000];
/**
 * @brief Compute the sharpness on a section of the image
 * 
 * @param params The autofocusControl structure containing all required information
 * @return void* 
 */
void *unbiasedSharpnessMono(void *params);

/**
 * @brief Multi thread the sharpness computation of the frame
 * 
 * @param imgMat The frame data
 * @param width The width of the frame
 * @param roi The ROI where to compute the sharpness
 * @return long int The sharpness of the image normalized by the average of the pixels
 */
long int unbiasedSharpnessThread(guint8 *imgMat, int width, ROI roi);

/**
 * @brief Get the Sharpness from a frame
 * 
 * @param pad The gstreamer pad
 * @param buf The gstreamer buffer
 * @param roi The ROI where to compute the sharpness 
 * @return long int The sharpness value of the frame
 */
long int getSharpness(GstPad *pad, GstBuffer *buf, ROI roi);

void naivePDAStepHandler(I2CDevice *device, int bus, int dec, int nbIter);

/**
 * @brief A simple implementation of an autofocus algorithm
 * Cover the pda range specified by the conf while the frame is getting sharper with a given step
 * 
 * @param device The i2c device used to control the lens
 * @param bus    The i2c bus used to control the lens
 * @return long int return -1 while the algorithm is in progress, or the sharpness of the frame when the algorimth ended
 */
long int naiveAutofocus(I2CDevice *device, int bus, long int sharpness);

/**
 * @brief Autofocus on the ROI
 * Cover the whole pda range with a given step
 * Save the two best pda value
 * Use the naiveAutofocus with the range found earlier to find the sharpest frame
 * 
 * @param device The i2c device used to control the lens
 * @param bus    The i2c bus used to control the lens
 * @param debug  The string holding some debugging information about the autofocus
 * @return long int return -1 while the algorithm is in progress, or the sharpness of the frame when the algorimth ended
 */
long int twoPhaseAutofocus(I2CDevice *device, int bus, long int sharpness);


long int weightedMeanAutofocus(I2CDevice *device, int bus, long int sharpness,int* important_PDAs, int nb_important_PDAs, int latency);

long int autofocusBenchmark(I2CDevice *device, int bus,long int sharpness,int* important_PDAs, int nb_important_PDAs, int latency, int number_of_iterations,int expected,int min_expexted, float* results);


long int gaussianPredictionAutofocus(I2CDevice *device, int bus, long int sharpness,int* important_PDAs, int nb_important_PDAs, int latency);

/**
 * @brief Reset a given autofocus strategy and reconfigure it
 * 
 * @param strat  The autofocus algorithm to reset
 * @param conf   The configure to use
 * @param device The i2c device used to control the lens
 * @param bus    The i2c bus
 */
void resetAutofocus(AutofocusStrategy strat, AutofocusConf *conf, I2CDevice *device, int bus);

/**
 * @brief Check if the pda is in the allowed pda range
 * otherwise snap it back into the range
 * 
 * @param pda The pda value to check
 * @param pdaMin The lower bound of the range
 * @param pdaMax The upper  bound of the range
 */
void checkPDABounds(int *pda, int pdaMin, int pdaMax);

/**
 * @brief Empty the debug info log
 * 
 */
void resetDebugInfo(void);

/**
 * @brief Free the debug info from memory
 * 
 */
void freeDebugInfo(void);

/**
 * @brief Get the debug info of the last autofocus run;
 * 
 * @return char* 
 */
char *getDebugInfo(size_t *len);

/**
 * @brief Log information about the time taken to do the autofocus
 * 
 * @param time The time take by the autofocus
 */
void logAutofocusTime(double time);
