#include "autofocusControl.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>

#include "curve_fit.h"

static AutofocusConf currentConf;
void logAutofocusInfo(int nbIter, long int sharpness);
void goToPDA(I2CDevice *device, int bus, int pda);
int min_arr(long int * arr, long int len);
int weightedMean(long int *arr, int *arr_PDA,int nb_important_PDAs);
int arg_max(long int * arr, long int len);
bool decreasing(long int * arr, int nb_values);

int min(int a,int b);

List debugInfo = { NULL, 0 };

/**
 * @brief Log information about the current status of the autofocus
 * 
 * @param nbIter    The number of iteration already made by the algorithm
 * @param sharpness The sharpness of the current frame
 */
void logAutofocusInfo(int nbIter, long int sharpness)
{
    if (currentConf.debugLvl >= FULL)
    {
        char tmp[100];
        int effectivePda = currentConf.pdaValue - (currentConf.tmpOffset * currentConf.pdaStep);
        checkPDABounds(&effectivePda, currentConf.pdaMin, currentConf.pdaMax);
            
        sprintf(tmp, "%.2d, %ld, %d, %d\n", nbIter, (nbIter >= currentConf.offset) ? sharpness : -1, effectivePda, currentConf.pdaValue);
        insert(&debugInfo, tmp);
        g_print("%s", tmp);
    }
}

/**
 * @brief Send the specified command to the lens passing by zero
 * 
 * @param device The pda i2c device
 * @param bus The i2c bus
 * @param pda The pda command to be sent
 */
void goToPDA(I2CDevice *device, int bus, int pda)
{
    write_VdacPda(*device, bus, 0);

    if (pda != 0)   // don't send the same command twice
    {
        usleep(10000);
        write_VdacPda(*device, bus, currentConf.bestPdaValue); // Send the PDA command to get the sharpest image
    }
}

void checkPDABounds(int *pda, int pdaMin, int pdaMax)
{
    if (*pda < pdaMin)
        *pda = pdaMin;

    if (*pda > pdaMax)
        *pda = pdaMax;
}

void *unbiasedSharpnessMono(void *params)
{
    SharpnessParameters *parameters = (SharpnessParameters *)params;

    unsigned char *imgMat = parameters->imgMat;

    ROI threadRoi = parameters->threadsROI;

    int width = parameters->width;
    int endX = threadRoi.x + threadRoi.width;
    int endY = threadRoi.y + threadRoi.height;

    long int tmpRes = 0;
    long int tmpAverage = 0;

    for (int y = threadRoi.y; y < endY; y += 4)
    {
        for (int x = threadRoi.x; x < endX; x += 4)
        {
            int tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp8;

            int px0 = (y * width) + x;
            int px1 = px0 + width; //((y + 1) * width) + x;
            int px2 = px1 + width; //((y + 2) * width) + x;
            int px3 = px2 + width; //((y + 3) * width) + x;

            tmp1 = (imgMat[px0]     - imgMat[px1]);
            tmp2 = (imgMat[px0 + 1] - imgMat[px1 + 1]);
            tmp3 = (imgMat[px2 + 2] - imgMat[px3 + 2]);
            tmp4 = (imgMat[px2 + 3] - imgMat[px3 + 3]);
            tmp5 = (imgMat[px0 + 2] - imgMat[px0 + 3]);
            tmp6 = (imgMat[px1 + 2] - imgMat[px1 + 3]);
            tmp7 = (imgMat[px2]     - imgMat[px2 + 1]);
            tmp8 = (imgMat[px3]     - imgMat[px3 + 1]);

            tmpAverage += (imgMat[px0] * imgMat[px0]) + (imgMat[px0 + 1] * imgMat[px0 + 1]) + (imgMat[px0 + 2] * imgMat[px0 + 2]) + (imgMat[px0 + 3] * imgMat[px0 + 3]) +
                          (imgMat[px1] * imgMat[px1]) + (imgMat[px1 + 1] * imgMat[px1 + 1]) + (imgMat[px1 + 2] * imgMat[px1 + 2]) + (imgMat[px1 + 3] * imgMat[px1 + 3]) +
                          (imgMat[px2] * imgMat[px2]) + (imgMat[px2 + 1] * imgMat[px2 + 1]) + (imgMat[px2 + 2] * imgMat[px2 + 2]) + (imgMat[px2 + 3] * imgMat[px2 + 3]) +
                          (imgMat[px3] * imgMat[px3]) + (imgMat[px3 + 1] * imgMat[px3 + 1]) + (imgMat[px3 + 2] * imgMat[px3 + 2]) + (imgMat[px3 + 3] * imgMat[px3 + 3]);

            tmpRes += tmp1 * tmp1;
            tmpRes += tmp2 * tmp2;
            tmpRes += tmp3 * tmp3;
            tmpRes += tmp4 * tmp4;
            tmpRes += tmp5 * tmp5;
            tmpRes += tmp6 * tmp6;
            tmpRes += tmp7 * tmp7;
            tmpRes += tmp8 * tmp8;
        }
    }

    parameters->result = tmpRes;
    parameters->average = tmpAverage;

    pthread_exit(NULL);
}

long int unbiasedSharpnessThread(guint8 *imgMat, int width, ROI roi)
{
    long int finalResult = 0;
    long int finalAverage = 0;
	long int n;
    pthread_t threads[NB_THREADS];
    SharpnessParameters params[NB_THREADS];

    // Spread the computation of the sharpness on multiple threads
    for (int i = 0; i < NB_THREADS; i++)
    {
        params[i].threadsROI.width  = roi.width;
        params[i].threadsROI.height = roi.height / NB_THREADS;

        params[i].threadsROI.x = roi.x;
        params[i].threadsROI.y = roi.y + (params[i].threadsROI.height * i);
	
        params[i].imgMat  = imgMat;
        params[i].width   = width;
        params[i].result  = 0;
        params[i].average = 0;
	if(params[i].threadsROI.width + params[i].threadsROI.x > 1916)
	{
		params[i].threadsROI.width = 1916 - params[i].threadsROI.x;
	}

	if(params[i].threadsROI.height + params[i].threadsROI.y > 1076)
	{
		params[i].threadsROI.height = 1076 - params[i].threadsROI.y;
	}

        pthread_create(&threads[i], NULL, unbiasedSharpnessMono, (void *)&params[i]);
    }

    for (int i = 0; i < NB_THREADS; i++)
    {
        pthread_join(threads[i], NULL);

        finalResult += params[i].result;
        finalAverage += params[i].average;
    }

    n = roi.width * roi.height;
    finalAverage = finalAverage / n;

    return finalResult / finalAverage;
}

long int getSharpness(GstPad *pad, GstBuffer *buf, ROI roi)
{
 long int sharp;
    GstMapInfo map;
	GstCaps *caps;
gboolean res;
    gint width, height;
GstStructure *s;
    gst_buffer_map(buf, &map, GST_MAP_READ);

    caps = gst_pad_get_current_caps(pad);
    s = gst_caps_get_structure(caps, 0);

    

    // we need to get the final caps on the buffer to get the size
    res = gst_structure_get_int(s, "width", &width);
    res |= gst_structure_get_int(s, "height", &height);

    if (!res)
    {
        g_print("could not get snapshot dimension\n");
        exit(-1);
    }

     sharp = unbiasedSharpnessThread(map.data, width, roi);



    gst_buffer_unmap(buf, &map);

    return sharp;
}


int min_arr(long int * arr, long int len)
{
	long int min = arr[0];
	for(int i=1; i<len; i++)
	{
		if(min > arr[i])
		{
			min = arr[i];
		}
	}
	return min;
}



int arg_max(long int * arr, long int len)
{
	long int max = arr[0];
	int arg=0;
	for(int i=1; i<len; i++)
	{
		if(max < arr[i])
		{
			max = arr[i];
			arg = i;
		}
	}
	return arg;
}


bool decreasing(long int * arr, int nb_values)
{
	long int prec_value = arr[0];
	for(int i=1; i<nb_values; i++)
	{
		if(prec_value < arr[i])
		{
			return false;
		}
	}
	return true;
}



int weightedMean(long int *arr, int *arr_PDA,int nb_important_PDAs)
{
	long int weighted_mean = 0;
	long int sum_sharpness = 0;
	long int min = min_arr(arr,nb_important_PDAs);


	for(int i = 0; i < nb_important_PDAs; i++)
	{
		weighted_mean += arr_PDA[i] * (arr[i] - min);
		sum_sharpness += arr[i]- min;
	}

	weighted_mean = weighted_mean / sum_sharpness;

	return (int) weighted_mean;
}


int min(int a,int b)
{
if(a<b)
return a;
else
return b;
}


long int weightedMeanAutofocus(I2CDevice *device, int bus, long int sharpness,int* important_PDAs, int nb_important_PDAs, int latency)
{
	printf("entré1");
	long int res = -1;
	if(important_PDA_arg<nb_important_PDAs)
	{	
		write_VdacPda(*device, bus, important_PDAs[important_PDA_arg]);
		important_PDA_arg++;
		if(frame-latency >= 0)
		{
			sharpness_saved[frame-latency] = sharpness;
		}
		frame++;
	}
	else if(waiting_frames<latency)
	{
		if(frame-latency >= 0)
		{
			sharpness_saved[frame-latency] = sharpness;
		}
		waiting_frames++;
		frame++;
		goToPDA(device, bus, 0);
	}
	else
	{
		int weight;
		if(nb_important_PDAs>5){
		//int weighted_mean = weightedMean(sharpness_saved,important_PDAs,nb_important_PDAs);
		int arg = arg_max(sharpness_saved,nb_important_PDAs);
		int arg1 = min(nb_important_PDAs-arg,arg);
		int arg2 = min(arg1,5);
		//int arg3 = min(arg1,3);
		//int arg4 = min(arg1,2);
		//int weight1 = weightedMean(sharpness_saved + arg - arg1,important_PDAs + arg - arg1,2*arg1);
		weight = weightedMean(sharpness_saved + arg - arg2,important_PDAs + arg - arg2,2*arg2);
		//int weight3 = weightedMean(sharpness_saved + arg - arg3,important_PDAs + arg - arg3,2*arg3);
		//int weight4 = weightedMean(sharpness_saved + arg - arg4,important_PDAs + arg - arg4,2*arg4);}
		}
		else
		{
			weight = weightedMean(sharpness_saved,important_PDAs,nb_important_PDAs);
			printf("weight %d\n",weight);
		}

		res = sharpness;
		
		waiting_frames = 0;
		frame = 0;
		important_PDA_arg = 0;
		if(decreasing(sharpness_saved,4) && nb_important_PDAs >8)
		{
			write_VdacPda(*device, bus, -60);
			return res;
		}
		
		
		write_VdacPda(*device, bus, (int)weight);
		printf("out \n");

	}
	return res;
}


long int gaussianPredictionAutofocus(I2CDevice *device, int bus, long int sharpness,int* important_PDAs, int nb_important_PDAs, int latency)
{

	long int res = -1;
	if(important_PDA_arg<nb_important_PDAs)
	{	
		write_VdacPda(*device, bus, important_PDAs[important_PDA_arg]);

		important_PDA_arg++;
		if(frame-latency >= 0)
		{
			sharpness_saved[frame-latency] = sharpness;
		}
		frame++;


	}
	else if(waiting_frames<latency)
	{
		if(frame-latency >= 0)
		{
			sharpness_saved[frame-latency] = sharpness;
		}
		waiting_frames++;
		frame++;
		write_VdacPda(*device, bus,0);


	}
	else
	{

		double *pdas= (double*)malloc(sizeof(double)*nb_important_PDAs);
		double *sharp= (double*)malloc(sizeof(double)*nb_important_PDAs);
		/*if(decreasing(sharpness_saved,4))
		{
			write_VdacPda(*device, bus, -60);
			return res;
		}*/
		char tmp[1280];
		int pda;
		res = sharpness;
		tmp[0]=0;
		//resetDebugInfo();
		for(int i=0;i<nb_important_PDAs;i++)
		{
			if (currentConf.debugLvl >= MINIMAL)
			{		
			char tmp1[100];	
			pdas[i]=important_PDAs[i]/800.0;
			sharp[i]=(double)sharpness_saved[i];
			sprintf(tmp1,"PDA : %d sharpness : %ld\n",important_PDAs[i],(long int)sharp[i]);


			strcat(tmp,tmp1);

			}
		}

		insert(&debugInfo, tmp);
		pda = prediction(pdas,sharp,nb_important_PDAs);

		write_VdacPda(*device, bus, pda);//-(pda*100)/800);
		waiting_frames = 0;
		frame = 0;
		important_PDA_arg = 0;

	}
	return res;
}



long int naiveAutofocus(I2CDevice *device, int bus, long int sharpness)
{
    static long int maxSharpness  = 0;
    static long int prevSharpness = 0;

    static int dec = 0;

    static int nbIter = 0;

    long int res = -1;

    // While the pda value hasn't reach the maximum value allowed, the frames are still getting sharper,
    // there are still some frames to be check and we have waited for all required frames
    if (((((currentConf.pdaValue + currentConf.pdaStep) < currentConf.pdaMax) && (dec <= currentConf.maxDec))) || currentConf.tmpOffset > -1)
    {
        logAutofocusInfo(nbIter, sharpness);

        if (nbIter >= currentConf.offset)
        {
            if (sharpness > maxSharpness) // update the maximal sharpness value found
            {
                maxSharpness = sharpness;
                currentConf.bestPdaValue = currentConf.pdaValue - (currentConf.tmpOffset * currentConf.pdaStep);
                
                checkPDABounds(&(currentConf.bestPdaValue), currentConf.pdaMin, currentConf.pdaMax);
            }

            if (prevSharpness <= sharpness || currentConf.currentStrategy == TWO_PHASES) // reset the dec counter if the frame is sharper than the previous one
            {
                dec = 0;
            }
            else // increase the counter if the frame is blurier
            {
                dec++;
            }

            prevSharpness = sharpness;
        }

        if ((currentConf.pdaValue + currentConf.pdaStep) <= currentConf.pdaMax && dec <= currentConf.maxDec)
        {
            currentConf.pdaValue +=  currentConf.pdaStep;
            write_VdacPda(*device, bus, currentConf.pdaValue);
        }
        else
        {
            if (currentConf.pdaValue < currentConf.pdaMax && dec <= currentConf.maxDec)
            {
                currentConf.pdaValue = currentConf.pdaMax;
                write_VdacPda(*device, bus, currentConf.pdaValue);
            }           
            else if (nbIter > currentConf.offset)
            {
                currentConf.tmpOffset--;
            }
        }

        nbIter++;
    }
    else
    {
        goToPDA(device, bus, currentConf.bestPdaValue);

        if (currentConf.debugLvl >= MINIMAL)
        {
            char tmp[128];
            sprintf(tmp, "Phase %d completed after %.2d iterations\n\tThe best sharpness is %ld, at PDA %.3d\n", currentConf.phase, nbIter, maxSharpness, currentConf.bestPdaValue);
            g_print("%s", tmp);
            insert(&debugInfo, tmp);
        }

        res = maxSharpness;

        maxSharpness = 0;
        prevSharpness = 0;
        nbIter = 0;
        dec = 0;
    }

    return res;
}

long int twoPhaseAutofocus(I2CDevice *device, int bus, long int sharpness)
{
    static long int maxSharpness    = 0;
    static int bestPdaValue = 0;

    long int res = -1;

    // Phase one of the algorithm cover the allowed pda range with a big step
    if (currentConf.phase == PHASE_1)
    {
        if ((maxSharpness = naiveAutofocus(device, bus, sharpness)) != -1)
        {
            // Prime the second phase of the algorithm with the data found and the current configuration
            bestPdaValue = currentConf.bestPdaValue;

            currentConf.pdaMin = bestPdaValue - (currentConf.pdaBigStep * 3.0f/4.0f);
            currentConf.pdaMax = bestPdaValue + (currentConf.pdaBigStep * 3.0f/4.0f);
            
            checkPDABounds(&currentConf.pdaMin, currentConf.pdaMin, currentConf.pdaMax);
            checkPDABounds(&currentConf.pdaMax, currentConf.pdaMin, currentConf.pdaMax);
            
            currentConf.pdaStep = currentConf.pdaBigStep;

            currentConf.phase = PHASE_2;

            resetAutofocus(NAIVE, &currentConf, device, bus);
        }
    }
    else // Second phase of the algorithm, cover the new range with a small step
    {
        res = naiveAutofocus(device, bus, sharpness);

        // If the resultat of phase 2 are worse than the first one warn the user about it
        if (res != -1)
        {
            if (res < maxSharpness)
            {
                char tmp[] = "Warning: the best focus was found during the phase 1\n\tyou might need to recalibrate\n";
                g_print("%s", tmp);
                insert(&debugInfo, tmp);

                goToPDA(device, bus, bestPdaValue);
            }

            currentConf.phase = PHASE_1;

            maxSharpness = 0;
            bestPdaValue = 0;
        }
    }

    return res;
}

void resetAutofocus(AutofocusStrategy strat, AutofocusConf *conf, I2CDevice *device, int bus)
{
    if (conf == NULL)
    {
        g_print("Error: conf is null\n");
    }
    else
    {
        char tmp[128];
        currentConf.debugLvl = conf->debugLvl;
        currentConf.bestPdaValue = 0;
        currentConf.pdaValue = conf->pdaMin;
        currentConf.pdaMin = conf->pdaMin;
        currentConf.pdaMax = conf->pdaMax;
        currentConf.pdaSmallStep = conf->pdaSmallStep;
        currentConf.pdaBigStep   = conf->pdaBigStep;
        currentConf.maxDec = conf->maxDec;
        currentConf.offset = conf->offset;
        currentConf.tmpOffset = currentConf.offset;
        currentConf.pdaStep = (strat == NAIVE) ? conf->pdaSmallStep : conf->pdaBigStep;
        currentConf.phase = conf->phase;
        currentConf.currentStrategy = strat;
        
        
        
        if (currentConf.debugLvl >= MINIMAL)
        {
            
            sprintf(tmp, "Phase %d, PDA range [%.3d, %.3d], step = %.2d\n", conf->phase, conf->pdaMin, conf->pdaMax, currentConf.pdaStep);
            insert(&debugInfo, tmp);
            g_print("%s", tmp);
        }
        
        if (currentConf.debugLvl >= FULL)
        {
            sprintf(tmp, "Frame id, sharpness, sharpness pda, current pda\n");
            insert(&debugInfo, tmp);
            g_print("%s", tmp);
        }    
    }

    goToPDA(device, bus, currentConf.pdaMin);
}



long int autofocusBenchmark(I2CDevice *device, int bus,long int sharpness,int* important_PDAs, int nb_important_PDAs, int latency, int number_of_iterations,int expected, int min_expexted, float* results)
{
	
	currentConf.debugLvl = MINIMAL;
	if(iteration==0)
	{
		if(proc==0)
		{
			proc=1;
		
		}
		sum_of_sharpness=0;
	}
	if(frame_between_autofocus==5 && iteration<number_of_iterations) // this iteration is finished
	{
		frame_between_autofocus=0;
		sum_of_sharpness+=sharpness; // save the sharpness
		bench_accuracy[iteration]=((float)sharpness-min_expexted)/(expected-min_expexted)*100;
		iteration++;
		printf("sharpness : %ld\n",sharpness);

	}
	if(frame_between_autofocus==0)
	{
		printf("rly\n");
		calculated_sharpness = gaussianPredictionAutofocus(device,bus, sharpness, important_PDAs, nb_important_PDAs, latency);
		printf("ah oui \n");
	}

	if(calculated_sharpness != -1)
	{
		frame_between_autofocus++;
	}
	if(iteration<number_of_iterations)
	{
		return -1; //not finished yet
	}	
	else
	{
		iteration=0;
		
		float accuracy_mean = ((float)sum_of_sharpness/number_of_iterations-min_expexted)/(expected-min_expexted)*100;

		float min_sharpness=bench_accuracy[0];
		float max_sharpness=bench_accuracy[0];
		double std=0;
		for (int i=0; i<number_of_iterations; i++)
		{
			printf("bench : %f\n",bench_accuracy[i]);
			if(bench_accuracy[i]>max_sharpness)
			{
				max_sharpness=bench_accuracy[i];
			}
			else if(bench_accuracy[i]<min_sharpness)
			{
				min_sharpness=bench_accuracy[i];	
			}
			std+= (accuracy_mean - bench_accuracy[i])*(accuracy_mean - bench_accuracy[i]); // sum (X - mu)²
		}
	
		std=sqrt(std/number_of_iterations); // sqrt( sum (X - mu)² / N )
		results[0]=accuracy_mean ;
 		results[1]=(float)min_sharpness;
 		results[2]=(float)max_sharpness;
 		results[3]=(float)std;
		proc=0;
		printf("fin calcul\n");
		return sum_of_sharpness; // all measures are done
	}
	
}

void resetDebugInfo(void)
{
    invalidList(&debugInfo);
}

void freeDebugInfo(void)
{
    freeList(&debugInfo);
}

char *getDebugInfo(size_t *len)
{
    if (len != NULL)
        *len = debugInfo.len;
    
    return getListStr(&debugInfo);
}

void logAutofocusTime(double time)
{
    if (currentConf.debugLvl >= MINIMAL)
    {
        char tmp[25];
        sprintf(tmp, "\tTook %.3f seconds\n", time);
        insert(&debugInfo, tmp);
        g_print("%s", tmp);
    }
}
