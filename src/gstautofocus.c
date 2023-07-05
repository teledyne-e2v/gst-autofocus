/*
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * 
 * Teledyne e2V
 * Copyright (C) 2022 Nicolas
 * Copyright (C) 2023 Loic Chevallier <Teledyne e2V>
 * 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the
 * GNU Lesser General Public License Version 2.1 (the "LGPL"), in
 * which case the following provisions apply instead of the ones
 * mentioned above:
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * SECTION:element-autofocus
 *
 * FIXME:Describe autofocus here.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v -m fakesrc ! autofocus ! fakesink silent=TRUE
 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <gst/gst.h>
#include <unistd.h>
#include <stdio.h>

#include "gstautofocus.h"
#include "i2c_control.h"

GST_DEBUG_CATEGORY_STATIC(gst_autofocus_debug);
#define GST_CAT_DEFAULT gst_autofocus_debug

/* Filter signals and args */
enum
{
    /* FILL ME */
    LAST_SIGNAL
};

enum
{
    PROP_0,
    PROP_STRATEGY,
    PROP_AUTOFOCUS_STATUS,
    PROP_STEP_SMALL,
    PROP_STEP_BIG,
    PROP_PDA_MIN,
    PROP_PDA_MAX,
    PROP_DEC_MAX,
    PROP_X,
    PROP_Y,
    PROP_WIDTH,
    PROP_HEIGHT,
    PROP_OFFSET,
    PROP_CONTINUOUS,
    PROP_CONTINUOUS_UPDATE_INTERVAL,
    PROP_CONTINUOUS_TIMEOUT,
    PROP_CONTINUOUS_THRESHOLD,
    PROP_LISTEN,
    PROP_AUTOFOCUS_LOST,
    PROP_SHARPNESS,
    PROP_CALIBRATING,
    PROP_DEBUG_LOG,
    PROP_DEBUG_LEVEL,
    PROP_PDA_HOLD_CMD,
    PROP_BENCHMARK_EXPECTED_SHARPNESS,
    PROP_BENCHMARK_ITERATIONS,
    PROP_BENCHMARK_MIN_EXPECTED_SHARPNESS,
    PROP_SHARPNESS_CALCULATION
};
void printHelp(void);
void *autofocusHandler(void *autofocus);

I2CDevice device;
I2CDevice devicepda;
int bus;

ROI roi;
AutofocusConf conf;

int listen = 1;
int frameCount = 0;

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE("sink",
                                                                   GST_PAD_SINK,
                                                                   GST_PAD_ALWAYS,
                                                                   GST_STATIC_CAPS("ANY"));

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE("src",
                                                                  GST_PAD_SRC,
                                                                  GST_PAD_ALWAYS,
                                                                  GST_STATIC_CAPS("ANY"));

#define gst_autofocus_parent_class parent_class
G_DEFINE_TYPE(Gstautofocus, gst_autofocus, GST_TYPE_ELEMENT)

static void gst_autofocus_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec);
static void gst_autofocus_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec);

static GstFlowReturn gst_autofocus_chain(GstPad *pad, GstObject *parent, GstBuffer *buf);

static void gst_autofocus_finalize(void);//GObject *object);

#define TYPE_AUTOFOCUS_STATUS (autofocus_status_get_type())
static GType
autofocus_status_get_type(void)
{
    static GType autofocus_status = 0;

    if (!autofocus_status)
    {
        static const GEnumValue status[] =
            {
                {PENDING, "Pending", "pending"},
                {IN_PROGRESS, "In progress", "in_progress"},
                {COMPLETED, "Completed", "completed"},
                {0, NULL, NULL}
            };

        autofocus_status = g_enum_register_static("AutofocusStatus", status);
    }

    return autofocus_status;
}

#define TYPE_DEBUG_LEVEL (debug_level_get_type())
static GType
debug_level_get_type(void)
{
    static GType debug_level = 0;

    if (!debug_level)
    {
        static const GEnumValue status[] =
            {
                {NONE, "None", "none"},
                {MINIMAL, "Minimal", "minimal"},
                {FULL, "Full", "full"},
                {0, NULL, NULL}
            };

        debug_level = g_enum_register_static("DebugLevel", status);
    }

    return debug_level;
}

void printHelp(void)
{
    g_print("Help:\n");
    g_print("\ta: start autofocus\n");
    g_print("\ts: change autofocus strategy\n");
}

/**
 * @brief Prevent the ROI from protuding from the image
 */
static void checkRoi(void)
{
    // Prevent the ROI from being to close to the very end of the frame as it migth crash when calculating the sharpness
    if (roi.x > 1916)
    {
        roi.x = 1916;
    }

    if (roi.y > 1076)
    {
        roi.y = 1076;
    }

    // Prevent the ROI from going outsides the bounds of the image
    if (roi.x + roi.width >= 1920)
    {
        roi.width -= ((roi.x + roi.width) - 1920);
    }

    if (roi.y + roi.height > 1080)
    {
        roi.height -= ((roi.y + roi.height) - 1080);
    }
}

void *autofocusHandler(void *autofocus)
{
    Gstautofocus *focus = (Gstautofocus *)autofocus;

    if (focus != NULL)
    {
        while (listen)
        {
            char input;
	    int res;
            res = scanf(" %c", &input);
	    if(res!=1)
	    {
		g_print("Scanf failed, check input ");
	    }
            if (input == 'a' && focus->autofocusStatus == COMPLETED)
            {
                focus->autofocusStatus = PENDING;
            }
            else if (input == 's' && focus->autofocusStatus == COMPLETED)
            {
                int newStrat;
                g_print("Choose an other autofocus strategy: ");
                res = scanf(" %d", &newStrat);
		if(res!=1)
	    	{
			g_print("Scanf failed, check input ");
	    	}
                if (newStrat < 0 || newStrat > 1)
                {
                    g_print("\tError: unknown strategy\n");
                }
                else
                {
                    focus->strategy = newStrat;
                    g_print("\tChanging autofocus strategy\n");
                }
            }
            else if (input == 'c')
            {
                focus->calibrating = TRUE;
                frameCount = 0;
                g_print("Calibrating autofocus...\n");
            }
            else
            {
                g_print("Unknown option or autofocus in progress\n");
                printHelp();
            }
        }
    }
    else
    {
        g_print("Error: autofocus is null\n");
    }

    pthread_exit(NULL);
}

/* GObject vmethod implementations */

/* initialize the autofocus's class */
static void gst_autofocus_class_init(GstautofocusClass *klass)
{
    GObjectClass *gobject_class;
    GstElementClass *gstelement_class;

    gobject_class = (GObjectClass *)klass;
    gstelement_class = (GstElementClass *)klass;

    gobject_class->set_property = gst_autofocus_set_property;
    gobject_class->get_property = gst_autofocus_get_property;
    gobject_class->finalize     = gst_autofocus_finalize;

    g_object_class_install_property(gobject_class, PROP_STRATEGY,
                                    g_param_spec_int("strategy", "Strategy",
                                                     "Set which algorithm is used to do the autofocus\n\t- 0 is the naive algorimth\n\t- 1 is the two pass algorimth",
                                                     0, 4, 1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_X,
                                    g_param_spec_int("x", "X", "The top left X coordinates of the ROI",
                                                     0, 1920, 0, G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_BENCHMARK_EXPECTED_SHARPNESS,
                                    g_param_spec_int("benchmark_expected_sharpness", "Benchmark_expected_sharpness", "The maximum sharpness possible in the configuration",
                                                     1, 1000000, 1, G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_BENCHMARK_MIN_EXPECTED_SHARPNESS,
                                    g_param_spec_int("benchmark_min_expected_sharpness", "Benchmark_min_expected_sharpness", "The minimum sharpness possible in the configuration",
                                                     1, 1000000, 1, G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_BENCHMARK_ITERATIONS,
                                    g_param_spec_int("benchmark_iterations", "Benchmark_iterations", "The number of iterations to do the benchmark",
                                                     1, 50000, 1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_Y,
                                    g_param_spec_int("y", "Y", "The top left Y coordinates of the ROI",
                                                     0, 1080, 0, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_WIDTH,
                                    g_param_spec_int("width", "Width", "The width of the ROI",
                                                     0, 1920, 1920, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_HEIGHT,
                                    g_param_spec_int("height", "Height", "The height of the ROI",
                                                     0, 1080, 1080, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_STEP_SMALL,
                                    g_param_spec_int("step_small", "Step_small", "The step of the PDA for the naive algorithm",
                                                     1, 700, 8, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_STEP_BIG,
                                    g_param_spec_int("step_big", "Step_big", "The step of the PDA for the two pass algorithm",
                                                     1, 700, 68, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_PDA_MIN,
                                    g_param_spec_int("pda_min", "pda_min", "The minimal PDA value used for the autofocus algorithm",
                                                     -90, 750, -90, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_PDA_MAX,
                                    g_param_spec_int("pda_max", "pda_max", "The maximal PDA value used for the autofocus algorithm",
                                                     0, 750, 750, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_DEC_MAX,
                                    g_param_spec_int("dec", "Dec", "The number of consecutive blurrier frames before stopping the autofocus",
                                                     0, 20, 3, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_AUTOFOCUS_STATUS,
                                    g_param_spec_enum("autofocusStatus", "AutofocusStatus", "The state of the autofocus:\n\tPENDING: the autofocus is about to start\n\tIN_PROGRESS: The autofocus is running\n\tCOMPLETED: The autofocus is done",
                                                      TYPE_AUTOFOCUS_STATUS, COMPLETED, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_OFFSET,
                                    g_param_spec_int("offset", "Offset", "The frame offset between a pda command and the arrival of the corresponding frame in the plugin",
                                                     0, 100, 4, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CONTINUOUS_UPDATE_INTERVAL,
                                    g_param_spec_int("continuous_update_interval", "update", "How often should the sharness be calculated",
                                                     1, 120, 30, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CONTINUOUS_TIMEOUT,
                                    g_param_spec_int("continuous_timeout", "timeout", "The response time in frame",
                                                     1, 100, 4, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CONTINUOUS_THRESHOLD,
                                    g_param_spec_float("continuous_threshold", "threshold", "The threshold to determine if the image is blurrier and if the autofocus should be relaunched",
                                                       1.0f, 100.0f, 25.0f, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CONTINUOUS,
                                    g_param_spec_boolean("continuous", "Continuous",
                                                         "How many times should the sharpness calculated be under the threshold before relaunching the autofocus algorthim\nThis parameter has no effect if the parameter continuous is set to false",
                                                         FALSE, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_LISTEN,
                                    g_param_spec_boolean("listen", "Listen", "Listen for user input in the terminal.",
                                                         TRUE, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_AUTOFOCUS_LOST,
                                    g_param_spec_boolean("focus_lost", "Focus lost", "Whether or not the focus has been lost",
                                                         TRUE, G_PARAM_READABLE));

    g_object_class_install_property(gobject_class, PROP_SHARPNESS,
                                    g_param_spec_long("sharpness", "Sharpness", "The sharpness of the frame",
                                                      0, G_MAXINT64, 0, G_PARAM_READABLE));

    g_object_class_install_property(gobject_class, PROP_CALIBRATING,
                                    g_param_spec_boolean("calibrating", "Calibrating", "Whether or not the plugin is calculating the response time",
                                                         FALSE, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_DEBUG_LOG,
                                    g_param_spec_string("debug", "Debug", "Hold debug information about the last run of the autofocus",
                                                        "\0", G_PARAM_READABLE));

    g_object_class_install_property(gobject_class, PROP_DEBUG_LEVEL,
                                    g_param_spec_enum("debug_level", "Debug level", "The debugging level:\n\tnone(0): nothing is logged\n\tminimal(1): Only log the step, pda range and best focus found\n\tfull(2): Add on top of the minimal level information about each step of the algorithm",
                                                        TYPE_DEBUG_LEVEL, MINIMAL, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_PDA_HOLD_CMD,
                                    g_param_spec_int("pda_hold_cmd", "pda_hold_cmd", "The number of frame between each command sent",
                                                     0, 1024, 0, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_SHARPNESS_CALCULATION,
                                    g_param_spec_boolean("sharpness_calculation", "Sharpness_calculation",
                                                         "Will calculate the sharpness each frames",
                                                         FALSE, G_PARAM_READWRITE));

    gst_element_class_set_details_simple(gstelement_class,
                                         "autofocus",
                                         "FIXME:Generic",
                                         "Autofocus of snappy2M module",
                                         "Esisar-PI2022 <<user@hostname.org>>");

    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&src_factory));
    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&sink_factory));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void gst_autofocus_init(Gstautofocus *autofocus)
{
    autofocus->sinkpad = gst_pad_new_from_static_template(&sink_factory, "sink");
    gst_pad_set_chain_function(autofocus->sinkpad,
                               GST_DEBUG_FUNCPTR(gst_autofocus_chain));
    GST_PAD_SET_PROXY_CAPS(autofocus->sinkpad);
    gst_element_add_pad(GST_ELEMENT(autofocus), autofocus->sinkpad);

    autofocus->srcpad = gst_pad_new_from_static_template(&src_factory, "src");
    GST_PAD_SET_PROXY_CAPS(autofocus->srcpad);
    gst_element_add_pad(GST_ELEMENT(autofocus), autofocus->srcpad);

    autofocus->autofocusStatus = COMPLETED;
    autofocus->strategy = TWO_PHASES;

    autofocus->listen = TRUE;

    autofocus->continuous = FALSE;
    autofocus->continuousUpdateInterval = 30;
    autofocus->continuousTimeout = 4;
    autofocus->continuousThreshold = 25.0f;

    autofocus->calibrating = FALSE;

    autofocus->autofocusLost = FALSE;

    autofocus->sharpness = 0;

    autofocus->debugInfo = NULL;
    autofocus->debugLvl  = MINIMAL;

    autofocus->benchmark_expected_sharpness=1;
    autofocus->benchmark_min_expected_sharpness=1;
    autofocus->benchmark_iterations=50;
    autofocus->pdaHoldCmd = 0;
    autofocus->sharpnessCalculation = false;
    roi.x = 0;
    roi.y = 0;
    roi.width = 1920;
    roi.height = 1080;

    conf.pdaMin = -90;
    conf.pdaMax = 750;
    conf.pdaSmallStep = 8;
    conf.pdaBigStep = 63;
    conf.maxDec = 3;
    conf.offset = 4;
    conf.phase = PHASE_1;
    conf.debugLvl = MINIMAL;

    i2c_err=i2cInit(&device, &devicepda, &bus);
    if(!i2c_err)
    {
    pthread_t thread;

    int rc;
    if ((rc = pthread_create(&thread, NULL, autofocusHandler, (void *)autofocus)))
    {
        g_print("Error: unable to create thread, %d\n", rc);
        exit(-1);
    }
}
}

static void gst_autofocus_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec)
{
    Gstautofocus *autofocus = GST_AUTOFOCUS(object);

    switch (prop_id)
    {
    case PROP_STRATEGY:
        autofocus->strategy = g_value_get_int(value);
        break;
    case PROP_AUTOFOCUS_STATUS:
    {
        AutofocusStatus tmp = g_value_get_enum(value);

        // Prevent the autofocus from being restarted while it is in progress
        if (autofocus->autofocusStatus == COMPLETED && tmp == PENDING)
        {
            autofocus->autofocusStatus = tmp;
        }
        break;
    }
    case PROP_STEP_SMALL:
        conf.pdaSmallStep = g_value_get_int(value);
        break;
    case PROP_STEP_BIG:
        conf.pdaBigStep = g_value_get_int(value);
        break;
    case PROP_PDA_MIN:
        conf.pdaMin = g_value_get_int(value);
        break;
    case PROP_PDA_MAX:
        conf.pdaMax = g_value_get_int(value);
        break;
    case PROP_DEC_MAX:
        conf.maxDec = g_value_get_int(value);
        break;
    case PROP_X:
        roi.x = g_value_get_int(value);
        checkRoi();
        break;
    case PROP_Y:
        roi.y = g_value_get_int(value);
        checkRoi();
        break;
    case PROP_WIDTH:
        roi.width = g_value_get_int(value);
        checkRoi();
        break;
    case PROP_HEIGHT:
        roi.height = g_value_get_int(value);
        checkRoi();
        break;
    case PROP_OFFSET:
        conf.offset = g_value_get_int(value);
        break;
    case PROP_CONTINUOUS:
        autofocus->continuous = g_value_get_boolean(value);
        break;
    case PROP_SHARPNESS_CALCULATION:
        autofocus->sharpnessCalculation = g_value_get_boolean(value);
        break;
    case PROP_CONTINUOUS_UPDATE_INTERVAL:
        autofocus->continuousUpdateInterval = g_value_get_int(value);
        break;
    case PROP_CONTINUOUS_TIMEOUT:
        autofocus->continuousTimeout = g_value_get_int(value);
        break;
    case PROP_CONTINUOUS_THRESHOLD:
        autofocus->continuousThreshold = g_value_get_float(value);
        break;
    case PROP_LISTEN:
        autofocus->listen = g_value_get_boolean(value);
        listen = autofocus->listen;
        break;
    case PROP_SHARPNESS:
        autofocus->sharpness = g_value_get_long(value);
        break;
    case PROP_BENCHMARK_EXPECTED_SHARPNESS:
    	autofocus->benchmark_expected_sharpness=g_value_get_int(value);
    	break;
    case PROP_BENCHMARK_MIN_EXPECTED_SHARPNESS:
    	autofocus->benchmark_min_expected_sharpness=g_value_get_int(value);
    	break;
    case PROP_BENCHMARK_ITERATIONS:
	autofocus->benchmark_iterations=g_value_get_int(value);
    	break;
    case PROP_CALIBRATING:
    {
        autofocus->calibrating = g_value_get_boolean(value);

        if (autofocus->calibrating == TRUE)
        {
            frameCount = 0;
            g_print("Calibrating autofocus...\n");
        }

        break;
    }
    case PROP_DEBUG_LEVEL:
        autofocus->debugLvl = g_value_get_enum(value);
        conf.debugLvl = autofocus->debugLvl;
        break;
    case PROP_PDA_HOLD_CMD:
        autofocus->pdaHoldCmd = g_value_get_int(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_autofocus_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec)
{
    Gstautofocus *autofocus = GST_AUTOFOCUS(object);

    switch (prop_id)
    {
    case PROP_STRATEGY:
        g_value_set_int(value, autofocus->strategy);
        break;
    case PROP_AUTOFOCUS_STATUS:
        g_value_set_enum(value, autofocus->autofocusStatus);
        break;
    case PROP_STEP_SMALL:
        g_value_set_int(value, conf.pdaSmallStep);
        break;
    case PROP_STEP_BIG:
        g_value_set_int(value, conf.pdaBigStep);
        break;
    case PROP_PDA_MIN:
        g_value_set_int(value, conf.pdaMin);
        break;
    case PROP_PDA_MAX:
        g_value_set_int(value, conf.pdaMax);
        break;
    case PROP_DEC_MAX:
        g_value_set_int(value, conf.maxDec);
        break;
    case PROP_X:
        g_value_set_int(value, roi.x);
        break;
    case PROP_Y:
        g_value_set_int(value, roi.y);
        break;
    case PROP_WIDTH:
        g_value_set_int(value, roi.width);
        break;
    case PROP_HEIGHT:
        g_value_set_int(value, roi.height);
        break;
    case PROP_OFFSET:
        g_value_set_int(value, conf.offset);
        break;
    case PROP_CONTINUOUS:
        g_value_set_boolean(value, autofocus->continuous);
        break;
    case PROP_SHARPNESS_CALCULATION:
        g_value_set_boolean(value, autofocus->sharpnessCalculation);
        break;
    case PROP_CONTINUOUS_UPDATE_INTERVAL:
        g_value_set_int(value, autofocus->continuousUpdateInterval);
        break;
    case PROP_CONTINUOUS_TIMEOUT:
        g_value_set_int(value, autofocus->continuousTimeout);
        break;
    case PROP_CONTINUOUS_THRESHOLD:
        g_value_set_float(value, autofocus->continuousThreshold);
        break;
    case PROP_LISTEN:
        g_value_set_boolean(value, autofocus->listen);
        break;
    case PROP_AUTOFOCUS_LOST:
        g_value_set_boolean(value, autofocus->autofocusLost);
        break;
    case PROP_SHARPNESS:
        g_value_set_long(value, autofocus->sharpness);
        break;
    case PROP_CALIBRATING:
        g_value_set_boolean(value, autofocus->calibrating);
        break;
    case PROP_DEBUG_LOG:
        g_value_set_string(value, autofocus->debugInfo);
        break;
    case PROP_DEBUG_LEVEL:
        g_value_set_enum(value, autofocus->debugLvl);
        break;
    case PROP_PDA_HOLD_CMD:
        g_value_set_int(value, autofocus->pdaHoldCmd);
        break;
    case PROP_BENCHMARK_EXPECTED_SHARPNESS:
    	g_value_set_int(value, autofocus->benchmark_expected_sharpness);
    	break;
    case PROP_BENCHMARK_MIN_EXPECTED_SHARPNESS:
    	g_value_set_int(value, autofocus->benchmark_min_expected_sharpness);
    	break;
    case PROP_BENCHMARK_ITERATIONS:
    	g_value_set_int(value, autofocus->benchmark_iterations);
    	break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_autofocus_chain(GstPad *pad, GstObject *parent, GstBuffer *buf)
{
	
    Gstautofocus *autofocus = GST_AUTOFOCUS(parent);
    GstCaps *caps = gst_pad_get_current_caps(pad);
    GstStructure *s = gst_caps_get_structure(caps, 0);
    if (!i2c_err)
    {
        gint width = 0, height = 0;

        static struct timeval start, end;

        static long sharpness = -1;
        static int nbFrame = 0;
        static int lostFocusCount = 0;
        static short int buffering = 8;
        static int waitRemaining = 0;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);
        if (roi.height > height)
        {
            roi.height = height - roi.y;
        }

        if (roi.width > width)
        {
            roi.width = width - roi.x;
        }
    
	

    // g_print("%ld\n", autofocus->sharpness);


    if (autofocus->calibrating == TRUE && autofocus->autofocusStatus == COMPLETED)
    {
        static long int prevSharpness = 0;
        static int offset = 0;
        
        if (frameCount == 0)
        {
            write_VdacPda(devicepda, bus, 0); // return to PDA 0
        }
        else if (frameCount == 5) // Wait for 5 frame, then send a command
        {
            write_VdacPda(devicepda, bus, 500); // go to PDA 500
        }
        else if (frameCount >= 5)
        {
            double relativeDiff = ((prevSharpness - autofocus->sharpness) / (double)autofocus->sharpness) * 100.0f;
            
            offset++;
            
            if (relativeDiff >= 25.0f || relativeDiff <= -25.0f)
            {
                conf.offset = offset - autofocus->pdaHoldCmd;
                if (conf.offset < 0)
                    conf.offset = 0;
                
                autofocus->calibrating = FALSE;

                g_print("Calibration complete: the new offset is %d\n", conf.offset);
                prevSharpness = 0;
                offset = 0;
            }
        }
        if (frameCount >= 60)
        {
            g_print("The calibration is too long, aborting\n");
            autofocus->calibrating = FALSE;
            prevSharpness = 0;
            offset = 0;
        }
        
        prevSharpness = autofocus->sharpness;

        frameCount++;
    }
    else if (autofocus->autofocusStatus == PENDING) // Get the time at the start of autofocus
    {
        resetDebugInfo();
        g_print("Starting the autofocus\n\n");
        
        resetAutofocus(autofocus->strategy, &conf, &devicepda, bus);

        waitRemaining = autofocus->pdaHoldCmd;

        autofocus->autofocusStatus = (waitRemaining == 0) ? IN_PROGRESS : WAITING;
        
        gettimeofday(&start, NULL);
    }
    else if (autofocus->autofocusStatus == WAITING)
    {
        waitRemaining--;

        if ((waitRemaining) <= 0)
            autofocus->autofocusStatus = IN_PROGRESS;
    }
    else if (autofocus->autofocusStatus == IN_PROGRESS)
    {

        if (autofocus->strategy == NAIVE)
            sharpness = naiveAutofocus(&devicepda, bus, autofocus->sharpness);
        else if (autofocus->strategy == TWO_PHASES)
            sharpness = twoPhaseAutofocus(&devicepda, bus, autofocus->sharpness);
	else if (autofocus->strategy == WEIGHTED_MEAN)
	{
	    int important_PDAs[]= {-80,140,360,580,800};
	    //int important_PDAs[] = {-80,-50,0,40,80,160,240,320,400,480,560,640,720};
            sharpness = weightedMeanAutofocus(&devicepda, bus, autofocus->sharpness,important_PDAs,5,conf.offset);
	}
	else if(autofocus->strategy == GAUSSIAN_PREDICTION)
	{
  	    int important_PDAs[] = {-80,-50,0,40,80,160,240,320,400,480,560,640,720,798};
            sharpness = gaussianPredictionAutofocus(&devicepda, bus, autofocus->sharpness,important_PDAs,14,conf.offset);
	}
	else if(autofocus->strategy == BENCHMARK)
	{
	    int important_PDAs[] = {-80,-50,0,40,80,160,240,320,400,480,560,640,720,798};
	    float results[4]; // {accuracy,min,max,std}
	    sharpness = autofocusBenchmark(&devicepda, bus, autofocus->sharpness,important_PDAs,14,conf.offset, autofocus->benchmark_iterations,autofocus->benchmark_expected_sharpness,autofocus->benchmark_min_expected_sharpness,results);
	
		if(sharpness>0)
		{
	    printf("accuracy; min; max; std; \n %f; %f; %f; %f\n", results[0],results[1],results[2],results[3]);
		}
	}
        else
        {
            g_print("Error: Unknown autofocus strategy!\n");

            sharpness = -1;
            autofocus->autofocusStatus = COMPLETED;
        }

        if (sharpness != -1)
        {
		double elapsed;
		char *tmp;
            size_t logLen = 0;
            gettimeofday(&end, NULL); // Get the time when the autofocus ended

            elapsed =
                ((end.tv_sec * 1000000 + end.tv_usec) -
                 (start.tv_sec * 1000000 + start.tv_usec)) /
                1000000.0f;

            logAutofocusTime(elapsed);
            
            
            tmp = getDebugInfo(&logLen);

            if (logLen != 0)
            {
                autofocus->debugInfo = (char*)realloc(autofocus->debugInfo, sizeof(char) * (logLen + 1));
                autofocus->debugInfo = strncpy(autofocus->debugInfo, tmp, logLen);
                autofocus->debugInfo[logLen] = '\0';
                free(tmp);
            }

            autofocus->autofocusStatus = COMPLETED;
            
            buffering = conf.offset * 2; // Prevent the continuous autofocus from starting before the first sharp frame arrive
        }
        else if (autofocus->pdaHoldCmd > 0)
        {
            waitRemaining = autofocus->pdaHoldCmd;
            autofocus->autofocusStatus = WAITING;
        }
    }
    else if (sharpness != -1 && buffering == 0) // When the autofocus has finish check if the frame is still sharp after a little while
    {
        if (nbFrame >= autofocus->continuousUpdateInterval)
        {
            double relativeDiff = ((sharpness - autofocus->sharpness) / (double)autofocus->sharpness) * 100.0f;

            if (relativeDiff > autofocus->continuousThreshold || relativeDiff < -autofocus->continuousThreshold)
            {
                //g_print("Warning: focus has been lost (may be); %ld\n", autofocus->sharpness);

                autofocus->autofocusLost = TRUE;
                lostFocusCount++;
            }
            else
            {
                autofocus->autofocusLost = FALSE;
                lostFocusCount = 0;
            }

            if (lostFocusCount > autofocus->continuousTimeout && autofocus->continuous == TRUE)
            {
                resetAutofocus(autofocus->strategy, &conf, &devicepda, bus);
                autofocus->autofocusStatus = PENDING;
                lostFocusCount = 0;
                g_print("Trying to refocus the frame...\n");
            }

            nbFrame = 0;
        }

        nbFrame++;
    }
    else
    {
        buffering--;
    }
    }

    if(autofocus->autofocusStatus == IN_PROGRESS || autofocus->continuous == TRUE || autofocus->sharpnessCalculation)
{
    autofocus->sharpness = getSharpness(pad, buf, roi);
}
	
    /* just push out the incoming buffer */
    return gst_pad_push(autofocus->srcpad, buf);
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean autofocus_init(GstPlugin *autofocus)
{
    /* debug category for fltering log messages
     *
     * exchange the string 'Template autofocus' with your description
     */
    GST_DEBUG_CATEGORY_INIT(gst_autofocus_debug, "autofocus",
                            0, "Template autofocus");

    return gst_element_register(autofocus, "autofocus", GST_RANK_NONE,
                                GST_TYPE_AUTOFOCUS);
}

static void gst_autofocus_finalize(void)//GObject *object)
{
    disable_VdacPda(devicepda, bus);
    i2c_close(bus);
    g_print("Bus closed\n");
    freeDebugInfo();
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "myfirstautofocus"
#endif

/* gstreamer looks for this structure to register autofocuss
 *
 * exchange the string 'Template autofocus' with your autofocus description
 */
GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    autofocus,
    "Template autofocus",
    autofocus_init,
    VERSION,
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/")
