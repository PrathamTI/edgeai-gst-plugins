/*
 * GStreamer TI TVM Inference Plugin
 * Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 * GStreamer element for TVM inference on TI processors with C7x DSP acceleration
 * Replicates tvm_inference_client functionality as GStreamer element
 */

#ifndef __GST_TI_TVM_H__
#define __GST_TI_TVM_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

/* Element type macros */
#define GST_TYPE_TI_TVM \
  (gst_ti_tvm_get_type())
#define GST_TI_TVM(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_TI_TVM,GstTiTvm))
#define GST_TI_TVM_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_TI_TVM,GstTiTvmClass))
#define GST_IS_TI_TVM(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_TI_TVM))
#define GST_IS_TI_TVM_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_TI_TVM))

typedef struct _GstTiTvm      GstTiTvm;
typedef struct _GstTiTvmClass GstTiTvmClass;

/* TVM inference performance data */
struct TiTvmPerformanceData {
    gint64 first_run_time;        /* First run latency (includes init) */
    gint64 *inference_times;      /* Array of inference times (excluding first run) */
    gdouble avg_time;             /* Average inference time */
    gdouble min_time;             /* Minimum inference time */
    gdouble max_time;             /* Maximum inference time */
    gdouble fps;                  /* Frames per second */
};

/* GStreamer TI TVM element structure */
struct _GstTiTvm
{
    GstBaseTransform element;

    /* Properties */
    gchar *model_path;            /* Path to TVM artifacts directory */
    gint iterations;              /* Number of inference iterations */
    gboolean benchmark;           /* Enable performance benchmarking */

    /* TVM runtime state */
    gboolean tvm_initialized;     /* TVM runtime initialization status */
    void *graph_executor;         /* TVM graph executor handle */
    void *set_input_func;         /* TVM set_input function */
    void *run_func;               /* TVM run function */
    void *get_output_func;        /* TVM get_output function */

    /* Input/output data */
    void *final_output;           /* Final inference output buffer */
    gsize output_num_floats;      /* Dynamic output size determined at inference time */

    /* Performance tracking */
    struct TiTvmPerformanceData perf_data;

    /* Execution state */
    gboolean inference_completed; /* Whether inference has run */
    gint current_iteration;       /* Current iteration number */
};

struct _GstTiTvmClass
{
    GstBaseTransformClass parent_class;
};

/* Function declarations */
GType gst_ti_tvm_get_type (void);

/* Element property IDs */
enum
{
    PROP_0,
    PROP_MODEL_PATH,
    PROP_ITERATIONS,
    PROP_BENCHMARK
};

/* Default values */
#define DEFAULT_MODEL_PATH ""
#define DEFAULT_ITERATIONS 1
#define DEFAULT_BENCHMARK TRUE

G_END_DECLS

#endif /* __GST_TI_TVM_H__ */