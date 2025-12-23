/*
 * Copyright (c) [2025] Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free,
 * non-exclusive license under copyrights and patents it now or hereafter
 * owns or controls to make, have made, use, import, offer to sell and sell
 * ("Utilize") this software subject to the terms herein.  With respect to
 * the foregoing patent license, such license is granted  solely to the extent
 * that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include
 * this software, other than combinations with devices manufactured by or
 * for TI (“TI Devices”).  No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce
 * this license (including the above copyright notice and the disclaimer
 * and (if applicable) source code license limitations below) in the
 * documentation and/or other materials provided with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted
 * provided that the following conditions are met:
 *
 * *    No reverse engineering, decompilation, or disassembly of this software
 *      is permitted with respect to any software provided in binary form.
 *
 * *    Any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *    Nothing shall obligate TI to provide you with source code for the
 *      software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution
 * of the source code are permitted provided that the following conditions are met:
 *
 * *    Any redistribution and use of the source code, including any resulting
 *      derivative works, are licensed by TI for use only with TI Devices.
 *
 * *    Any redistribution and use of any object code compiled from the source
 *      code and any resulting derivative works, are licensed by TI for use
 *      only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its
 * suppliers may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <fcntl.h>
#include <linux/videodev2.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "gsttiovxfc.h"

#include "gst-libs/gst/tiovx/gsttiovx.h"
#include "gst-libs/gst/tiovx/gsttiovxallocator.h"
#include "gst-libs/gst/tiovx/gsttiovxsimo.h"
#include "gst-libs/gst/tiovx/gsttiovxpad.h"
#include "gst-libs/gst/tiovx/gsttiovxqueueableobject.h"
#include "gst-libs/gst/tiovx/gsttiovxutils.h"
#include "gsttiovxmultiscalerpad.h"

#include "tiovx_fc_module.h"
#include "ti_2a_wrapper.h"

static const char default_tiovx_sensor_name[] = "SENSOR_SONY_IMX219_RPI";
#define GST_TYPE_TIOVX_FC_TARGET (gst_tiovx_fc_target_get_type())

GST_DEBUG_CATEGORY_STATIC (gst_tiovxfc_debug);


static const gint min_num_exposures = 1;
static const gint default_num_exposures = 1;
static const gint max_num_exposures = 4;

static const gint min_format_msb = 1;
static const gint default_format_msb = 7;
static const gint max_format_msb = 16;

static const gboolean default_lines_interleaved = FALSE;
static const gboolean default_wdr_enabled = FALSE;
static const gboolean default_bypass_cac = TRUE;
static const gboolean default_bypass_dwb = TRUE;
static const gboolean default_bypass_nsf4 = FALSE;
static const guint default_ee_mode = TIVX_VPAC_VISS_EE_MODE_OFF;

static const int input_param_id = 3;
static const int output0_param_id = 12;


static const int default_ae_mode = ALGORITHMS_ISS_AE_AUTO;
static const int default_awb_mode = ALGORITHMS_ISS_AWB_AUTO;
static const guint default_ae_num_skip_frames = 0;
static const guint default_awb_num_skip_frames = 0;

static const guint exposure_ctrl_id = V4L2_CID_EXPOSURE;
static const guint analog_gain_ctrl_id = V4L2_CID_ANALOGUE_GAIN;


#define MIN_ROI_VALUE 0
#define MAX_ROI_VALUE G_MAXUINT32
#define DEFAULT_ROI_VALUE 0

/* Parameters for VPAV MSC Operations */
/* Target definition */
enum
{
  TIVX_TARGET_VPAC_FC_ID = 0,
};

static GType
gst_tiovx_fc_target_get_type (void)
{
  static GType target_type = 0;

  static const GEnumValue targets[] = {
    {TIVX_TARGET_VPAC_FC_ID, "VPAC FC", TIVX_TARGET_VPAC_FC},
  };

  if (!target_type) {
    target_type = g_enum_register_static ("GstTIOVXFCTarget", targets);
  }
  return target_type;
}

#define DEFAULT_TIOVX_FC_TARGET TIVX_TARGET_VPAC_FC_ID

/* Interpolation Method definition */
#define GST_TYPE_TIOVX_FC_INTERPOLATION_METHOD (gst_tiovx_fc_interpolation_method_get_type())
static GType
gst_tiovx_fc_interpolation_method_get_type (void)
{
  static GType interpolation_method_type = 0;

  static const GEnumValue interpolation_methods[] = {
    {VX_INTERPOLATION_BILINEAR, "Bilinear", "bilinear"},
    {VX_INTERPOLATION_NEAREST_NEIGHBOR, "Nearest Neighbor", "nearest-neighbor"},
    {TIVX_VPAC_MSC_INTERPOLATION_GAUSSIAN_32_PHASE, "Gaussian 32 Phase",
        "gaussian-32-phase"},
    {0, NULL, NULL},
  };

  if (!interpolation_method_type) {
    interpolation_method_type =
        g_enum_register_static ("GstTIOVXFlexConnectInterpolationMethod",
        interpolation_methods);
  }
  return interpolation_method_type;
}

#define DEFAULT_TIOVX_FC_INTERPOLATION_METHOD VX_INTERPOLATION_BILINEAR


#define ISS_IMX390_GAIN_TBL_SIZE                (71U)

/* TIOVX FC Pad */
#define GST_TYPE_TIOVX_FC_PAD (gst_tiovx_fc_pad_get_type())
G_DECLARE_FINAL_TYPE (GstTIOVXFCPad, gst_tiovx_fc_pad,
    GST_TIOVX, FC_PAD, GstTIOVXPad);


struct _GstTIOVXFCPad
{
  GstTIOVXPad base;

  gchar *videodev;

  sensor_config_get sensor_in_data;
  sensor_config_set sensor_out_data;

  TI_2A_wrapper ti_2a_wrapper;

  /* TI_2A_wrapper settings */
  gchar *dcc_2a_config_file;
  gboolean ae_mode;
  gboolean awb_mode;
  guint ae_num_skip_frames;
  guint awb_num_skip_frames;

  tivx_aewb_config_t aewb_config;
  uint8_t *dcc_2a_buf;
  uint32_t dcc_2a_buf_size;
};

GST_DEBUG_CATEGORY_STATIC (gst_tiovx_fc_pad_debug_category);

G_DEFINE_TYPE_WITH_CODE (GstTIOVXFCPad, gst_tiovx_fc_pad,
    GST_TYPE_TIOVX_PAD,
    GST_DEBUG_CATEGORY_INIT (gst_tiovx_fc_pad_debug_category,
        "tiovxfcpad", 0, "debug category for TIOVX FC pad class"));

struct _GstTIOVXFCPadClass
{
  GstTIOVXPadClass parent_class;
};

enum
{
  PROP_DEVICE = 1,
  PROP_DCC_2A_CONFIG_FILE,
  PROP_AE_MODE,
  PROP_AWB_MODE,
  PROP_AE_NUM_SKIP_FRAMES,
  PROP_AWB_NUM_SKIP_FRAMES,
  PROP_INTERPOLATION_METHOD,
  PROP_0,
  PROP_DCC_ISP_CONFIG_FILE,
  PROP_SENSOR_NAME,
  PROP_TARGET,
  PROP_NUM_EXPOSURES,
  PROP_LINE_INTERLEAVED,
  PROP_FORMAT_MSB,
  PROP_WDR_ENABLED,
  PROP_BYPASS_CAC,
  PROP_BYPASS_DWB,
  PROP_BYPASS_NSF4,
  PROP_EE_MODE,
};


/* Formats definition */
#if defined(SOC_AM62A)
/* AM62A/VPAC3L with VISS_OUT2: Supports U8, NV12 only */
#define TIOVX_FC_SUPPORTED_FORMATS_SRC "{NV12, GRAY8}"
#else
/* Full VPAC support: All standard formats */
#define TIOVX_FC_SUPPORTED_FORMATS_SRC "{NV12, GRAY8, GRAY16_LE, UYVY, YUYV}"
#endif
// #define TIOVX_FC_SUPPORTED_FORMATS_SINK "{ NV12, GRAY8, GRAY16_LE }"
#define TIOVX_FC_SUPPORTED_FORMATS_SINK "{ bggr, gbrg, grbg, rggb}"
#define TIOVX_FC_SUPPORTED_WIDTH "[1 , 8192]"
#define TIOVX_FC_SUPPORTED_HEIGHT "[1 , 8192]"
#define TIOVX_FC_SUPPORTED_CHANNELS "[1 , 10]"

/* Src caps */
#define TIOVX_FC_STATIC_CAPS_SRC                           \
  "video/x-raw, "                                           \
  "format = (string) " TIOVX_FC_SUPPORTED_FORMATS_SRC ", " \
  "width = " TIOVX_FC_SUPPORTED_WIDTH ", "                 \
  "height = " TIOVX_FC_SUPPORTED_HEIGHT                    \
  "; "                                                      \
  "video/x-raw(" GST_CAPS_FEATURE_BATCHED_MEMORY "), "      \
  "format = (string) " TIOVX_FC_SUPPORTED_FORMATS_SRC ", " \
  "width = " TIOVX_FC_SUPPORTED_WIDTH ", "                 \
  "height = " TIOVX_FC_SUPPORTED_HEIGHT ", "                \
  "num-channels = " TIOVX_FC_SUPPORTED_CHANNELS

#define TIOVX_FC_STATIC_CAPS_SINK                         \
  "video/x-bayer, "                                                    \
  "format = (string) " TIOVX_FC_SUPPORTED_FORMATS_SINK ", " \
  "width = " TIOVX_FC_SUPPORTED_WIDTH ", "                 \
  "height = " TIOVX_FC_SUPPORTED_HEIGHT

/* Pads definitions */
static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (TIOVX_FC_STATIC_CAPS_SINK)
    );

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src_%u",
    GST_PAD_SRC,
    GST_PAD_REQUEST,
    GST_STATIC_CAPS (TIOVX_FC_STATIC_CAPS_SRC)
    );  


struct _GstTIOVXFC
{
  GstTIOVXSimo element;
  gchar *dcc_fc_config_file;
  gchar *sensor_name;
  gint target_id;
  SensorObj sensor_obj;

  gint num_exposures;
  gboolean line_interleaved;
  gint format_msb;
  gint meta_height_before;
  gint meta_height_after;
  guint wdr_enabled;
  guint bypass_cac;
  guint bypass_dwb;
  guint bypass_nsf4;
  guint ee_mode;
  gint total_height;
  gint image_height;

  GstTIOVXAllocator *user_data_allocator;

  GstMemory *aewb_memory;
  GstMemory *h3a_stats_memory;

  TIOVXFCModuleObj fc_obj; 

  vx_reference input_references[MAX_NUM_CHANNELS];
  
  gint num_channels;
  guint postprocess_iter;

  gint interpolation_method;

};

static GType
gst_tiovx_fc_awb_mode_get_type (void)
{
  static GType awb_mode_type = 0;

  static const GEnumValue awb_modes[] = {
    {ALGORITHMS_ISS_AWB_AUTO, "AWB mode auto", "AWB_MODE_AUTO"},
    {ALGORITHMS_ISS_AWB_MANUAL, "AWB mode manual", "AWB_MODE_MANUAL"},
    {ALGORITHMS_ISS_AWB_DISABLED, "AWB mode disabled", "AWB_MODE_DISABLED"},
    {0, NULL, NULL},
  };

  if (!awb_mode_type) {
    awb_mode_type = g_enum_register_static ("GstTIOVXISPAWBModes", awb_modes);
  }
  return awb_mode_type;
}

static GType
gst_tiovx_fc_ae_mode_get_type (void)
{
  static GType ae_mode_type = 0;

  static const GEnumValue targets[] = {
    {ALGORITHMS_ISS_AE_AUTO, "AE mode auto", "AE_MODE_AUTO"},
    {ALGORITHMS_ISS_AE_MANUAL, "AE mode manual", "AE_MODE_MANUAL"},
    {ALGORITHMS_ISS_AE_DISABLED, "AE mode disabled", "AE_MODE_DISABLED"},
    {0, NULL, NULL},
  };

  if (!ae_mode_type) {
    ae_mode_type = g_enum_register_static ("GstTIOVXISPAEModes", targets);
  }
  return ae_mode_type;
}

GST_DEBUG_CATEGORY_STATIC (gst_tiovx_fc_debug);
#define GST_CAT_DEFAULT gst_tiovx_fc_debug

#define gst_tiovx_fc_parent_class parent_class
G_DEFINE_TYPE_WITH_CODE (GstTIOVXFC, gst_tiovx_fc,
    GST_TYPE_TIOVX_SIMO, GST_DEBUG_CATEGORY_INIT (gst_tiovx_fc_debug,
        "tiovxfc", 0, "debug category for the tiovxfc element"));

/* Function Prototypes */

static const gchar *
target_id_to_target_name (gint target_id);

static gboolean
gst_tiovx_fc_deinit_module (GstTIOVXSimo * simo);

static gboolean
gst_tiovx_fc_postprocess (GstTIOVXSimo * simo);

static gboolean 
gst_tiovx_fc_create_graph ( GstTIOVXSimo * simo, vx_context context, 
    vx_graph graph);

static GList *
gst_tiovx_fc_fixate_caps (GstTIOVXSimo * simo,
GstCaps * sink_caps, GList * src_caps_list);

static gboolean 
gst_tiovx_fc_get_node_info (GstTIOVXSimo * simo, vx_node * node,
    GstTIOVXPad * sink_pad, GList * src_pads, GList ** queueable_objects);

static gboolean 
gst_tiovx_fc_configure_module (GstTIOVXSimo * simo);

static void
gst_tivox_fc_compute_src_dimension (GstTIOVXSimo * simo,
    const GValue * dimension, GValue * out_value, guint roi_len);

static void
gst_tivox_fc_compute_sink_dimension (GstTIOVXSimo * simo,
    const GValue * dimension, GValue * out_value, guint roi_len);

static GstCaps *
gst_tiovx_fc_get_sink_caps (GstTIOVXSimo * simo,
    GstCaps * filter, GList * src_caps_list, GList *src_pads);

static GstCaps *
gst_tiovx_fc_get_src_caps (GstTIOVXSimo * simo,
    GstCaps * filter, GstCaps * sink_caps, GstTIOVXPad *src_pad_tiovx);

static gboolean 
gst_tiovx_fc_init_module (GstTIOVXSimo * simo,
    vx_context context, GstTIOVXPad * sink_pad, GList * src_pads, GstCaps * sink_caps,
    GList * src_caps_list, guint num_channels);

static void
gst_tiovx_fc_set_property(GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);

static void
gst_tiovx_fc_get_property(GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static void
gst_tiovx_fc_finalize (GObject * obj);

static int32_t
get_imx219_ae_dyn_params (IssAeDynamicParams * p_ae_dynPrms);

static void
gst_tiovx_fc_map_2A_values (GstTIOVXFC * self, int exposure_time,
    int analog_gain, gint32 * exposure_time_mapped, gint32 * analog_gain_mapped);

static void
gst_tiovx_fc_pad_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);

static void
gst_tiovx_fc_pad_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static void gst_tiovx_fc_pad_finalize (GObject * obj);


static void
gst_tiovx_fc_pad_class_init (GstTIOVXFCPadClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);

  gobject_class->set_property =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_pad_set_property);
  gobject_class->get_property =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_pad_get_property);
  gobject_class->finalize = 
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_pad_finalize);

  g_object_class_install_property (gobject_class, PROP_DEVICE,
      g_param_spec_string ("device", "Device",
          "Device location, e.g, /dev/v4l-subdev1."
          "Required by the user to use the sensor IOCTL support",
          NULL,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_DCC_2A_CONFIG_FILE,
      g_param_spec_string ("dcc-2a-file", "DCC AE/AWB File",
          "TIOVX DCC tuning binary file for the given image sensor.",
          NULL,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_AE_MODE,
      g_param_spec_enum ("ae-mode", "Auto exposure mode",
          "Flag to set if the auto exposure algorithm mode.",
          gst_tiovx_fc_ae_mode_get_type (),
          default_ae_mode,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_AWB_MODE,
      g_param_spec_enum ("awb-mode", "Auto white balance mode",
          "Flag to set if the auto white balance algorithm mode.",
          gst_tiovx_fc_awb_mode_get_type (),
          default_awb_mode,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_AE_NUM_SKIP_FRAMES,
      g_param_spec_uint ("ae-num-skip-frames", "AE number of skipped frames",
          "To indicate the AE algorithm how often to process frames, 0 means every frame.",
          0, G_MAXUINT,
          default_ae_num_skip_frames,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_AWB_NUM_SKIP_FRAMES,
      g_param_spec_uint ("awb-num-skip-frames", "AWB number of skipped frames",
          "To indicate the AWB algorithm how often to process frames, 0 means every frame.",
          0, G_MAXUINT,
          default_awb_num_skip_frames,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));
}

static void
gst_tiovx_fc_pad_init (GstTIOVXFCPad * self)
{
  self->videodev = NULL;

  memset (&self->ti_2a_wrapper, 0, sizeof (self->ti_2a_wrapper));

  self->dcc_2a_config_file = NULL;
  self->ae_mode = default_ae_mode;
  self->awb_mode = default_awb_mode;
  self->ae_num_skip_frames = default_ae_num_skip_frames;
  self->awb_num_skip_frames = default_awb_num_skip_frames;

  self->dcc_2a_buf = NULL;
  self->dcc_2a_buf_size = 0;
}

static void
gst_tiovx_fc_pad_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTIOVXFCPad *self = GST_TIOVX_FC_PAD (object);

  GST_LOG_OBJECT (self, "set_property");

  GST_OBJECT_LOCK (self);
  switch (prop_id) {
    case PROP_DEVICE:
      g_free (self->videodev);
      self->videodev = g_value_dup_string (value);
      break;
    case PROP_DCC_2A_CONFIG_FILE:
      g_free (self->dcc_2a_config_file);
      self->dcc_2a_config_file = g_value_dup_string (value);
      break;
    case PROP_AE_MODE:
      self->ae_mode = g_value_get_enum (value);
      break;
    case PROP_AWB_MODE:
      self->awb_mode = g_value_get_enum (value);
      break;
    case PROP_AE_NUM_SKIP_FRAMES:
      self->ae_num_skip_frames = g_value_get_uint (value);
      break;
    case PROP_AWB_NUM_SKIP_FRAMES:
      self->awb_num_skip_frames = g_value_get_uint (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}

static void
gst_tiovx_fc_pad_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstTIOVXFCPad *self = GST_TIOVX_FC_PAD (object);

  GST_LOG_OBJECT (self, "get_property");

  GST_OBJECT_LOCK (self);
  switch (prop_id) {
    case PROP_DEVICE:
      g_value_set_string (value, self->videodev);
      break;
    case PROP_DCC_2A_CONFIG_FILE:
      g_value_set_string (value, self->dcc_2a_config_file);
      break;
    case PROP_AE_MODE:
      g_value_set_enum (value, self->ae_mode);
      break;
    case PROP_AWB_MODE:
      g_value_set_enum (value, self->awb_mode);
      break;
    case PROP_AE_NUM_SKIP_FRAMES:
      g_value_set_uint (value, self->ae_num_skip_frames);
      break;
    case PROP_AWB_NUM_SKIP_FRAMES:
      g_value_set_uint (value, self->awb_num_skip_frames);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}


static void
gst_tiovx_fc_pad_finalize (GObject * obj)
{
  GstTIOVXFCPad *self = GST_TIOVX_FC_PAD (obj);

  g_free (self->videodev);
  self->videodev = NULL;

  g_free (self->dcc_2a_config_file);
  self->dcc_2a_config_file = NULL;


  G_OBJECT_CLASS (gst_tiovx_fc_pad_parent_class)->finalize (obj);
}

static GType
gst_tiovx_fc_ee_mode_get_type (void)
{
  static GType ee_mode_type = 0;

  static const GEnumValue targets[] = {
    {TIVX_VPAC_VISS_EE_MODE_OFF, "EE mode off", "EE_MODE_OFF"},
    {TIVX_VPAC_VISS_EE_MODE_Y12, "Edge Enhancer is enabled on Y12 output (output0)", "EE_MODE_Y12"},
    {TIVX_VPAC_VISS_EE_MODE_Y8, "Edge Enhancer is enabled on Y8 output (output2)", "EE_MODE_Y8"},
    {0, NULL, NULL},
  };

  if (!ee_mode_type) {
    ee_mode_type = g_enum_register_static ("GstTIOVXFCEEModes", targets);
  }
  return ee_mode_type;
}

static void
gst_tiovx_fc_class_init(GstTIOVXFCClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);
  GstTIOVXSimoClass *gsttiovxsimo_class = GST_TIOVX_SIMO_CLASS (klass);
  GstPadTemplate *src_temp = NULL;
  GstPadTemplate *sink_temp = NULL;

  GST_DEBUG_CATEGORY_INIT (gst_tiovxfc_debug, "tiovxfc", 0, "TIOVX FC PLUGIN");
  GST_DEBUG("class_init reached");

  gst_element_class_set_details_simple (gstelement_class,
        "TIOVX VISS->MSC FC",
        "Filter",
        "VPAC (VISS->MSC) Flexconnect using the TIOVX Modules API",
        "Pratham Deshmukh <p-deshmukh@ti.com>");

    src_temp =
      gst_pad_template_new_from_static_pad_template_with_gtype (&src_template,
      GST_TYPE_TIOVX_MULTISCALER_PAD);
      gst_element_class_add_pad_template (gstelement_class, src_temp);

    sink_temp = 
      gst_pad_template_new_from_static_pad_template_with_gtype (&sink_template,
      GST_TYPE_TIOVX_FC_PAD);
      gst_element_class_add_pad_template (gstelement_class, sink_temp);

    gobject_class->set_property = gst_tiovx_fc_set_property;
    gobject_class->get_property = gst_tiovx_fc_get_property;                
    
    gobject_class->finalize = GST_DEBUG_FUNCPTR (gst_tiovx_fc_finalize);    
  
  g_object_class_install_property (gobject_class, PROP_DCC_ISP_CONFIG_FILE,
      g_param_spec_string ("dcc-fc-isp-file", "DCC FC ISP File",
          "TIOVX DCC tuning binary file for the given image sensor.",
          NULL,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_SENSOR_NAME,
      g_param_spec_string ("sensor-name", "Sensor name",
          "TIOVX camera sensor string ID. Below are the supported sensors\n"
          "                                   SENSOR_SONY_IMX390_UB953_D3\n"
          "                                   SENSOR_ONSEMI_AR0820_UB953_LI\n"
          "                                   SENSOR_ONSEMI_AR0233_UB953_MARS\n"
          "                                   SENSOR_SONY_IMX219_RPI\n"
          "                                   SENSOR_SONY_IMX728_UB971_D3\n"
          "                                   SENSOR_OX05B1S\n"
          "                                   SENSOR_OV2312_UB953_LI",
          NULL,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_TARGET,
      g_param_spec_enum ("target", "Target",
          "TIOVX target to use by this element.",
          GST_TYPE_TIOVX_FC_TARGET,
          DEFAULT_TIOVX_FC_TARGET,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_NUM_EXPOSURES,
      g_param_spec_int ("num-exposures", "Number of exposures",
          "Number of exposures for the incoming raw image.",
          min_num_exposures, max_num_exposures, default_num_exposures,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_LINE_INTERLEAVED,
      g_param_spec_boolean ("lines-interleaved", "Interleaved lines",
          "Flag to indicate if lines are interleaved.",
          default_lines_interleaved,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_FORMAT_MSB,
      g_param_spec_int ("format-msb", "Format MSB",
          "Flag indicating which is the most significant bit that still has data.",
          min_format_msb, max_format_msb, default_format_msb,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_WDR_ENABLED,
      g_param_spec_boolean ("wdr-enabled", "Wdr Enabled",
          "Set if Camera wdr mode is enabled", default_wdr_enabled,
          G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
          G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_BYPASS_CAC,
      g_param_spec_boolean ("bypass-cac", "Bypass CAC",
          "Set to bypass chromatic aberation correction (CAC)", default_bypass_cac,
          G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
          G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_BYPASS_DWB,
      g_param_spec_boolean ("bypass-dwb", "Bypass DWB",
          "Set to bypass dynamic white balance (DWB)", default_bypass_dwb,
          G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
          G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_BYPASS_NSF4,
      g_param_spec_boolean ("bypass-nsf4", "Bypass NSF4",
          "Set to bypass Noise Filtering", default_bypass_nsf4,
          G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
          G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_EE_MODE,
      g_param_spec_enum ("ee-mode", "Edge Enhancement mode",
          "Flag to set Edge Enhancement mode.",
          gst_tiovx_fc_ee_mode_get_type (),
          default_ee_mode,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY));

  g_object_class_install_property (gobject_class, PROP_INTERPOLATION_METHOD,
      g_param_spec_enum ("interpolation-method", "Interpolation Method",
          "Interpolation method to use by the scaler",
          GST_TYPE_TIOVX_FC_INTERPOLATION_METHOD,
          DEFAULT_TIOVX_FC_INTERPOLATION_METHOD,
          G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS));

  gsttiovxsimo_class->init_module =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_init_module);

  gsttiovxsimo_class->configure_module = 
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_configure_module);
  
  gsttiovxsimo_class->get_node_info =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_get_node_info);

  gsttiovxsimo_class->create_graph =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_create_graph);

  gsttiovxsimo_class->get_sink_caps =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_get_sink_caps);

  gsttiovxsimo_class->get_src_caps =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_get_src_caps);

  gsttiovxsimo_class->fixate_caps =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_fixate_caps);

  gsttiovxsimo_class->deinit_module =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_deinit_module);

  gsttiovxsimo_class->postprocess =
      GST_DEBUG_FUNCPTR (gst_tiovx_fc_postprocess);

}



static void
gst_tiovx_fc_init (GstTIOVXFC * self)
{
  GST_DEBUG_OBJECT (self, "Entering gst_tiovx_fc_init\n");

  self->dcc_fc_config_file = NULL;
  self->sensor_name = g_strdup (default_tiovx_sensor_name);

  self->num_exposures = default_num_exposures;
  self->line_interleaved = default_lines_interleaved;
  self->format_msb = default_format_msb;
  self->meta_height_before = 0;
  self->meta_height_after = 0;
  self->wdr_enabled = default_wdr_enabled;
  self->bypass_cac = default_bypass_cac;
  self->bypass_dwb = default_bypass_dwb;
  self->bypass_nsf4 = default_bypass_nsf4;
  self->ee_mode = default_ee_mode;

  self->aewb_memory = NULL;
  self->h3a_stats_memory = NULL;

  self->user_data_allocator = g_object_new (GST_TYPE_TIOVX_ALLOCATOR, NULL);

  self->num_channels = 0;
  self->postprocess_iter = 0;

  for (gint i = 0; i < MAX_NUM_CHANNELS; i++) {
    self->input_references[i] = NULL;
  }
  
  self->target_id = DEFAULT_TIOVX_FC_TARGET;
  self->interpolation_method = DEFAULT_TIOVX_FC_INTERPOLATION_METHOD;

}

static gboolean
gst_tiovx_fc_configure_module (GstTIOVXSimo * simo)
{
  GstTIOVXFC *self = NULL;
  gboolean ret = TRUE;
  vx_status status = VX_FAILURE;
  vx_status check_status = VX_FAILURE;

  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tiovx_fc_configure_module =====================");


  g_return_val_if_fail (simo, FALSE);

  self = GST_TIOVX_FC (simo);

  if (self->fc_obj.node == NULL) {
    GST_ERROR_OBJECT (self, "FC node is NULL, cannot configure module");
    ret = FALSE;
    goto out;
  }

  check_status = vxGetStatus((vx_reference)self->fc_obj.node);
  if(check_status != VX_SUCCESS) {
    GST_ERROR_OBJECT (self, "FC node is invalid (status=%d), cannot configure module", check_status);
    fprintf(stderr, "FC node is invalid (status=%d), cannot configure module\n", check_status);
    ret = FALSE;
    goto out;
  }else
  {
    printf("FC node is valid in gstreamer %p\n", self->fc_obj.node);
  }
  
  GST_DEBUG_OBJECT (self, "Update filter coeffs");
  status = tiovx_fc_module_update_filter_coeffs (&self->fc_obj);
  if (VX_SUCCESS != status){
    GST_ERROR_OBJECT (self,
      "Module configure filter coefficients failed with error: %d", status);
    ret = FALSE;
    goto out;
  }
  
  GST_DEBUG_OBJECT (self, "Release buffer scaler");
  status = tiovx_fc_module_release_buffers (&self->fc_obj);
  if (VX_SUCCESS != status) {
    GST_ERROR_OBJECT (self,
        "Module configure release buffer failed with error: %d", status);
    ret = FALSE;
    goto out;
  }else
  {
    GST_ERROR_OBJECT (self,
        "Release buffer status: %d\n", status);
  }

out: 
  GST_ERROR_OBJECT (self,
          "Configure module ret is: %d\n", ret);
  return ret;
}




static gboolean 
gst_tiovx_fc_create_graph ( GstTIOVXSimo * simo, vx_context context, 
    vx_graph graph)
{
  GstTIOVXFC *self = NULL;
  vx_status status = VX_FAILURE;
  gboolean ret = FALSE;
  const gchar *target = NULL;

  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tiovx_fc_create_graph =====================");

  g_return_val_if_fail (simo, FALSE);
  g_return_val_if_fail (context, FALSE);
  g_return_val_if_fail (graph, FALSE);

  self = GST_TIOVX_FC (simo);

  GST_OBJECT_LOCK (GST_OBJECT (self));
  target = target_id_to_target_name (self->target_id);
  GST_OBJECT_UNLOCK (GST_OBJECT (self));

  GST_INFO_OBJECT (self, "TIOVX Target to use: %s", target);

  GST_DEBUG_OBJECT (self, "Creating FC graph");
  status = tiovx_fc_module_create (graph, &self->fc_obj, NULL, NULL, target);

  if (VX_SUCCESS != status) {
    GST_ERROR_OBJECT (self, "Create graph failed with error: %d", status);
    goto out;
  }
  else{
    GST_INFO_OBJECT (self, "Create graph passed with status: %d", status);
    fprintf(stderr, "[FC-CREATE-GRAPH] Node created: %p with status: %d\n", 
                self->fc_obj.node, vxGetStatus((vx_reference)self->fc_obj.node));
  }

  GST_DEBUG_OBJECT (self, "Finished creating flexconnect graph");

  ret = TRUE;

out:
  return ret;
}

static void
gst_tivox_fc_compute_src_dimension (GstTIOVXSimo * simo,
    const GValue * dimension, GValue * out_value, guint roi_len)
{
  static const gint scale = 4;
  gint out_max = -1;
  gint out_min = -1;
  gint dim_max = -1;
  gint dim_min = -1;
  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tivox_fc_compute_src_dimension =====================");
  

  g_return_if_fail (simo);
  g_return_if_fail (dimension);
  g_return_if_fail (out_value);

  if (roi_len) {
    dim_max = dim_min = roi_len;
  } else if (GST_VALUE_HOLDS_INT_RANGE (dimension)) {
    dim_max = gst_value_get_int_range_max (dimension);
    dim_min = gst_value_get_int_range_min (dimension);
  } else {
    dim_max = dim_min = g_value_get_int (dimension);
  }

  out_max = dim_max;
  out_min = 1.0 * dim_min / scale + 0.5;

  /* Minimum dimension is 1, 0 is invalid */
  if (0 == out_min) {
    out_min = 1;
  }

  GST_DEBUG_OBJECT (simo,
      "computed an output of [%d, %d] from an input of [%d, %d]", out_min,
      out_max, dim_min, dim_max);

  g_value_init (out_value, GST_TYPE_INT_RANGE);
  gst_value_set_int_range (out_value, out_min, out_max);
}

static void
gst_tivox_fc_compute_sink_dimension (GstTIOVXSimo * simo,
    const GValue * dimension, GValue * out_value, guint roi_len)
{
  static const gint scale = 4;
  gint out_max = -1;
  gint out_min = -1;
  gint dim_max = -1;
  gint dim_min = -1;

  g_return_if_fail (simo);
  g_return_if_fail (dimension);
  g_return_if_fail (out_value);

  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tivox_fc_compute_sink_dimension =====================");

  if (GST_VALUE_HOLDS_INT_RANGE (dimension)) {
    dim_max = gst_value_get_int_range_max (dimension);
    dim_min = gst_value_get_int_range_min (dimension);
  } else {
    dim_max = dim_min = g_value_get_int (dimension);
  }

  out_min = dim_min;

  if (G_MAXINT == dim_max || (G_MAXINT / scale) < dim_max || roi_len) {
    out_max = G_MAXINT;
  } else {
    out_max = dim_max * scale;
  }

  GST_DEBUG_OBJECT (simo,
      "computed an input of [%d, %d] from an output of [%d, %d]", out_min,
      out_max, dim_min, dim_max);

  g_value_init (out_value, GST_TYPE_INT_RANGE);
  gst_value_set_int_range (out_value, out_min, out_max);
}

typedef void (*GstTIOVXDimFunc) (GstTIOVXSimo * simo,
    const GValue * dimension, GValue * out_value, guint roi_len);

static void
gst_tivox_fc_compute_named (GstTIOVXSimo * simo,
    GstStructure * structure, const gchar * name, guint roi_len,
    GstTIOVXDimFunc func)
{
  const GValue *input = NULL;
  GValue output = G_VALUE_INIT;

  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tivox_fc_compute_named =====================");
  
  g_return_if_fail (simo);
  g_return_if_fail (structure);
  g_return_if_fail (name);
  g_return_if_fail (func);

  input = gst_structure_get_value (structure, name);
  func (simo, input, &output, roi_len);
  gst_structure_set_value (structure, name, &output);

  g_value_unset (&output);
}

static GstCaps *
gst_tiovx_fc_get_sink_caps (GstTIOVXSimo * simo,
    GstCaps * filter, GList * src_caps_list, GList *src_pads)
{
  GstCaps *sink_caps = NULL;
  GstCaps *template_caps = NULL;
  GList *l = NULL;
  GList *p = NULL;
  gint i = 0;

  GstTIOVXDimFunc func;
  GstCaps *src_caps;
  GstTIOVXMultiScalerPad *src_pad;

  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tiovx_fc_get_sink_caps =====================");

  g_return_val_if_fail (simo, NULL);
  g_return_val_if_fail (src_caps_list, NULL);

  GST_DEBUG_OBJECT (simo,
      "Computing sink caps based on src caps and filter %"
      GST_PTR_FORMAT, filter);

  template_caps = gst_static_pad_template_get_caps (&sink_template);

  if (template_caps != NULL) {
    GST_DEBUG_OBJECT (simo, "Retrieved sink template caps: %s", 
        gst_caps_to_string(template_caps));
  } else {
    GST_ERROR_OBJECT (simo, "Failed to retrieve capabilities from the sink template");
  }

  sink_caps = gst_caps_copy (template_caps);
  gst_caps_unref (template_caps);

  for (l = src_caps_list, p = src_pads; l != NULL;
      l = l->next, p = p->next) {
     func = gst_tivox_fc_compute_sink_dimension;
     src_caps = gst_caps_copy ((GstCaps *) l->data);
     src_pad = (GstTIOVXMultiScalerPad *) p->data;

    for (i = 0; i < gst_caps_get_size (src_caps); i++) {
      GstStructure *st = gst_caps_get_structure (src_caps, i);
      gst_tivox_fc_compute_named (simo, st, "width",
          src_pad->roi_width, func);
      gst_tivox_fc_compute_named (simo, st, "height",
          src_pad->roi_height, func);
    }

    /* Free the copied caps */
    gst_caps_unref (src_caps);
  }
    // Apply filter if provided
  if (filter) {
    GstCaps *tmp = sink_caps;
    sink_caps = gst_caps_intersect (sink_caps, filter);
    gst_caps_unref (tmp);
  }
  GST_DEBUG_OBJECT (simo, "Final Sink caps: %" GST_PTR_FORMAT, sink_caps);

  return sink_caps;

}

static GstCaps *
gst_tiovx_fc_get_src_caps (GstTIOVXSimo * simo,
    GstCaps * filter, GstCaps * sink_caps, GstTIOVXPad *src_pad_tiovx)
{
  GstCaps *src_caps = NULL;
  GstCaps *template_caps = NULL;
  gint i = 0;
  GstTIOVXMultiScalerPad *src_pad;
  GstTIOVXDimFunc func;

  GST_DEBUG_OBJECT (simo, "===================== Entering gst_tiovx_fc_get_src_caps =====================");

  g_return_val_if_fail (simo, NULL);
  g_return_val_if_fail (sink_caps, NULL);

  GST_DEBUG_OBJECT (simo,
      "Computing src caps based on sink caps %" GST_PTR_FORMAT " and filter %"
      GST_PTR_FORMAT, sink_caps, filter);

  template_caps = gst_static_pad_template_get_caps (&src_template);

  if (template_caps != NULL) {
    GST_DEBUG_OBJECT (simo, "Successfully retrieved src template capabilities: %s",
        gst_caps_to_string(template_caps));
  } else {
    GST_ERROR_OBJECT (simo, "Failed to retrieve capabilities from the static pad template");
    return NULL;
  }

  src_caps = gst_caps_copy (template_caps);
  gst_caps_unref (template_caps);
  src_pad = (GstTIOVXMultiScalerPad *) src_pad_tiovx;

  GST_INFO_OBJECT (simo,
      "The gst_caps_get_size is: %d", gst_caps_get_size (src_caps));

  for (i = 0; i < gst_caps_get_size (src_caps); i++) {
    GstStructure *st = gst_caps_get_structure (src_caps, i);

    GST_INFO_OBJECT (simo,
      "Entering the func loop");

    GST_INFO_OBJECT (simo,
      "Entering in gst_tivox_fc_compute_src_dimension");

    func = gst_tivox_fc_compute_src_dimension;

    gst_tivox_fc_compute_named (simo, st, "width",
        src_pad->roi_width, func);
    gst_tivox_fc_compute_named (simo, st, "height",
        src_pad->roi_height, func);
  }

  if (filter) {
    GstCaps *tmp = src_caps;
    src_caps = gst_caps_intersect (src_caps, filter);
    gst_caps_unref (tmp);
  }

  GST_INFO_OBJECT (simo,
      "Resulting supported src caps by TIOVX flexconnect node: %"
      GST_PTR_FORMAT, src_caps);

  return src_caps;
}

static GList *
gst_tiovx_fc_fixate_caps (GstTIOVXSimo *simo,
GstCaps *sink_caps,
GList *src_caps_list)
{
GstStructure *sink_structure = NULL;
GList *result_caps_list = NULL;
GList *l = NULL;

gint width = 0, height = 0;
gint meta_height_before = 0, meta_height_after = 0;

const gchar *input_format = NULL;
const GValue *vframerate = NULL;

g_return_val_if_fail (simo, NULL);
g_return_val_if_fail (sink_caps, NULL);
g_return_val_if_fail (gst_caps_is_fixed (sink_caps), NULL);
g_return_val_if_fail (src_caps_list, NULL);

sink_structure = gst_caps_get_structure (sink_caps, 0);

if (!gst_structure_get_int (sink_structure, "width", &width)) {
  GST_ERROR_OBJECT (simo, "Width is missing in sink caps");
  return NULL;
}
if (!gst_structure_get_int (sink_structure, "height", &height)) {
  GST_ERROR_OBJECT (simo, "Height is missing in sink caps");
  return NULL;
}

gst_structure_get_int (sink_structure, "meta-height-before", &meta_height_before);
gst_structure_get_int (sink_structure, "meta-height-after", &meta_height_after);

height = height - meta_height_before - meta_height_after;
if (height <= 0) {
GST_ERROR_OBJECT (simo,
"Invalid effective height after meta cropping: %d (before=%d after=%d)",
height, meta_height_before, meta_height_after);
return NULL;
}

input_format = gst_structure_get_string (sink_structure, "format");
if (NULL == input_format) {
GST_ERROR_OBJECT (simo, "Format is missing in sink caps");
return NULL;
}

vframerate = gst_structure_get_value (sink_structure, "framerate");
if (NULL == vframerate) {
GST_ERROR_OBJECT (simo, "Framerate is missing in sink caps");
return NULL;
}

GST_DEBUG_OBJECT (simo, "Fixating src caps from sink caps %" GST_PTR_FORMAT, sink_caps);

for (l = src_caps_list; l != NULL; l = l->next) {
    GstCaps *src_caps = (GstCaps *) l->data;
    GstStructure *src_st = NULL;
    GstCaps *out_caps = NULL;
    GstStructure *out_st = NULL;
    const GValue *vwidth = NULL, *vheight = NULL;
    const gchar *chosen_out_fmt = NULL;

    #if defined(SOC_AM62A) || defined(SOC_J722S)
    GValue output_formats = G_VALUE_INIT;
    GValue fmt = G_VALUE_INIT;
    #endif

    if (src_caps == NULL) {
      GST_ERROR_OBJECT (simo, "NULL src_caps in src_caps_list");
      goto error;
    }

    GST_DEBUG_OBJECT (simo, "Processing src_caps: %" GST_PTR_FORMAT, src_caps);

    out_caps = gst_caps_make_writable (gst_caps_copy (src_caps));

    src_st = gst_caps_get_structure (src_caps, 0);
    vwidth = gst_structure_get_value (src_st, "width");
    vheight = gst_structure_get_value (src_st, "height");

    out_st = gst_caps_get_structure (out_caps, 0);

    gst_structure_set_value (out_st, "framerate", vframerate);

    if (vwidth) {
      gst_structure_fixate_field_nearest_int (out_st, "width", width);
    }
    if (vheight) {
      gst_structure_fixate_field_nearest_int (out_st, "height", height);
    }

    if (g_list_length (src_caps_list) > 1) {
      gst_structure_fixate_field_nearest_int (out_st,
          "num-channels", g_list_length (src_caps_list));
      gst_caps_set_features_simple (out_caps,
          gst_tiovx_get_batched_memory_feature ());
    }

    #if defined(SOC_AM62A) || defined(SOC_J722S)

    if (NULL == g_strrstr (input_format, "i")) {
      const GValue *current_format = gst_structure_get_value (out_st, "format");

      if (current_format && GST_VALUE_HOLDS_LIST (current_format)) {
        g_value_init (&output_formats, GST_TYPE_LIST);
        g_value_init (&fmt, G_TYPE_STRING);

        g_value_set_string (&fmt, "NV12");
        gst_value_list_append_value (&output_formats, &fmt);
        g_value_reset (&fmt);

        g_value_set_string (&fmt, "GRAY8");
        gst_value_list_append_value (&output_formats, &fmt);

        gst_structure_set_value (out_st, "format", &output_formats);
        g_value_unset (&fmt);
        g_value_unset (&output_formats);

        GST_DEBUG_OBJECT (simo, "AM62A: Restricted format list to {NV12, GRAY8}");
      } else {
        GST_DEBUG_OBJECT (simo, "AM62A: Format already fixed, not overriding");
      }
    }
    #endif

    /* Final fixation */
    out_caps = gst_caps_fixate (out_caps);

    if (!out_caps || gst_caps_is_empty (out_caps)) {
      GST_ERROR_OBJECT (simo, "Failed to fixate caps");
      if (out_caps) {
        gst_caps_unref (out_caps);
      }
      goto error;
    }

    out_st = gst_caps_get_structure (out_caps, 0);
    chosen_out_fmt = gst_structure_get_string (out_st, "format");

    GST_INFO_OBJECT (simo,
        "FlexConnect fixation: input=%s (%dx%d) -> output=%s caps=%" GST_PTR_FORMAT,
        input_format, width, height,
        chosen_out_fmt ? chosen_out_fmt : "(null)",
        out_caps);

    result_caps_list = g_list_append (result_caps_list, out_caps);

}

return result_caps_list;

error:
  for (l = result_caps_list; l != NULL; l = l->next) {
  gst_caps_unref ((GstCaps *) l->data);
  }
  g_list_free (result_caps_list);
return NULL;
}

static gboolean
gst_tiovx_fc_deinit_module (GstTIOVXSimo * simo)
{
  GstTIOVXFC *self = NULL;
  vx_status status = VX_FAILURE;
  gboolean ret = FALSE;
     
  GST_DEBUG_OBJECT (self, "===================== Entering gst_tiovx_fc_deinit_module =====================");

  g_return_val_if_fail (simo, FALSE);

  self = GST_TIOVX_FC (simo);
  
  /* Delete graph */
  status = tiovx_fc_module_delete (&self->fc_obj);
  if (VX_SUCCESS != status) {
    GST_ERROR_OBJECT (self, "Module graph delete failed with error: %d", status);
    goto out;
  }
  
  /* Deinitialize module */
  status = tiovx_fc_module_deinit (&self->fc_obj);
  if (VX_SUCCESS != status) {
    GST_ERROR_OBJECT (self, "Module deinit failed with error: %d", status);
    goto out;
  }

  ret = TRUE;
  
out:
  return ret;

}

static const gchar *
target_id_to_target_name (gint target_id)
{
  GType type = G_TYPE_NONE;
  GEnumClass *enum_class = NULL;
  GEnumValue *enum_value = NULL;
  const gchar *value_nick = NULL;

  type = gst_tiovx_fc_target_get_type ();
  enum_class = G_ENUM_CLASS (g_type_class_ref (type));
  enum_value = g_enum_get_value (enum_class, target_id);
  value_nick = enum_value->value_nick;
  g_type_class_unref (enum_class);

  return value_nick;
}

static gboolean
gst_tiovx_fc_postprocess (GstTIOVXSimo * simo)
{
    GstTIOVXFC *self = NULL;
    GList *sink_pad_p = NULL;
    GList *l = NULL;
    gboolean ret = FALSE;
    struct v4l2_control control;
    gchar *video_dev = NULL;
    gint i = 0;
    g_return_val_if_fail (simo, FALSE);
    self = GST_TIOVX_FC (simo);

    GST_LOG_OBJECT (self, "Entering postprocess");
    sink_pad_p = GST_ELEMENT (simo)->sinkpads;
    
    for (l = sink_pad_p, i = 0; l != NULL; l = g_list_next (l), i++)
    {
        GstTIOVXFCPad *sink_pad = (GstTIOVXFCPad *) l->data;

          GST_DEBUG_OBJECT(self, "2A processing disabled - returning success");
          GST_LOG_OBJECT (self, "Entering postprocess with 2A enabled");

          GST_LOG_OBJECT (self, "sensor name is: %s\n", self->sensor_name);
          GST_LOG_OBJECT (self, "get_imx219_ae_dyn_params is: %p\n", &sink_pad->sensor_in_data.ae_dynPrms);
          if (g_strcmp0 (self->sensor_name, "SENSOR_SONY_IMX219_RPI") == 0)
          {
              get_imx219_ae_dyn_params (&sink_pad->sensor_in_data.ae_dynPrms);
          }

          GST_LOG_OBJECT (sink_pad, "&sink_pad->sensor_out_data is: %p\n", &sink_pad->sensor_out_data);
          GST_LOG_OBJECT (sink_pad,
                        "sink_pad is: %p", sink_pad);
          GST_LOG_OBJECT (sink_pad,
              "sink_pad->videodev is: %s", sink_pad->videodev);
          video_dev = sink_pad->videodev;
          if (NULL == video_dev) {
            GST_LOG_OBJECT (sink_pad,
                "Device location was not provided, skipping IOCTL calls");
          }
          else
          {
         
          gint fd = -1;
          int ret_val = -1;
          gint32 analog_gain = 0;
          gint32 coarse_integration_time = 0;

          fd = open (video_dev, O_RDWR | O_NONBLOCK);
          if (-1 == fd)
            {
              GST_ERROR_OBJECT (self, "Unable to open video device: %s", video_dev);
              goto exit;
            }
          gst_tiovx_fc_map_2A_values (self,
              sink_pad->sensor_out_data.aePrms.exposureTime[0],
              sink_pad->sensor_out_data.aePrms.analogGain[0],
              &coarse_integration_time, &analog_gain);
         
          control.id = exposure_ctrl_id;
          control.value = coarse_integration_time;
          ret_val = ioctl (fd, VIDIOC_S_CTRL, &control);
          if (ret_val < 0) {
            GST_ERROR_OBJECT (self, "Unable to call exposure ioctl: %d", ret_val);
            goto close_fd;
          }

          control.id = analog_gain_ctrl_id;
          control.value = analog_gain;
          ret_val = ioctl (fd, VIDIOC_S_CTRL, &control);
            if (ret_val < 0) {
              GST_ERROR_OBJECT (self, "Unable to call analog gain ioctl: %d",
                ret_val);
          }
          close_fd:
          close (fd);
        }
  
    ret = TRUE;
    GST_ERROR_OBJECT (self, "Return status is: %d\n", ret);
 
    exit:
        return ret;

    }

    return TRUE;
}

static int32_t
get_imx219_ae_dyn_params (IssAeDynamicParams * p_ae_dynPrms)
{
  int32_t status = -1;
  uint8_t count = 0;

  g_return_val_if_fail (p_ae_dynPrms, status);

  p_ae_dynPrms->targetBrightnessRange.min = 40;
  p_ae_dynPrms->targetBrightnessRange.max = 50;
  p_ae_dynPrms->targetBrightness = 45;
  p_ae_dynPrms->threshold = 1;
  p_ae_dynPrms->enableBlc = 1;
  p_ae_dynPrms->exposureTimeStepSize = 1;

  p_ae_dynPrms->exposureTimeRange[count].min = 10;
  p_ae_dynPrms->exposureTimeRange[count].max = 33333;  
  p_ae_dynPrms->analogGainRange[count].min = 1024;
  p_ae_dynPrms->analogGainRange[count].max = 8192;
  p_ae_dynPrms->digitalGainRange[count].min = 256;
  p_ae_dynPrms->digitalGainRange[count].max = 256;
  count++;

  p_ae_dynPrms->numAeDynParams = count;
  status = 0;
  return status;
}


static gboolean
gst_tiovx_fc_init_module (GstTIOVXSimo * simo,
vx_context context, GstTIOVXPad * sink_pad, GList * src_pads, GstCaps * sink_caps,
GList * src_caps_list, guint num_channels)
{
    GstTIOVXFC *self = NULL;
    GList *l = NULL;
    GstVideoInfo in_info = { };
    GstVideoInfo out_info = { };
    gboolean ret = FALSE;
    vx_status status = VX_FAILURE;
    const gchar *format_str = NULL;
    GstStructure *sink_caps_st = NULL;
    TIOVXFCModuleObj * flexconnect = NULL;
    gboolean is_bayer_input = FALSE;

   GST_DEBUG_OBJECT (self, "=====================Entering gst_tiovx_fc_init_module=====================");
    
    g_return_val_if_fail (simo, FALSE);
    g_return_val_if_fail (context, FALSE);
    g_return_val_if_fail (sink_pad, FALSE);
    g_return_val_if_fail (src_pads, FALSE);
    g_return_val_if_fail (sink_caps, FALSE);
    g_return_val_if_fail (src_caps_list, FALSE);

    self = GST_TIOVX_FC (simo);
    GST_DEBUG_OBJECT (self, "self pointer: %p", self);
    
    flexconnect = &self->fc_obj;
    GST_DEBUG_OBJECT (self, "flexconnect pointer: %p", flexconnect);
    
    GST_DEBUG_OBJECT (self, "Querying sensor");
    tiovx_querry_sensor (&self->sensor_obj);
    GST_DEBUG_OBJECT (self, "Initializing sensor '%s'", self->sensor_name);
    tiovx_init_sensor (&self->sensor_obj, self->sensor_name);
    
    GST_DEBUG_OBJECT (self, "Setting sensor parameters");
    self->sensor_obj.num_cameras_enabled = num_channels;
    self->sensor_obj.sensor_wdr_enabled = self->wdr_enabled;
    
    GST_DEBUG_OBJECT (self, "Number of num_channels are: %d\n", self->sensor_obj.num_cameras_enabled);

    GST_DEBUG_OBJECT (self, "Getting sink caps");
    if (NULL == sink_caps) {
        GST_ERROR_OBJECT (self, "Failed to get sink caps");
        goto out;
    }
    
    GST_DEBUG_OBJECT (self, "sink_caps: %" GST_PTR_FORMAT, sink_caps);
    sink_caps_st = gst_caps_get_structure (sink_caps, 0);
    GST_DEBUG_OBJECT (self, "sink_caps_st: %p", sink_caps_st);
    
    /* Initialize the input parameters */
    GST_DEBUG_OBJECT (self, "Extracting video info from caps");
    if (!gst_video_info_from_caps (&in_info, sink_caps)) {
        GST_ERROR_OBJECT (self, "Failed to get info from input pad: %" GST_PTR_FORMAT,
            sink_caps);
        goto out;
    }
    
    /* Extract metadata heights from sink caps */
    GST_DEBUG_OBJECT (self, "Extracting metadata heights");
    gst_structure_get_int (sink_caps_st, "meta-height-before", &self->meta_height_before);
    gst_structure_get_int (sink_caps_st, "meta-height-after", &self->meta_height_after);
    
    /* Store total and image heights */
    GST_DEBUG_OBJECT (self, "Computing image dimensions");
    
    self->total_height = GST_VIDEO_INFO_HEIGHT (&in_info);
    GST_DEBUG_OBJECT (self, "total_height is: %d\n", self->total_height);
    
    self->image_height = self->total_height - self->meta_height_before - self->meta_height_after;
    GST_DEBUG_OBJECT (self, "image_height is: %d\n", self->image_height);
    
    self->num_channels = num_channels;
    GST_DEBUG_OBJECT (self, "num_channels are: %d\n", self->num_channels);
    
    GST_DEBUG_OBJECT (self, "Setting up VISS input parameters");
    flexconnect->viss_input.bufq_depth = 1;
    flexconnect->viss_input.params.num_exposures = self->num_exposures;
    flexconnect->viss_input.params.line_interleaved = self->line_interleaved;
    flexconnect->viss_input.params.format[0].msb = self->format_msb;
    flexconnect->viss_input.params.meta_height_before = self->meta_height_before;
    flexconnect->viss_input.params.meta_height_after = self->meta_height_after;
    flexconnect->viss_input.params.width = GST_VIDEO_INFO_WIDTH (&in_info);
    flexconnect->viss_input.params.height = GST_VIDEO_INFO_HEIGHT (&in_info)
        - self->meta_height_before
        - self->meta_height_after;
    
    GST_DEBUG_OBJECT (self, "Getting format string");
    format_str = gst_structure_get_string (sink_caps_st, "format");
    if (NULL == format_str) {
        GST_ERROR_OBJECT (self, "Format is missing in sink caps");
        goto out;
    }
    GST_DEBUG_OBJECT (self, "Format string: %s", format_str);
    
    GST_DEBUG_OBJECT (self, "Determining pixel container format");
    if ((g_strcmp0 (format_str, "bggr16") == 0)
        || (g_strcmp0 (format_str, "gbrg16") == 0)
        || (g_strcmp0 (format_str, "grbg16") == 0)
        || (g_strcmp0 (format_str, "rggb16") == 0)
        || (g_strcmp0 (format_str, "bggr10") == 0)
        || (g_strcmp0 (format_str, "gbrg10") == 0)
        || (g_strcmp0 (format_str, "grbg10") == 0)
        || (g_strcmp0 (format_str, "rggb10") == 0)
        || (g_strcmp0 (format_str, "rggi10") == 0)
        || (g_strcmp0 (format_str, "grig10") == 0)
        || (g_strcmp0 (format_str, "bggi10") == 0)
        || (g_strcmp0 (format_str, "gbig10") == 0)
        || (g_strcmp0 (format_str, "girg10") == 0)
        || (g_strcmp0 (format_str, "iggr10") == 0)
        || (g_strcmp0 (format_str, "gibg10") == 0)
        || (g_strcmp0 (format_str, "iggb10") == 0)
        || (g_strcmp0 (format_str, "bggr12") == 0)
        || (g_strcmp0 (format_str, "gbrg12") == 0)
        || (g_strcmp0 (format_str, "grbg12") == 0)
        || (g_strcmp0 (format_str, "rggb12") == 0)
    ) {
        flexconnect->viss_input.params.format[0].pixel_container =
            TIVX_RAW_IMAGE_16_BIT;
        GST_DEBUG_OBJECT (self, "Setting 16-bit pixel container");
    } else if ((g_strcmp0 (format_str, "bggr") == 0)
            || (g_strcmp0 (format_str, "gbrg") == 0)
            || (g_strcmp0 (format_str, "grbg") == 0)
            || (g_strcmp0 (format_str, "rggb") == 0)
    ) {
        flexconnect->viss_input.params.format[0].pixel_container =
            TIVX_RAW_IMAGE_8_BIT;
        GST_DEBUG_OBJECT (self, "Setting 8-bit Bayer pixel container");
    } else if ((g_strcmp0 (format_str, "NV12") == 0) ||
            (g_strcmp0 (format_str, "NV12_P12") == 0)) {
        flexconnect->viss_input.params.format[0].pixel_container = TIVX_RAW_IMAGE_8_BIT;
        GST_DEBUG_OBJECT (self, "Setting 8-bit NV12/NV12_P12 pixel container");
    } else {
        GST_ERROR_OBJECT (self, "Couldn't determine pixel container from caps");
        goto out;
    }
    
    GST_DEBUG_OBJECT(self, "Setting up VISS-MSC input mapping");
    flexconnect->fc_params.msc_in_thread_viss_out_map[0] = TIVX_VPAC_FC_VISS_OUT2;
    flexconnect->fc_params.msc_in_thread_viss_out_map[1] = TIVX_VPAC_FC_MSC_CH_INVALID;
    flexconnect->fc_params.msc_in_thread_viss_out_map[2] = TIVX_VPAC_FC_MSC_CH_INVALID;
    flexconnect->fc_params.msc_in_thread_viss_out_map[3] = TIVX_VPAC_FC_MSC_CH_INVALID;
    
    GST_INFO_OBJECT (self,
        "Input parameters:\n"
        "\tWidth: %d\n"
        "\tHeight: %d\n"
        "\tPool size: %d\n"
        "\tNum exposures: %d\n"
        "\tLines interleaved: %d\n"
        "\tFormat pixel container: 0x%x\n"
        "\tFormat MSB: %d\n"
        "\tMeta height before: %d\n"
        "\tMeta height after: %d",
        flexconnect->viss_input.params.width,
        flexconnect->viss_input.params.height,
        flexconnect->viss_input.bufq_depth,
        flexconnect->viss_input.params.num_exposures,
        flexconnect->viss_input.params.line_interleaved,
        flexconnect->viss_input.params.format[0].pixel_container,
        flexconnect->viss_input.params.format[0].msb,
        flexconnect->viss_input.params.meta_height_before,
        flexconnect->viss_input.params.meta_height_after);
    
    /* Initialize tiovx params */
    GST_DEBUG_OBJECT (self, "Initializing tiovx params");
    tivx_vpac_fc_params_init(&flexconnect->fc_params);

    /* Initialize the output parameters */
    GST_DEBUG_OBJECT (self, "Initializing output parameters");
    for (l = src_caps_list; l != NULL; l = l->next) {
        GstCaps *src_caps = (GstCaps *) l->data;
        
        gint i = g_list_position (src_caps_list, l);
        
        GST_DEBUG_OBJECT (self, "Processing output %d, src_caps: %" GST_PTR_FORMAT, i, src_caps);
        
        if (!gst_video_info_from_caps (&out_info, src_caps)) {
            GST_ERROR_OBJECT (self, "Failed to get info from caps: %" GST_PTR_FORMAT, src_caps);
            ret = FALSE;
            goto out;
        }
        

        GST_DEBUG_OBJECT (self, "Setting up MSC output %d parameters", i);
        flexconnect->msc_output[i].width = GST_VIDEO_INFO_WIDTH (&out_info);
        flexconnect->msc_output[i].height = GST_VIDEO_INFO_HEIGHT (&out_info);
        flexconnect->msc_output[i].color_format = gst_format_to_vx_format (out_info.finfo->format);
        flexconnect->msc_output[i].bufq_depth = 1;
        flexconnect->msc_output[i].graph_parameter_index = i + 1;
        GST_DEBUG_OBJECT(self, "[FC-MODULE] MSC output %d reference: %p\n",i, (void *)&flexconnect->msc_output[i]);

        
        GST_INFO_OBJECT (self,
            "Output %d parameters: \n  Width: %d \n  Height: %d \n  Pool size: %d \n  Color format: 0x%x",
            i, 
            flexconnect->msc_output[i].width,
            flexconnect->msc_output[i].height,
            flexconnect->msc_output[i].bufq_depth,
            flexconnect->msc_output[i].color_format);
        GST_INFO_OBJECT (self, "[FC-GST] MSC output %d item: %p\n",i, (void *)&flexconnect->msc_output[i]);

    }
    
    GST_DEBUG_OBJECT (self, "Setting up ROI parameters");
    for (l = src_pads; l != NULL; l = l->next) {
        GstTIOVXMultiScalerPad *src_pad = (GstTIOVXMultiScalerPad *) l->data;
        gint i = g_list_position (src_pads, l);
        
        GST_DEBUG_OBJECT (self, "Processing src pad %d", i);
        
        flexconnect->raw_params.width = flexconnect->viss_input.params.width;
        flexconnect->raw_params.height = flexconnect->viss_input.params.height;

        GST_DEBUG_OBJECT (self, "raw_params width: %d, height: %d", 
                         flexconnect->raw_params.width, 
                         flexconnect->raw_params.height);
                         
        if (src_pad->roi_width == 0) {
            GST_DEBUG_OBJECT (self, "ROI width is 0, setting to full width");
            src_pad->roi_width = flexconnect->raw_params.width - src_pad->roi_startx;
            GST_DEBUG_OBJECT (self, "New ROI width: %d", src_pad->roi_width);
        }
        
        if (src_pad->roi_height == 0) {
            GST_DEBUG_OBJECT (self, "ROI height is 0, setting to full height");
            src_pad->roi_height = flexconnect->raw_params.height - src_pad->roi_starty;
            GST_DEBUG_OBJECT (self, "New ROI height: %d", src_pad->roi_height);
        }
        
        GST_DEBUG_OBJECT (self, "Checking ROI bounds");
        GST_DEBUG_OBJECT (self, "ROI: startx=%d, starty=%d, width=%d, height=%d", 
                         src_pad->roi_startx, src_pad->roi_starty, 
                         src_pad->roi_width, src_pad->roi_height);
        
        if (src_pad->roi_startx + src_pad->roi_width > flexconnect->raw_params.width) {
            GST_ERROR_OBJECT (self, "ROI width exceeds the input image");
            ret = FALSE;
            goto out;
        }
        
        if (src_pad->roi_starty + src_pad->roi_height > flexconnect->raw_params.height) {
            GST_ERROR_OBJECT (self, "ROI height exceeds the input image");
            ret = FALSE;
            goto out;
        }
                
        GST_DEBUG_OBJECT (self, "Checking downscaling factor");
        if (flexconnect->msc_output[i].width < src_pad->roi_width/4 ||
            flexconnect->msc_output[i].height < src_pad->roi_height/4) {
            GST_ERROR_OBJECT (self, "Flexconnect does not support downscaling by a factor > 4");
            ret = FALSE;
            goto out;
        }
        
        GST_DEBUG_OBJECT (self, "Setting crop parameters for output %d", i);
        flexconnect->msc_crop_params[i].crop_start_x = src_pad->roi_startx;
        flexconnect->msc_crop_params[i].crop_start_y = src_pad->roi_starty;
        flexconnect->msc_crop_params[i].crop_width = src_pad->roi_width;
        flexconnect->msc_crop_params[i].crop_height = src_pad->roi_height;
        
        GST_DEBUG_OBJECT (self, "Crop params for output %d: x=%d, y=%d, w=%d, h=%d", 
                         i, 
                         flexconnect->msc_crop_params[i].crop_start_x,
                         flexconnect->msc_crop_params[i].crop_start_y,
                         flexconnect->msc_crop_params[i].crop_width,
                         flexconnect->msc_crop_params[i].crop_height);
    }
    
#if defined(SOC_AM62A) 
    GST_DEBUG_OBJECT (self, "Setting up AM62A/J722S specific parameters");
    
    flexconnect->msc_num_outputs = g_list_length(src_caps_list); 
    GST_DEBUG_OBJECT(self, "Setting msc_num_outputs to %d based on src_caps_list length", 
                flexconnect->msc_num_outputs);
    GST_DEBUG_OBJECT (self, "Checking format for bypass_pcid");
    if (NULL == g_strrstr (format_str, "i")) {
        flexconnect->fc_params.tivxVissPrms.bypass_pcid = 1;
        GST_DEBUG_OBJECT (self, "Setting bypass_pcid=1 (non-i format)");
    } else {
        flexconnect->fc_params.tivxVissPrms.bypass_pcid = 0;
        GST_DEBUG_OBJECT (self, "Setting bypass_pcid=0 (i format)");
    }
    
    GST_DEBUG_OBJECT (self, "Configuring IR/Bayer operation based on output format");

    /* Detect if input is Bayer format for GRAY8 output decision */
    is_bayer_input = FALSE;
    if ((g_strrstr (format_str, "bggr") != NULL) ||
        (g_strrstr (format_str, "gbrg") != NULL) ||
        (g_strrstr (format_str, "grbg") != NULL) ||
        (g_strrstr (format_str, "rggb") != NULL)) {
        is_bayer_input = TRUE;
    }

    /* Map GStreamer video format to OpenVX format and configure VISS mode */
    switch (out_info.finfo->format) {
    case GST_VIDEO_FORMAT_NV12:
        GST_DEBUG_OBJECT (self, "Output format: NV12 (YUV 4:2:0) - Bayer mode");
        flexconnect->color_format = VX_DF_IMAGE_NV12;
        flexconnect->fc_params.tivxVissPrms.enable_ir_op = TIVX_VPAC_VISS_IR_DISABLE;
        flexconnect->fc_params.tivxVissPrms.enable_bayer_op = TIVX_VPAC_VISS_BAYER_ENABLE;
        break;

    case GST_VIDEO_FORMAT_GRAY8:
        flexconnect->color_format = VX_DF_IMAGE_U8;
        if (is_bayer_input) {
            /* Bayer sensor → GRAY8: Extract Y channel, keep Bayer processing */
            GST_DEBUG_OBJECT (self, "Output format: GRAY8 (U8) from Bayer input - Bayer mode (Y extract)");
            flexconnect->fc_params.tivxVissPrms.enable_ir_op = TIVX_VPAC_VISS_IR_DISABLE;
            flexconnect->fc_params.tivxVissPrms.enable_bayer_op = TIVX_VPAC_VISS_BAYER_ENABLE;
        } else {
            /* IR/Mono sensor → GRAY8: Direct passthrough */
            GST_DEBUG_OBJECT (self, "Output format: GRAY8 (U8) from IR/Mono input - IR mode");
            flexconnect->fc_params.tivxVissPrms.enable_ir_op = TIVX_VPAC_VISS_IR_ENABLE;
            flexconnect->fc_params.tivxVissPrms.enable_bayer_op = TIVX_VPAC_VISS_BAYER_DISABLE;
        }
        break;

    case GST_VIDEO_FORMAT_GRAY16_LE:
        GST_DEBUG_OBJECT (self, "Output format: GRAY16_LE (U16) - Bayer mode");
        flexconnect->color_format = VX_DF_IMAGE_U16;
        flexconnect->fc_params.tivxVissPrms.enable_ir_op = TIVX_VPAC_VISS_IR_DISABLE;
        flexconnect->fc_params.tivxVissPrms.enable_bayer_op = TIVX_VPAC_VISS_BAYER_ENABLE;
        break;

    case GST_VIDEO_FORMAT_UYVY:
        GST_DEBUG_OBJECT (self, "Output format: UYVY (YUV 4:2:2 packed) - Bayer mode");
        flexconnect->color_format = VX_DF_IMAGE_UYVY;
        flexconnect->fc_params.tivxVissPrms.enable_ir_op = TIVX_VPAC_VISS_IR_DISABLE;
        flexconnect->fc_params.tivxVissPrms.enable_bayer_op = TIVX_VPAC_VISS_BAYER_ENABLE;
        break;

    case GST_VIDEO_FORMAT_YUY2:
        GST_DEBUG_OBJECT (self, "Output format: YUYV (YUV 4:2:2 packed) - Bayer mode");
        flexconnect->color_format = VX_DF_IMAGE_YUYV;
        flexconnect->fc_params.tivxVissPrms.enable_ir_op = TIVX_VPAC_VISS_IR_DISABLE;
        flexconnect->fc_params.tivxVissPrms.enable_bayer_op = TIVX_VPAC_VISS_BAYER_ENABLE;
        break;

    default:
        GST_ERROR_OBJECT (self, "Unsupported output format: %s (0x%x)",
            gst_video_format_to_string (out_info.finfo->format),
            out_info.finfo->format);
        GST_ERROR_OBJECT (self, "Supported: NV12, GRAY8, GRAY16_LE, UYVY, YUYV");
        goto out;
    }

    GST_INFO_OBJECT (self, "Configured color_format=0x%x, IR=%d, Bayer=%d",
        flexconnect->color_format,
        flexconnect->fc_params.tivxVissPrms.enable_ir_op,
        flexconnect->fc_params.tivxVissPrms.enable_bayer_op);
    
    /* Apply processing settings */
    GST_DEBUG_OBJECT (self, "Setting processing parameters");
    GST_DEBUG_OBJECT (self, "bypass_cac: %d", self->bypass_cac);
    flexconnect->fc_params.tivxVissPrms.bypass_cac = self->bypass_cac;
    
    GST_DEBUG_OBJECT (self, "bypass_dwb: %d", self->bypass_dwb);
    flexconnect->fc_params.tivxVissPrms.bypass_dwb = self->bypass_dwb;
    
    GST_DEBUG_OBJECT (self, "bypass_nsf4: %d", self->bypass_nsf4);
    flexconnect->fc_params.tivxVissPrms.bypass_nsf4 = self->bypass_nsf4;
    
    GST_DEBUG_OBJECT (self, "ee_mode: %d", self->ee_mode);
    flexconnect->fc_params.tivxVissPrms.fcp[0].ee_mode = self->ee_mode;
    
    GST_DEBUG_OBJECT (self, "interpolation_method: %d", self->interpolation_method);
    flexconnect->interpolation_method = self->interpolation_method;
    
    flexconnect->msc_num_outputs = g_list_length (src_caps_list);
    GST_DEBUG_OBJECT (self, "Setting msc_num_outputs to %d", flexconnect->msc_num_outputs);
    
    GST_DEBUG_OBJECT (self, "enable_ir_op: %d", flexconnect->fc_params.tivxVissPrms.enable_ir_op);
    if (flexconnect->fc_params.tivxVissPrms.enable_ir_op) {
        /* For IR operation */
        GST_DEBUG_OBJECT (self, "Configuring for IR operation");
        
        /* Configure MSC outputs - route IR output to MSC output 0 */
        GST_DEBUG_OBJECT (self, "Configuring MSC outputs for IR");
        for (int i = 6; i <= 10; i++) {
            flexconnect->msc_output_select[i] = TIOVX_FC_MODULE_OUTPUT_EN;
            GST_DEBUG_OBJECT (self, "Number of outputs enabled for IR operation: %d", i);

            /* Configure MSC output 0 */
        GST_DEBUG_OBJECT (self, "Setting MSC output parameters for IR");
        flexconnect->msc_output[i].width = GST_VIDEO_INFO_WIDTH(&out_info);
        flexconnect->msc_output[i].height = GST_VIDEO_INFO_HEIGHT(&out_info);
        flexconnect->msc_output[i].color_format = gst_format_to_vx_format(out_info.finfo->format);
        flexconnect->msc_output[i].bufq_depth = 1;
        
        GST_DEBUG_OBJECT (self, "MSC output 0 configured: width=%d, height=%d, format=0x%x, enabled=%d",
                 flexconnect->msc_output[i].width,
                 flexconnect->msc_output[i].height,
                 flexconnect->msc_output[i].color_format,
                 flexconnect->msc_output_select[i]);
                 
        GST_INFO_OBJECT(self, 
            "FlexConnect IR output parameters:\n"
            "\tWidth: %d\n"
            "\tHeight: %d\n", 
            flexconnect->msc_output[i].width, 
            flexconnect->msc_output[i].height);

        }
        
        
    } else if (flexconnect->fc_params.tivxVissPrms.enable_bayer_op) 
#endif
    {
        GST_DEBUG_OBJECT (self, "Configuring for Bayer/Color operation");
        
        GST_DEBUG_OBJECT (self, "Setting MSC output selections");
            flexconnect->msc_output_select[0] = TIOVX_FC_MODULE_OUTPUT_EN;
            GST_DEBUG_OBJECT (self, "msc_output_select 0 is %d\n", flexconnect->msc_output_select[0]);
       
        for (l = src_caps_list; l != NULL; l = l->next) {
          GstCaps *src_caps = (GstCaps *) l->data;
          GstVideoInfo out_info = { };
          gint i = g_list_position (src_caps_list, l);
        
          if (!gst_video_info_from_caps (&out_info, src_caps)) {
          GST_ERROR_OBJECT (self, "Failed to get info from caps: %"
              GST_PTR_FORMAT, src_caps);
          ret = FALSE;
          goto out;
          }

          /* Configure MSC output */
          GST_DEBUG_OBJECT (self, "Setting MSC output parameters for Bayer");
            flexconnect->msc_output[i].width = GST_VIDEO_INFO_WIDTH(&out_info);
            flexconnect->msc_output[i].height = GST_VIDEO_INFO_HEIGHT(&out_info);
            flexconnect->msc_output[i].color_format = gst_format_to_vx_format(out_info.finfo->format);
            flexconnect->msc_output[i].bufq_depth = 1;
            
            GST_INFO_OBJECT(self, 
                "FlexConnect Bayer output parameters:\n"
                "\tWidth: %d\n"
                "\tHeight: %d\n"
                "\tColor format: 0x%x", 
                flexconnect->msc_output[i].width, 
                flexconnect->msc_output[i].height,
                flexconnect->msc_output[i].color_format);
        }
    }  
    
    /* Set interpolation method */
    GST_DEBUG_OBJECT (self, "Setting interpolation method");
    GST_OBJECT_LOCK (GST_OBJECT (self));
    flexconnect->interpolation_method = self->interpolation_method;
    GST_OBJECT_UNLOCK (GST_OBJECT (self));
    GST_DEBUG_OBJECT (self, "interpolation_method set to %d", flexconnect->interpolation_method);
    
    /* Initialize the FlexConnect module */
    GST_INFO_OBJECT (self, "Initializing FlexConnect module");
    GST_DEBUG_OBJECT (self, "context: %p, flexconnect: %p, sensor_obj: %p", 
                     context, flexconnect, &self->sensor_obj);
                     
    GST_DEBUG_OBJECT (self, "Checking if sensor_obj is properly initialized");
    GST_DEBUG_OBJECT (self, "sensor_name: %s", self->sensor_obj.sensor_name);
    GST_DEBUG_OBJECT (self, "num_cameras_enabled: %d", self->sensor_obj.num_cameras_enabled);
    GST_DEBUG_OBJECT (self, "sensor_wdr_enabled: %d", self->sensor_obj.sensor_wdr_enabled);
    
    /* Check raw_params is initialized */
    GST_DEBUG_OBJECT (self, "Checking if raw_params is initialized");
    GST_DEBUG_OBJECT (self, "raw_params width: %d, height: %d", 
                     flexconnect->raw_params.width, 
                     flexconnect->raw_params.height);
    
    /* About to call the module init function */
    GST_DEBUG_OBJECT (self, "Calling tiovx_fc_module_init()");
    for (int i = 1; i < 6; i++){
    GST_DEBUG_OBJECT (self, "msc_output_select before tiovx_fc_module_init is %d", flexconnect->msc_output_select[i]);
    }
    status = tiovx_fc_module_init (context, flexconnect, &self->sensor_obj);
    
    GST_DEBUG_OBJECT (self, "tiovx_fc_module_init() returned status: %d", status);
    if (VX_SUCCESS != status) {
        GST_ERROR_OBJECT (self, "Module init failed with error: %d", status);
        goto out;
    }
    
    GST_DEBUG_OBJECT (self, "Module init succeeded");
    ret = TRUE;
    
out:    
    GST_DEBUG_OBJECT (self, "Leaving gst_tiovx_fc_init_module with result: %d", ret);
    return ret;
}

static gboolean
gst_tiovx_fc_get_node_info (GstTIOVXSimo * simo, vx_node * node,
    GstTIOVXPad * sink_pad, GList * src_pads, GList ** queueable_objects)
{
  GstTIOVXFC *self = NULL;
  GList *l = NULL;

  GST_DEBUG_OBJECT (simo, "=====================Entering gst_tiovx_fc_get_node_info=====================");
  
  
  g_return_val_if_fail (simo, FALSE);
  g_return_val_if_fail (sink_pad, FALSE);
  g_return_val_if_fail (src_pads, FALSE);
  
  self = GST_TIOVX_FC (simo);
  GST_DEBUG_OBJECT (simo, "Cast to GstTIOVXFC successful");
  
  *node = self->fc_obj.node;
  fprintf(stderr, "[FC-GET-NODE-INFO] Assigning node: %p with status: %d\n", 
            *node, vxGetStatus((vx_reference)*node));
  GST_DEBUG_OBJECT (simo, "Node assigned: %p", *node);
  
  /* Set input parameters */
  GST_DEBUG_OBJECT (simo, "Setting sink pad parameters");
  gst_tiovx_pad_set_params (sink_pad,
      self->fc_obj.viss_input.arr[0], 
      (vx_reference) self->fc_obj.viss_input.image_handle[0],
      self->fc_obj.viss_input.graph_parameter_index, input_param_id);
  GST_DEBUG_OBJECT (simo, "Sink pad parameters set with graph_parameter_index: %d", 
      self->fc_obj.viss_input.graph_parameter_index);
  
  GST_DEBUG_OBJECT (simo, "Processing %d output pads", g_list_length(src_pads));
  for (l = src_pads; l != NULL; l = l->next) {
    GstTIOVXPad *src_pad = (GstTIOVXPad *) l->data;
    gint i = g_list_position (src_pads, l);
    
    /* Set output parameters */
    GST_DEBUG_OBJECT (simo, "Setting src pad %d parameters", i);
    gst_tiovx_pad_set_params (src_pad,
        self->fc_obj.msc_output[i].arr[0],
        (vx_reference) self->fc_obj.msc_output[i].image_handle[0],
        self->fc_obj.msc_output[i].graph_parameter_index, output0_param_id + i);
    GST_DEBUG_OBJECT (simo, "Src pad %d parameters set with graph_parameter_index: %d, param_id: %d", 
        i, self->fc_obj.msc_output[i].graph_parameter_index, output0_param_id + i);
  }
  
 
  GST_DEBUG_OBJECT (simo, "Exiting gst_tiovx_fc_get_node_info successfully");
  return TRUE;
}

static void
gst_tiovx_fc_map_2A_values (GstTIOVXFC * self, int exposure_time,
    int analog_gain, gint32 * exposure_time_mapped, gint32 * analog_gain_mapped)
{
  double multiplier = 0;

  g_return_if_fail (self);
  g_return_if_fail (exposure_time_mapped);
  g_return_if_fail (analog_gain_mapped);

  if (g_strcmp0 (self->sensor_name, "SENSOR_SONY_IMX390_UB953_D3") == 0) {
    // gint i = 0;
    // for (i = 0; i < ISS_IMX390_GAIN_TBL_SIZE - 1; i++) {
    //   if (gIMX390GainsTable[i][0] >= analog_gain) {
    //     break;
    //   }
    // }
    // *exposure_time_mapped = exposure_time;
    // *analog_gain_mapped = gIMX390GainsTable[i][1];

    g_print("Nothing to print here 1\n");
  } else if (g_strcmp0 (self->sensor_name, "SENSOR_SONY_IMX728_UB971_D3") == 0) {
    // gint i = 0;
    // for (i = 0; i < ISS_IMX728_GAIN_TBL_SIZE - 1; i++) {
    //   if (gIMX728GainsTable[i][0] >= analog_gain) {
    //     break;
    //   }
    // }
    // *exposure_time_mapped = exposure_time;
    // *analog_gain_mapped = gIMX728GainsTable[i][1];
    g_print("Nothing to print here 2\n");
  } else if (g_strcmp0 (self->sensor_name, "SENSOR_SONY_IMX219_RPI") == 0) {
    GST_LOG_OBJECT (self, "Check for exposure time\n");

    /* convert exposure time from micro seconds to number of lines - refer to sensor datasheet */ 
    *exposure_time_mapped = (1080 * exposure_time / 33333);  /* for 1920x1080 at 30fps */
    //*exposure_time_mapped = (2464 * exposure_time / 66666);  /* for 3280x2464 at 15fps */

    /* convert gain to the format assumed by the sensor - refer to sensor data sheet */ 
    multiplier = analog_gain / 1024.0;  // 1024 is 1x gain */
    *analog_gain_mapped = 256.0 - 256.0 / multiplier;
  } else if (g_strcmp0 (self->sensor_name, "SENSOR_OV2312_UB953_LI") == 0) {
    *exposure_time_mapped = (60 * 1300 * exposure_time / 1000000);
    // ms to row_time conversion - row_time(us) = 1000000/fps/height
    *analog_gain_mapped = analog_gain;
} else if (g_strcmp0 (self->sensor_name, "SENSOR_OX05B1S") == 0) {
    *exposure_time_mapped = (int) ((double)exposure_time * 2128 * 60 / 1000000 + 0.5);
    *analog_gain_mapped = analog_gain / 64;
  } else {
    GST_ERROR_OBJECT (self, "Unknown sensor: %s", self->sensor_name);
  }
}



static void
gst_tiovx_fc_finalize (GObject * obj)
{
  GstTIOVXFC *self = GST_TIOVX_FC (obj);
  gint i = 0;
 
  GST_DEBUG_OBJECT (self, "===================== Entering gst_tiovx_fc_finalize =====================");
 
  g_free (self->dcc_fc_config_file);
  self->dcc_fc_config_file = NULL;
  g_free (self->sensor_name);
  self->sensor_name = NULL;
 
  if (NULL != self->aewb_memory) {
    gst_memory_unref (self->aewb_memory);
    GST_DEBUG_OBJECT (self, "aewb memory status: %p ", self->aewb_memory);
  }else{
    GST_DEBUG_OBJECT (self, "aewb memory is NULL: %p ", self->aewb_memory);

  }
  if (NULL != self->h3a_stats_memory) {
    gst_memory_unref (self->h3a_stats_memory);
  }else{
    GST_DEBUG_OBJECT (self, "h3a_stats memory is NULL: %p ", self->h3a_stats_memory);

  }
  if (self->user_data_allocator) {
    g_object_unref (self->user_data_allocator);
    GST_DEBUG_OBJECT (self, "user data allocator is : %p ", self->user_data_allocator);

  }else{
    GST_DEBUG_OBJECT (self, "user data allocator is NULL: %p ", self->user_data_allocator);

  }
 
  for (i = 0; i < MAX_NUM_CHANNELS; i++) {
    if (self->input_references[i]) {
      vxReleaseReference (&self->input_references[i]);
      self->input_references[i] = NULL;
    }
  }
 
  G_OBJECT_CLASS (gst_tiovx_fc_parent_class)->finalize (obj);
 
}

static void
gst_tiovx_fc_set_property(GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTIOVXFC *self = GST_TIOVX_FC (object);

  GST_DEBUG_OBJECT (self, "===================== Entering gst_tiovx_fc_set_property =====================");

  GST_OBJECT_LOCK (self);
  switch (prop_id) {
    case PROP_DCC_ISP_CONFIG_FILE:
      g_free (self->dcc_fc_config_file);
      self->dcc_fc_config_file = g_value_dup_string (value);
      break;
    case PROP_SENSOR_NAME:
      g_free (self->sensor_name);
      self->sensor_name = g_value_dup_string (value);
      break;
    case PROP_TARGET:
      self->target_id = g_value_get_enum (value);
      break;
    case PROP_NUM_EXPOSURES:
      self->num_exposures = g_value_get_int (value);
      break;
    case PROP_LINE_INTERLEAVED:
      self->line_interleaved = g_value_get_boolean (value);
      break;
    case PROP_FORMAT_MSB:
      self->format_msb = g_value_get_int (value);
      break;
    case PROP_WDR_ENABLED:
      self->wdr_enabled = g_value_get_boolean (value);
      break;
    case PROP_BYPASS_CAC:
      self->bypass_cac = g_value_get_boolean (value);
      break;
    case PROP_BYPASS_DWB:
      self->bypass_dwb = g_value_get_boolean (value);
      break;
    case PROP_BYPASS_NSF4:
      self->bypass_nsf4 = g_value_get_boolean (value);
      break;
    case PROP_EE_MODE:
      self->ee_mode = g_value_get_enum (value);
      break;
    case PROP_INTERPOLATION_METHOD:
      self->interpolation_method = g_value_get_enum (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}

static void
gst_tiovx_fc_get_property(GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstTIOVXFC *self = GST_TIOVX_FC (object);

  GST_DEBUG_OBJECT (self, "===================== Entering gst_tiovx_fc_get_property =====================");

  GST_OBJECT_LOCK (self);
  switch (prop_id) {
    case PROP_DCC_ISP_CONFIG_FILE:
      g_value_set_string (value, self->dcc_fc_config_file);
      break;
    case PROP_SENSOR_NAME:
      g_value_set_string (value, self->sensor_name);
      break;
    case PROP_TARGET:
      g_value_set_enum (value, self->target_id);
      break;
    case PROP_NUM_EXPOSURES:
      g_value_set_int (value, self->num_exposures);
      break;
    case PROP_LINE_INTERLEAVED:
      g_value_set_boolean (value, self->line_interleaved);
      break;
    case PROP_FORMAT_MSB:
      g_value_set_int (value, self->format_msb);
      break;
    case PROP_WDR_ENABLED:
      g_value_set_boolean (value, self->wdr_enabled);
      break;
    case PROP_BYPASS_CAC:
      g_value_set_boolean (value, self->bypass_cac);
      break;
    case PROP_BYPASS_DWB:
      g_value_set_boolean (value, self->bypass_dwb);
      break;
    case PROP_BYPASS_NSF4:
      g_value_set_boolean (value, self->bypass_nsf4);
      break;
    case PROP_EE_MODE:
      g_value_set_enum (value, self->ee_mode);
      break;
    case PROP_INTERPOLATION_METHOD:
      g_value_set_enum (value, self->interpolation_method);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}
