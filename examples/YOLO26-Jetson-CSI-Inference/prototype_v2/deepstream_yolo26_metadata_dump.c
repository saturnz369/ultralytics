// Separate DeepStream prototype_v2 metadata dump app.
// This keeps full frames on the DeepStream/GPU path and exports only bbox metadata.

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cuda_runtime_api.h>

#include "gstnvdsmeta.h"

#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 720
#define DEFAULT_FPS_N 30
#define DEFAULT_FPS_D 1
#define DEFAULT_MAX_DISPLAY_LEN 128
#define DEFAULT_MAX_OBJECTS 256

typedef struct AppConfig {
  gint sensor_id;
  gint width;
  gint height;
  gint fps_n;
  gint fps_d;
  gboolean show;
  guint max_frames;
  gchar *infer_config;
  gchar *state_file;
} AppConfig;

typedef struct AppState {
  GMainLoop *loop;
  GstElement *pipeline;
  guint frame_number;
  guint max_frames;
  FILE *state_fp;
} AppState;

static gchar *
env_strdup_default(const gchar *key, const gchar *fallback)
{
  const gchar *value = g_getenv(key);
  return g_strdup((value && *value) ? value : fallback);
}

static gint
env_int_default(const gchar *key, gint fallback)
{
  const gchar *value = g_getenv(key);
  return (value && *value) ? (gint) g_ascii_strtoll(value, NULL, 10) : fallback;
}

static gboolean
env_bool_default(const gchar *key, gboolean fallback)
{
  const gchar *value = g_getenv(key);
  if (!value || !*value) {
    return fallback;
  }
  if (!g_ascii_strcasecmp(value, "1") ||
      !g_ascii_strcasecmp(value, "true") ||
      !g_ascii_strcasecmp(value, "yes") ||
      !g_ascii_strcasecmp(value, "on")) {
    return TRUE;
  }
  if (!g_ascii_strcasecmp(value, "0") ||
      !g_ascii_strcasecmp(value, "false") ||
      !g_ascii_strcasecmp(value, "no") ||
      !g_ascii_strcasecmp(value, "off")) {
    return FALSE;
  }
  return fallback;
}

static void
load_config(AppConfig *cfg)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->sensor_id = env_int_default("SENSOR_ID", 0);
  cfg->width = env_int_default("CAMERA_WIDTH", DEFAULT_WIDTH);
  cfg->height = env_int_default("CAMERA_HEIGHT", DEFAULT_HEIGHT);
  cfg->fps_n = env_int_default("CAMERA_FPS_N", DEFAULT_FPS_N);
  cfg->fps_d = env_int_default("CAMERA_FPS_D", DEFAULT_FPS_D);
  cfg->show = env_bool_default("SHOW", TRUE);
  cfg->max_frames = (guint) env_int_default("MAX_FRAMES", 0);
  cfg->infer_config = env_strdup_default(
      "INFER_CONFIG",
      "/home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/config/config_infer_primary_yolo26.txt");
  cfg->state_file = env_strdup_default("STATE_FILE", "/tmp/deepstream_yolo26_metadata.jsonl");
}

static void
free_config(AppConfig *cfg)
{
  g_free(cfg->infer_config);
  g_free(cfg->state_file);
}

static gboolean
bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
  (void) bus;
  AppState *state = (AppState *) data;
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
      g_print("End of stream\n");
      g_main_loop_quit(state->loop);
      break;
    case GST_MESSAGE_ERROR: {
      GError *error = NULL;
      gchar *debug = NULL;
      gst_message_parse_error(msg, &error, &debug);
      g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), error->message);
      if (debug) {
        g_printerr("Details: %s\n", debug);
      }
      g_clear_error(&error);
      g_free(debug);
      g_main_loop_quit(state->loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

static void
write_frame_json(FILE *fp, guint frame_number, NvDsFrameMeta *frame_meta)
{
  NvDsMetaList *l_obj = NULL;
  guint object_count = 0;

  fprintf(fp,
          "{\"frame_idx\":%u,\"source_id\":%u,\"ntp_timestamp\":%" G_GUINT64_FORMAT ",\"objects\":[",
          frame_number,
          frame_meta->source_id,
          frame_meta->ntp_timestamp);

  for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
    NvDsObjectMeta *obj_meta = (NvDsObjectMeta *) l_obj->data;
    const gdouble left = obj_meta->rect_params.left;
    const gdouble top = obj_meta->rect_params.top;
    const gdouble width = obj_meta->rect_params.width;
    const gdouble height = obj_meta->rect_params.height;
    const gdouble cx = left + (width / 2.0);
    const gdouble cy = top + (height / 2.0);
    const gchar *label = obj_meta->obj_label && *obj_meta->obj_label ? obj_meta->obj_label : "unknown";

    if (object_count++ > 0) {
      fprintf(fp, ",");
    }

    fprintf(fp,
            "{\"track_id\":%" G_GUINT64_FORMAT ",\"class_id\":%d,\"label\":\"%s\",\"confidence\":%.4f,"
            "\"left\":%.2f,\"top\":%.2f,\"width\":%.2f,\"height\":%.2f,\"cx\":%.2f,\"cy\":%.2f}",
            (guint64) obj_meta->object_id,
            obj_meta->class_id,
            label,
            obj_meta->confidence,
            left,
            top,
            width,
            height,
            cx,
            cy);
  }

  fprintf(fp, "]}\n");
  fflush(fp);
}

static GstPadProbeReturn
pgie_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  AppState *state = (AppState *) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
  NvDsMetaList *l_frame = NULL;

  if (!batch_meta) {
    return GST_PAD_PROBE_OK;
  }

  for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
    NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) l_frame->data;
    guint object_count = 0;
    NvDsMetaList *l_obj = NULL;

    for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
      object_count++;
    }

    g_print("Frame %u | objects=%u\n", state->frame_number, object_count);

    if (state->state_fp) {
      write_frame_json(state->state_fp, state->frame_number, frame_meta);
    }

    state->frame_number++;
    if (state->max_frames > 0 && state->frame_number >= state->max_frames) {
      gst_element_send_event(state->pipeline, gst_event_new_eos());
      break;
    }
  }

  return GST_PAD_PROBE_OK;
}

int
main(int argc, char *argv[])
{
  (void) argc;
  (void) argv;

  AppConfig cfg;
  AppState state;
  GstElement *pipeline = NULL;
  GstElement *source = NULL;
  GstElement *source_caps = NULL;
  GstElement *source_conv = NULL;
  GstElement *streammux = NULL;
  GstElement *pgie = NULL;
  GstElement *nvvidconv = NULL;
  GstElement *nvosd = NULL;
  GstElement *sink = NULL;
  GstBus *bus = NULL;
  GstCaps *caps = NULL;
  GstPad *mux_sink_pad = NULL;
  GstPad *source_src_pad = NULL;
  GstPad *pgie_src_pad = NULL;
  guint bus_watch_id = 0;
  struct cudaDeviceProp prop;

  memset(&state, 0, sizeof(state));
  load_config(&cfg);

  gst_init(NULL, NULL);

  state.loop = g_main_loop_new(NULL, FALSE);
  state.max_frames = cfg.max_frames;

  cudaGetDeviceProperties(&prop, 0);

  pipeline = gst_pipeline_new("deepstream-yolo26-metadata-dump");
  source = gst_element_factory_make("nvarguscamerasrc", "csi-source");
  source_caps = gst_element_factory_make("capsfilter", "source-caps");
  source_conv = gst_element_factory_make("nvvideoconvert", "source-convert");
  streammux = gst_element_factory_make("nvstreammux", "stream-muxer");
  pgie = gst_element_factory_make("nvinfer", "primary-infer");
  nvvidconv = gst_element_factory_make("nvvideoconvert", "video-convert");
  nvosd = gst_element_factory_make("nvdsosd", "osd");
  sink = gst_element_factory_make(cfg.show ? "nv3dsink" : "fakesink", cfg.show ? "display" : "fakesink");

  if (!pipeline || !source || !source_caps || !source_conv || !streammux || !pgie || !nvvidconv || !nvosd || !sink) {
    g_printerr("Failed to create one or more GStreamer elements.\n");
    free_config(&cfg);
    return 1;
  }

  state.pipeline = pipeline;
  if (cfg.state_file && *cfg.state_file) {
    state.state_fp = fopen(cfg.state_file, "w");
    if (!state.state_fp) {
      g_printerr("Failed to open STATE_FILE: %s\n", cfg.state_file);
      free_config(&cfg);
      return 1;
    }
  }

  caps = gst_caps_from_string(g_strdup_printf(
      "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/%d, format=(string)NV12",
      cfg.width, cfg.height, cfg.fps_n, cfg.fps_d));

  g_object_set(G_OBJECT(source), "sensor-id", cfg.sensor_id, NULL);
  g_object_set(G_OBJECT(source_caps), "caps", caps, NULL);
  g_object_set(G_OBJECT(streammux),
               "width", cfg.width,
               "height", cfg.height,
               "batch-size", 1,
               "batched-push-timeout", 40000,
               "live-source", TRUE,
               NULL);
  g_object_set(G_OBJECT(pgie), "config-file-path", cfg.infer_config, NULL);
  if (cfg.show) {
    g_object_set(G_OBJECT(sink), "sync", FALSE, NULL);
  }

  gst_bin_add_many(GST_BIN(pipeline), source, source_caps, source_conv, streammux, pgie, nvvidconv, nvosd, sink, NULL);

  if (!gst_element_link_many(source, source_caps, source_conv, NULL)) {
    g_printerr("Failed to link source chain.\n");
    goto cleanup;
  }

  source_src_pad = gst_element_get_static_pad(source_conv, "src");
  mux_sink_pad = gst_element_get_request_pad(streammux, "sink_0");
  if (!source_src_pad || !mux_sink_pad || gst_pad_link(source_src_pad, mux_sink_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link source converter to streammux.\n");
    goto cleanup;
  }

  if (!gst_element_link_many(streammux, pgie, nvvidconv, nvosd, sink, NULL)) {
    g_printerr("Failed to link inference/display chain.\n");
    goto cleanup;
  }

  pgie_src_pad = gst_element_get_static_pad(pgie, "src");
  if (!pgie_src_pad) {
    g_printerr("Failed to get PGIE src pad.\n");
    goto cleanup;
  }
  gst_pad_add_probe(pgie_src_pad, GST_PAD_PROBE_TYPE_BUFFER, pgie_src_pad_buffer_probe, &state, NULL);

  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, bus_call, &state);
  gst_object_unref(bus);

  g_print("Starting prototype_v2 DeepStream metadata dump on CSI sensor %d\n", cfg.sensor_id);
  g_print("Infer config: %s\n", cfg.infer_config);
  g_print("State file: %s\n", cfg.state_file);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(state.loop);

cleanup:
  if (bus_watch_id > 0) {
    g_source_remove(bus_watch_id);
  }
  if (pgie_src_pad) {
    gst_object_unref(pgie_src_pad);
  }
  if (source_src_pad) {
    gst_object_unref(source_src_pad);
  }
  if (mux_sink_pad) {
    gst_object_unref(mux_sink_pad);
  }
  if (caps) {
    gst_caps_unref(caps);
  }
  if (pipeline) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
  }
  if (state.state_fp) {
    fclose(state.state_fp);
  }
  if (state.loop) {
    g_main_loop_unref(state.loop);
  }
  free_config(&cfg);
  return 0;
}
