// Separate DeepStream prototype_v2 combined RTSP + tracked-metadata control preview.
// This app keeps video streaming and metadata-only control preview in one pipeline.

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "gstnvdsmeta.h"

#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 720
#define DEFAULT_FPS_N 30
#define DEFAULT_FPS_D 1
#define DEFAULT_TRACKER_WIDTH 960
#define DEFAULT_TRACKER_HEIGHT 544
#define DEFAULT_RTSP_PORT 8554
#define DEFAULT_UDP_PORT 5400
#define DEFAULT_RTSP_BITRATE 4000000
#define DEFAULT_RTSP_QUEUE_BUFFERS 4
#define DEFAULT_RTSP_VIEWER_LATENCY_MS 80
#define MAX_CANDIDATES 256

typedef struct AppConfig {
  gint sensor_id;
  gint width;
  gint height;
  gint fps_n;
  gint fps_d;
  gboolean show;
  guint max_frames;
  gint target_class_id;
  gchar *selection;
  gint target_id;
  gint lost_buffer;
  gdouble pan_gain;
  gdouble tilt_gain;
  gdouble deadzone;
  gdouble smooth_alpha;
  gdouble fast_smooth_alpha;
  gdouble fast_error_zone;
  gdouble command_boost_zone;
  gdouble min_active_command;
  gdouble response_gamma;
  gdouble max_command;
  gboolean invert_pan;
  gboolean invert_tilt;
  gboolean rtsp_enable;
  gint rtsp_port;
  gint udp_port;
  gint bitrate;
  gint rtsp_queue_buffers;
  gint rtsp_viewer_latency_ms;
  gchar *infer_config;
  gchar *tracker_config;
  gchar *state_file;
} AppConfig;

typedef struct TrackCandidate {
  NvDsObjectMeta *obj_meta;
  guint64 track_id;
  gint class_id;
  gchar label[64];
  gdouble confidence;
  gdouble left;
  gdouble top;
  gdouble width;
  gdouble height;
  gdouble cx;
  gdouble cy;
  gdouble area;
} TrackCandidate;

typedef struct AppState {
  AppConfig cfg;
  GMainLoop *loop;
  GstElement *pipeline;
  guint frame_number;
  FILE *state_fp;
  gboolean has_locked_target;
  guint64 locked_target_id;
  guint lost_frames;
  gdouble smooth_pan_error;
  gdouble smooth_tilt_error;
  GstRTSPServer *rtsp_server;
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

static gdouble
env_double_default(const gchar *key, gdouble fallback)
{
  const gchar *value = g_getenv(key);
  return (value && *value) ? g_ascii_strtod(value, NULL) : fallback;
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
  cfg->show = env_bool_default("SHOW", FALSE);
  cfg->max_frames = (guint) env_int_default("MAX_FRAMES", 0);
  cfg->target_class_id = env_int_default("TARGET_CLASS_ID", 0);
  cfg->selection = env_strdup_default("SELECTION", "center");
  cfg->target_id = env_int_default("TARGET_ID", -1);
  cfg->lost_buffer = env_int_default("LOST_BUFFER", 15);
  cfg->pan_gain = env_double_default("PAN_GAIN", 0.65);
  cfg->tilt_gain = env_double_default("TILT_GAIN", 0.65);
  cfg->deadzone = env_double_default("DEADZONE", 0.04);
  cfg->smooth_alpha = env_double_default("SMOOTH_ALPHA", 0.35);
  cfg->fast_smooth_alpha = env_double_default("FAST_SMOOTH_ALPHA", 0.75);
  cfg->fast_error_zone = env_double_default("FAST_ERROR_ZONE", 0.18);
  cfg->command_boost_zone = env_double_default("COMMAND_BOOST_ZONE", 0.10);
  cfg->min_active_command = env_double_default("MIN_ACTIVE_COMMAND", 0.18);
  cfg->response_gamma = env_double_default("RESPONSE_GAMMA", 0.65);
  cfg->max_command = env_double_default("MAX_COMMAND", 1.0);
  cfg->invert_pan = env_bool_default("INVERT_PAN", FALSE);
  cfg->invert_tilt = env_bool_default("INVERT_TILT", FALSE);
  cfg->rtsp_enable = env_bool_default("RTSP_ENABLE", TRUE);
  cfg->rtsp_port = env_int_default("RTSP_PORT", DEFAULT_RTSP_PORT);
  cfg->udp_port = env_int_default("UDP_PORT", DEFAULT_UDP_PORT);
  cfg->bitrate = env_int_default("BITRATE", DEFAULT_RTSP_BITRATE);
  cfg->rtsp_queue_buffers = env_int_default("RTSP_QUEUE_BUFFERS", DEFAULT_RTSP_QUEUE_BUFFERS);
  cfg->rtsp_viewer_latency_ms = env_int_default("RTSP_VIEWER_LATENCY_MS", DEFAULT_RTSP_VIEWER_LATENCY_MS);
  cfg->infer_config = env_strdup_default(
      "INFER_CONFIG",
      "/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_320/config/config_infer_primary_yolo26.txt");
  cfg->tracker_config = env_strdup_default(
      "TRACKER_CONFIG",
      "/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_320/config/tracker_config.txt");
  cfg->state_file = env_strdup_default(
      "STATE_FILE",
      "/tmp/deepstream_yolo26_target_control.jsonl");
}

static void
free_config(AppConfig *cfg)
{
  g_free(cfg->selection);
  g_free(cfg->infer_config);
  g_free(cfg->tracker_config);
  g_free(cfg->state_file);
}

static gboolean
start_rtsp_streaming(AppState *state)
{
  GstRTSPMountPoints *mounts = NULL;
  GstRTSPMediaFactory *factory = NULL;
  gchar launch_desc[512];
  gchar port_str[32];

  if (!state->cfg.rtsp_enable) {
    return TRUE;
  }

  g_snprintf(
      launch_desc,
      sizeof(launch_desc),
      "( udpsrc name=pay0 port=%d buffer-size=%u caps=\"application/x-rtp, media=video, "
      "clock-rate=90000, encoding-name=H264, payload=96\" )",
      state->cfg.udp_port,
      512 * 1024);
  g_snprintf(port_str, sizeof(port_str), "%d", state->cfg.rtsp_port);

  state->rtsp_server = gst_rtsp_server_new();
  g_object_set(state->rtsp_server, "service", port_str, NULL);

  mounts = gst_rtsp_server_get_mount_points(state->rtsp_server);
  factory = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(factory, launch_desc);
  gst_rtsp_media_factory_set_shared(factory, TRUE);
  gst_rtsp_mount_points_add_factory(mounts, "/ds-test", factory);
  g_object_unref(mounts);

  gst_rtsp_server_attach(state->rtsp_server, NULL);
  g_print("\n *** prototype_v2: Launched RTSP Streaming at rtsp://localhost:%d/ds-test ***\n\n",
          state->cfg.rtsp_port);
  return TRUE;
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

static gdouble
clamp_value(gdouble value, gdouble limit)
{
  const gdouble abs_limit = fabs(limit);
  if (value > abs_limit) {
    return abs_limit;
  }
  if (value < -abs_limit) {
    return -abs_limit;
  }
  return value;
}

static gdouble
apply_deadzone(gdouble value, gdouble deadzone)
{
  gdouble dz = fabs(deadzone);
  if (dz > 0.99) {
    dz = 0.99;
  }
  if (fabs(value) <= dz) {
    return 0.0;
  }
  return value > 0.0 ? (fabs(value) - dz) / (1.0 - dz) : -((fabs(value) - dz) / (1.0 - dz));
}

static gdouble
smooth_value(gdouble previous, gdouble current, gdouble alpha)
{
  gdouble a = alpha;
  if (a < 0.0) {
    a = 0.0;
  } else if (a > 1.0) {
    a = 1.0;
  }
  return a * current + (1.0 - a) * previous;
}

static gdouble
select_smooth_alpha(gdouble error_mag, const AppConfig *cfg)
{
  gdouble base = cfg->smooth_alpha;
  gdouble fast = cfg->fast_smooth_alpha;
  gdouble zone = fabs(cfg->fast_error_zone);

  if (base < 0.0) {
    base = 0.0;
  } else if (base > 1.0) {
    base = 1.0;
  }
  if (fast < 0.0) {
    fast = 0.0;
  } else if (fast > 1.0) {
    fast = 1.0;
  }
  if (fast < base) {
    fast = base;
  }
  if (zone <= 1e-6) {
    return fast;
  }

  if (error_mag <= 0.0) {
    return base;
  }
  if (error_mag >= zone) {
    return fast;
  }

  return base + ((fast - base) * (error_mag / zone));
}

static gdouble
shape_command(gdouble error_value, gdouble gain, const AppConfig *cfg)
{
  gdouble limit = fabs(cfg->max_command);
  gdouble raw = fabs(error_value) * gain;
  gdouble boost_zone = fabs(cfg->command_boost_zone);
  gdouble min_active = fabs(cfg->min_active_command);
  gdouble gamma = cfg->response_gamma;
  gdouble normalized = 0.0;
  gdouble boosted = 0.0;

  if (limit <= 1e-6 || raw <= 1e-6) {
    return 0.0;
  }

  raw = clamp_value(raw, limit);
  if (boost_zone > limit) {
    boost_zone = limit;
  }
  if (min_active > limit) {
    min_active = limit;
  }
  if (gamma < 0.10) {
    gamma = 0.10;
  } else if (gamma > 3.0) {
    gamma = 3.0;
  }

  if (raw <= boost_zone || boost_zone >= limit) {
    return error_value < 0.0 ? -raw : raw;
  }

  normalized = (raw - boost_zone) / (limit - boost_zone);
  if (normalized < 0.0) {
    normalized = 0.0;
  } else if (normalized > 1.0) {
    normalized = 1.0;
  }

  boosted = min_active + ((limit - min_active) * pow(normalized, gamma));
  if (boosted < raw) {
    boosted = raw;
  }
  return error_value < 0.0 ? -boosted : boosted;
}

static gboolean
load_tracker_config(const gchar *tracker_config_path, GstElement *nvtracker)
{
  gboolean ret = FALSE;
  GError *error = NULL;
  GKeyFile *key_file = g_key_file_new();
  gchar **keys = NULL;
  gchar **key = NULL;

  if (!g_key_file_load_from_file(key_file, tracker_config_path, G_KEY_FILE_NONE, &error)) {
    g_printerr("Failed to load tracker config: %s\n", error->message);
    goto done;
  }

  keys = g_key_file_get_keys(key_file, "tracker", NULL, &error);
  if (error != NULL) {
    g_printerr("Failed to parse tracker config: %s\n", error->message);
    goto done;
  }

  for (key = keys; key && *key; key++) {
    if (!g_strcmp0(*key, "tracker-width")) {
      gint value = g_key_file_get_integer(key_file, "tracker", "tracker-width", &error);
      if (error) goto done;
      g_object_set(G_OBJECT(nvtracker), "tracker-width", value, NULL);
    } else if (!g_strcmp0(*key, "tracker-height")) {
      gint value = g_key_file_get_integer(key_file, "tracker", "tracker-height", &error);
      if (error) goto done;
      g_object_set(G_OBJECT(nvtracker), "tracker-height", value, NULL);
    } else if (!g_strcmp0(*key, "gpu-id")) {
      guint value = (guint) g_key_file_get_integer(key_file, "tracker", "gpu-id", &error);
      if (error) goto done;
      g_object_set(G_OBJECT(nvtracker), "gpu_id", value, NULL);
    } else if (!g_strcmp0(*key, "ll-lib-file")) {
      gchar *value = g_key_file_get_string(key_file, "tracker", "ll-lib-file", &error);
      if (error) goto done;
      g_object_set(G_OBJECT(nvtracker), "ll-lib-file", value, NULL);
      g_free(value);
    } else if (!g_strcmp0(*key, "ll-config-file")) {
      gchar *value = g_key_file_get_string(key_file, "tracker", "ll-config-file", &error);
      if (error) goto done;
      g_object_set(G_OBJECT(nvtracker), "ll-config-file", value, NULL);
      g_free(value);
    } else if (!g_strcmp0(*key, "display-tracking-id")) {
      gboolean value = g_key_file_get_boolean(key_file, "tracker", "display-tracking-id", &error);
      if (error) goto done;
      g_object_set(G_OBJECT(nvtracker), "display-tracking-id", value, NULL);
    }
  }

  ret = TRUE;

done:
  if (error) {
    g_printerr("Tracker config error: %s\n", error->message);
    g_error_free(error);
  }
  if (keys) {
    g_strfreev(keys);
  }
  if (key_file) {
    g_key_file_free(key_file);
  }
  return ret;
}

static guint
extract_candidates(NvDsFrameMeta *frame_meta, const AppConfig *cfg, TrackCandidate *candidates, guint max_candidates)
{
  NvDsMetaList *l_obj = NULL;
  guint count = 0;

  for (l_obj = frame_meta->obj_meta_list; l_obj != NULL && count < max_candidates; l_obj = l_obj->next) {
    NvDsObjectMeta *obj_meta = (NvDsObjectMeta *) l_obj->data;
    if (obj_meta->class_id != cfg->target_class_id) {
      continue;
    }

    TrackCandidate *candidate = &candidates[count++];
    memset(candidate, 0, sizeof(*candidate));
    candidate->obj_meta = obj_meta;
    candidate->track_id = (guint64) obj_meta->object_id;
    candidate->class_id = obj_meta->class_id;
    g_strlcpy(candidate->label,
              (obj_meta->obj_label && *obj_meta->obj_label) ? obj_meta->obj_label : "unknown",
              sizeof(candidate->label));
    candidate->confidence = obj_meta->confidence;
    candidate->left = obj_meta->rect_params.left;
    candidate->top = obj_meta->rect_params.top;
    candidate->width = obj_meta->rect_params.width;
    candidate->height = obj_meta->rect_params.height;
    candidate->cx = candidate->left + candidate->width / 2.0;
    candidate->cy = candidate->top + candidate->height / 2.0;
    candidate->area = candidate->width * candidate->height;
  }

  return count;
}

static void
set_osd_color(NvOSD_ColorParams *color, gdouble r, gdouble g, gdouble b, gdouble a)
{
  color->red = r;
  color->green = g;
  color->blue = b;
  color->alpha = a;
}

static void
style_candidate_box(TrackCandidate *candidate, gboolean selected)
{
  NvDsObjectMeta *obj_meta = candidate ? candidate->obj_meta : NULL;
  if (obj_meta == NULL) {
    return;
  }

  obj_meta->rect_params.has_bg_color = 0;
  obj_meta->text_params.set_bg_clr = 1;
  obj_meta->text_params.font_params.font_size = selected ? 14 : 12;

  if (selected) {
    obj_meta->rect_params.border_width = 4;
    set_osd_color(&obj_meta->rect_params.border_color, 0.10, 0.95, 0.20, 1.0);
    set_osd_color(&obj_meta->text_params.font_params.font_color, 0.05, 0.05, 0.05, 1.0);
    set_osd_color(&obj_meta->text_params.text_bg_clr, 0.10, 0.95, 0.20, 1.0);
  } else {
    obj_meta->rect_params.border_width = 2;
    set_osd_color(&obj_meta->rect_params.border_color, 0.78, 0.78, 0.78, 1.0);
    set_osd_color(&obj_meta->text_params.font_params.font_color, 1.0, 1.0, 1.0, 1.0);
    set_osd_color(&obj_meta->text_params.text_bg_clr, 0.25, 0.25, 0.25, 0.85);
  }
}

static void
setup_text(NvOSD_TextParams *txt, gint x, gint y, const gchar *text)
{
  txt->display_text = g_strdup(text);
  txt->x_offset = x;
  txt->y_offset = y;
  txt->font_params.font_name = "Serif";
  txt->font_params.font_size = 14;
  set_osd_color(&txt->font_params.font_color, 0.10, 0.95, 0.20, 1.0);
  txt->set_bg_clr = 1;
  set_osd_color(&txt->text_bg_clr, 0.0, 0.0, 0.0, 0.75);
}

static void
setup_line(NvOSD_LineParams *line, guint x1, guint y1, guint x2, guint y2, guint width,
           gdouble r, gdouble g, gdouble b, gdouble a)
{
  line->x1 = x1;
  line->y1 = y1;
  line->x2 = x2;
  line->y2 = y2;
  line->line_width = width;
  set_osd_color(&line->line_color, r, g, b, a);
}

static void
add_overlay_meta(
    NvDsBatchMeta *batch_meta,
    NvDsFrameMeta *frame_meta,
    const AppState *state,
    const TrackCandidate *selected,
    guint visible_tracks,
    const gchar *status,
    const gchar *command_status,
    gdouble dx_norm,
    gdouble dy_norm,
    gdouble pan_cmd,
    gdouble tilt_cmd)
{
  NvDsDisplayMeta *display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
  gchar line1[160];
  gchar line2[160];
  gchar line3[160];
  guint cx = (guint) (state->cfg.width / 2);
  guint cy = (guint) (state->cfg.height / 2);

  if (selected) {
    g_snprintf(
        line1,
        sizeof(line1),
        "%s | target id:%" G_GUINT64_FORMAT " %s | tracks %u | lost %u",
        status,
        selected->track_id,
        selected->label,
        visible_tracks,
        state->lost_frames);
    g_snprintf(line2, sizeof(line2), "err %+.2f,%+.2f | cmd %+.2f,%+.2f", dx_norm, dy_norm, pan_cmd, tilt_cmd);
  } else {
    g_snprintf(line1, sizeof(line1), "%s | target none | tracks %u | lost %u", status, visible_tracks, state->lost_frames);
    g_snprintf(line2, sizeof(line2), "err n/a | cmd %+.2f,%+.2f", pan_cmd, tilt_cmd);
  }
  g_snprintf(line3, sizeof(line3), "prototype_v2 | tracking+control | %s", command_status);

  display_meta->num_labels = 3;
  setup_text(&display_meta->text_params[0], 14, 18, line1);
  setup_text(&display_meta->text_params[1], 14, 42, line2);
  setup_text(&display_meta->text_params[2], 14, 66, line3);

  display_meta->num_lines = selected ? 3 : 2;
  setup_line(&display_meta->line_params[0], cx - 18, cy, cx + 18, cy, 2, 0.0, 1.0, 1.0, 1.0);
  setup_line(&display_meta->line_params[1], cx, cy - 18, cx, cy + 18, 2, 0.0, 1.0, 1.0, 1.0);
  if (selected) {
    setup_line(
        &display_meta->line_params[2],
        cx,
        cy,
        (guint) selected->cx,
        (guint) selected->cy,
        3,
        1.0,
        0.95,
        0.0,
        1.0);
  }

  nvds_add_display_meta_to_frame(frame_meta, display_meta);
}

static TrackCandidate *
find_candidate_by_id(TrackCandidate *candidates, guint count, guint64 track_id)
{
  guint i;
  for (i = 0; i < count; i++) {
    if (candidates[i].track_id == track_id) {
      return &candidates[i];
    }
  }
  return NULL;
}

static TrackCandidate *
choose_candidate(TrackCandidate *candidates, guint count, const AppConfig *cfg)
{
  guint i;
  TrackCandidate *best = NULL;

  if (count == 0) {
    return NULL;
  }

  if (g_strcmp0(cfg->selection, "manual-id") == 0 && cfg->target_id >= 0) {
    return find_candidate_by_id(candidates, count, (guint64) cfg->target_id);
  }

  if (g_strcmp0(cfg->selection, "largest") == 0) {
    for (i = 0; i < count; i++) {
      if (!best || candidates[i].area > best->area) {
        best = &candidates[i];
      }
    }
    return best;
  }

  for (i = 0; i < count; i++) {
    const gdouble dx = candidates[i].cx - (cfg->width / 2.0);
    const gdouble dy = candidates[i].cy - (cfg->height / 2.0);
    const gdouble dist2 = dx * dx + dy * dy;
    if (!best) {
      best = &candidates[i];
      continue;
    }
    const gdouble best_dx = best->cx - (cfg->width / 2.0);
    const gdouble best_dy = best->cy - (cfg->height / 2.0);
    const gdouble best_dist2 = best_dx * best_dx + best_dy * best_dy;
    if (dist2 < best_dist2) {
      best = &candidates[i];
    }
  }

  return best;
}

static void
write_state_json(FILE *fp,
                 guint frame_idx,
                 const gchar *status,
                 const gchar *command_status,
                 guint visible_tracks,
                 guint lost_frames,
                 const TrackCandidate *target,
                 gdouble dx_norm,
                 gdouble dy_norm,
                 gdouble smooth_dx,
                 gdouble smooth_dy,
                 gdouble pan_cmd,
                 gdouble tilt_cmd)
{
  fprintf(fp,
          "{\"frame_idx\":%u,\"status\":\"%s\",\"command_status\":\"%s\",\"visible_tracks\":%u,"
          "\"lost_frames\":%u,\"target_id\":",
          frame_idx,
          status,
          command_status,
          visible_tracks,
          lost_frames);

  if (target) {
    fprintf(fp,
            "%" G_GUINT64_FORMAT ",\"class_id\":%d,\"class_name\":\"%s\",\"confidence\":%.4f,"
            "\"dx_norm\":%.4f,\"dy_norm\":%.4f,",
            target->track_id,
            target->class_id,
            target->label,
            target->confidence,
            dx_norm,
            dy_norm);
  } else {
    fprintf(fp, "null,\"class_id\":null,\"class_name\":null,\"confidence\":null,\"dx_norm\":null,\"dy_norm\":null,");
  }

  fprintf(fp,
          "\"smooth_dx_norm\":%.4f,\"smooth_dy_norm\":%.4f,\"pan_cmd\":%.4f,\"tilt_cmd\":%.4f}\n",
          smooth_dx,
          smooth_dy,
          pan_cmd,
          tilt_cmd);
  fflush(fp);
}

static GstPadProbeReturn
tracker_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
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
    TrackCandidate candidates[MAX_CANDIDATES];
    TrackCandidate *selected = NULL;
    gchar target_id_buf[32];
    guint visible_tracks = extract_candidates(frame_meta, &state->cfg, candidates, MAX_CANDIDATES);
    const gchar *status = "tracked";
    const gchar *command_status = "no_target";
    gdouble dx_norm = 0.0;
    gdouble dy_norm = 0.0;
    gdouble pan_cmd = 0.0;
    gdouble tilt_cmd = 0.0;

    if (state->has_locked_target) {
      selected = find_candidate_by_id(candidates, visible_tracks, state->locked_target_id);
      if (selected) {
        status = state->lost_frames > 0 ? "reacquired" : "tracked";
        state->lost_frames = 0;
      } else {
        state->lost_frames++;
        status = "lost";
        if (state->lost_frames > (guint) state->cfg.lost_buffer) {
          state->has_locked_target = FALSE;
        }
      }
    }

    if (!state->has_locked_target) {
      selected = choose_candidate(candidates, visible_tracks, &state->cfg);
      if (selected) {
        state->locked_target_id = selected->track_id;
        state->has_locked_target = TRUE;
        state->lost_frames = 0;
        status = "acquired";
      } else {
        status = "searching";
      }
    }

    if (selected) {
      dx_norm = (selected->cx - (state->cfg.width / 2.0)) / (state->cfg.width / 2.0);
      dy_norm = (selected->cy - (state->cfg.height / 2.0)) / (state->cfg.height / 2.0);
      {
        gdouble pan_error = apply_deadzone(dx_norm, state->cfg.deadzone);
        gdouble tilt_error = apply_deadzone(dy_norm, state->cfg.deadzone);
        gdouble pan_alpha = select_smooth_alpha(fabs(pan_error), &state->cfg);
        gdouble tilt_alpha = select_smooth_alpha(fabs(tilt_error), &state->cfg);

        state->smooth_pan_error = smooth_value(state->smooth_pan_error, pan_error, pan_alpha);
        state->smooth_tilt_error = smooth_value(state->smooth_tilt_error, tilt_error, tilt_alpha);
      }
      pan_cmd = shape_command(state->smooth_pan_error, state->cfg.pan_gain, &state->cfg);
      tilt_cmd = shape_command(state->smooth_tilt_error, state->cfg.tilt_gain, &state->cfg);
      if (state->cfg.invert_pan) {
        pan_cmd *= -1.0;
      }
      if (state->cfg.invert_tilt) {
        tilt_cmd *= -1.0;
      }
      command_status = (pan_cmd == 0.0 && tilt_cmd == 0.0) ? "deadzone" : "active";
    } else {
      state->smooth_pan_error = smooth_value(state->smooth_pan_error, 0.0, state->cfg.smooth_alpha);
      state->smooth_tilt_error = smooth_value(state->smooth_tilt_error, 0.0, state->cfg.smooth_alpha);
    }

    if (selected) {
      g_snprintf(target_id_buf, sizeof(target_id_buf), "%" G_GUINT64_FORMAT, selected->track_id);
    } else {
      g_strlcpy(target_id_buf, "n/a", sizeof(target_id_buf));
    }

    {
      guint i;
      for (i = 0; i < visible_tracks; i++) {
        style_candidate_box(&candidates[i], selected != NULL && candidates[i].track_id == selected->track_id);
      }
    }
    add_overlay_meta(
        batch_meta,
        frame_meta,
        state,
        selected,
        visible_tracks,
        status,
        command_status,
        selected ? dx_norm : 0.0,
        selected ? dy_norm : 0.0,
        pan_cmd,
        tilt_cmd);

    g_print("Frame %u | status=%s | tracks=%u | target=%s | id=%s | err=%+.2f,%+.2f | cmd=%+.2f,%+.2f\n",
            state->frame_number,
            status,
            visible_tracks,
            selected ? selected->label : "none",
            target_id_buf,
            selected ? dx_norm : 0.0,
            selected ? dy_norm : 0.0,
            pan_cmd,
            tilt_cmd);

    if (state->state_fp) {
      write_state_json(
          state->state_fp,
          state->frame_number,
          status,
          command_status,
          visible_tracks,
          state->lost_frames,
          selected,
          dx_norm,
          dy_norm,
          state->smooth_pan_error,
          state->smooth_tilt_error,
          pan_cmd,
          tilt_cmd);
    }

    state->frame_number++;
    if (state->cfg.max_frames > 0 && state->frame_number >= state->cfg.max_frames) {
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

  AppState state;
  GstElement *pipeline = NULL;
  GstElement *source = NULL;
  GstElement *source_caps = NULL;
  GstElement *source_conv = NULL;
  GstElement *streammux = NULL;
  GstElement *pgie = NULL;
  GstElement *nvtracker = NULL;
  GstElement *nvvidconv = NULL;
  GstElement *nvosd = NULL;
  GstElement *tee = NULL;
  GstElement *display_queue = NULL;
  GstElement *rtsp_queue = NULL;
  GstElement *rtsp_conv = NULL;
  GstElement *rtsp_caps = NULL;
  GstElement *encoder = NULL;
  GstElement *h264parse = NULL;
  GstElement *rtppay = NULL;
  GstElement *udpsink = NULL;
  GstElement *sink = NULL;
  GstBus *bus = NULL;
  GstCaps *caps = NULL;
  GstCaps *encoder_caps = NULL;
  gchar *caps_str = NULL;
  gchar *encoder_caps_str = NULL;
  GstPad *mux_sink_pad = NULL;
  GstPad *source_src_pad = NULL;
  GstPad *tracker_src_pad = NULL;
  GstPad *tee_display_pad = NULL;
  GstPad *tee_rtsp_pad = NULL;
  GstPad *display_queue_pad = NULL;
  GstPad *rtsp_queue_pad = NULL;
  guint bus_watch_id = 0;

  memset(&state, 0, sizeof(state));
  load_config(&state.cfg);

  gst_init(NULL, NULL);
  state.loop = g_main_loop_new(NULL, FALSE);

  pipeline = gst_pipeline_new("deepstream-yolo26-rtsp-target-control");
  source = gst_element_factory_make("nvarguscamerasrc", "csi-source");
  source_caps = gst_element_factory_make("capsfilter", "source-caps");
  source_conv = gst_element_factory_make("nvvideoconvert", "source-convert");
  streammux = gst_element_factory_make("nvstreammux", "stream-muxer");
  pgie = gst_element_factory_make("nvinfer", "primary-infer");
  nvtracker = gst_element_factory_make("nvtracker", "tracker");
  nvvidconv = gst_element_factory_make("nvvideoconvert", "video-convert");
  nvosd = gst_element_factory_make("nvdsosd", "osd");
  tee = gst_element_factory_make("tee", "branch-tee");
  display_queue = gst_element_factory_make("queue", "display-queue");
  rtsp_queue = gst_element_factory_make("queue", "rtsp-queue");
  rtsp_conv = gst_element_factory_make("nvvideoconvert", "rtsp-convert");
  rtsp_caps = gst_element_factory_make("capsfilter", "rtsp-caps");
  encoder = gst_element_factory_make("nvv4l2h264enc", "rtsp-encoder");
  h264parse = gst_element_factory_make("h264parse", "rtsp-h264parse");
  rtppay = gst_element_factory_make("rtph264pay", "rtsp-payloader");
  udpsink = gst_element_factory_make("udpsink", "rtsp-udpsink");
  sink = gst_element_factory_make(state.cfg.show ? "nv3dsink" : "fakesink", state.cfg.show ? "display" : "fakesink");

  if (!pipeline || !source || !source_caps || !source_conv || !streammux || !pgie || !nvtracker ||
      !nvvidconv || !nvosd || !tee || !display_queue || !rtsp_queue || !rtsp_conv || !rtsp_caps ||
      !encoder || !h264parse || !rtppay || !udpsink || !sink) {
    g_printerr("Failed to create one or more GStreamer elements.\n");
    free_config(&state.cfg);
    return 1;
  }

  state.pipeline = pipeline;
  if (state.cfg.state_file && *state.cfg.state_file) {
    state.state_fp = fopen(state.cfg.state_file, "w");
    if (!state.state_fp) {
      g_printerr("Failed to open STATE_FILE: %s\n", state.cfg.state_file);
      free_config(&state.cfg);
      return 1;
    }
  }

  caps_str = g_strdup_printf(
      "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/%d, format=(string)NV12",
      state.cfg.width,
      state.cfg.height,
      state.cfg.fps_n,
      state.cfg.fps_d);
  caps = gst_caps_from_string(caps_str);
  g_free(caps_str);

  g_object_set(G_OBJECT(source), "sensor-id", state.cfg.sensor_id, NULL);
  g_object_set(G_OBJECT(source_caps), "caps", caps, NULL);
  g_object_set(G_OBJECT(streammux),
               "width", state.cfg.width,
               "height", state.cfg.height,
               "batch-size", 1,
               "batched-push-timeout", 33000,
               "live-source", TRUE,
               NULL);
  g_object_set(G_OBJECT(pgie), "config-file-path", state.cfg.infer_config, NULL);
  if (!load_tracker_config(state.cfg.tracker_config, nvtracker)) {
    goto cleanup;
  }
  if (state.cfg.show) {
    g_object_set(G_OBJECT(sink), "sync", FALSE, NULL);
  }
  g_object_set(G_OBJECT(display_queue),
               "leaky", 2,
               "max-size-buffers", 2,
               "max-size-bytes", 0,
               "max-size-time", 0,
               NULL);
  g_object_set(G_OBJECT(rtsp_queue),
               "leaky", 2,
               "max-size-buffers", MAX(1, state.cfg.rtsp_queue_buffers),
               "max-size-bytes", 0,
               "max-size-time", 0,
               NULL);
  encoder_caps_str = g_strdup("video/x-raw(memory:NVMM), format=(string)I420");
  encoder_caps = gst_caps_from_string(encoder_caps_str);
  g_free(encoder_caps_str);
  g_object_set(G_OBJECT(rtsp_caps), "caps", encoder_caps, NULL);
  g_object_set(G_OBJECT(encoder),
               "bitrate", state.cfg.bitrate,
               "control-rate", 1,
               "preset-level", 1,
               "maxperf-enable", TRUE,
               "iframeinterval", 15,
               "idrinterval", 15,
               "insert-sps-pps", TRUE,
               "insert-vui", TRUE,
               NULL);
  g_object_set(G_OBJECT(rtppay), "pt", 96, "config-interval", 1, NULL);
  g_object_set(G_OBJECT(udpsink),
               "host", "127.0.0.1",
               "port", state.cfg.udp_port,
               "async", FALSE,
               "sync", FALSE,
               NULL);

  gst_bin_add_many(GST_BIN(pipeline),
                   source, source_caps, source_conv, streammux, pgie, nvtracker, nvvidconv, nvosd,
                   tee, display_queue, rtsp_queue, rtsp_conv, rtsp_caps, encoder, h264parse, rtppay,
                   udpsink, sink, NULL);

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

  if (!gst_element_link_many(streammux, pgie, nvtracker, nvvidconv, nvosd, tee, NULL)) {
    g_printerr("Failed to link inference/tracker/tee chain.\n");
    goto cleanup;
  }

  if (!gst_element_link_many(display_queue, sink, NULL)) {
    g_printerr("Failed to link display branch.\n");
    goto cleanup;
  }

  if (!gst_element_link_many(rtsp_queue, rtsp_conv, rtsp_caps, encoder, h264parse, rtppay, udpsink, NULL)) {
    g_printerr("Failed to link RTSP branch.\n");
    goto cleanup;
  }

  tee_display_pad = gst_element_get_request_pad(tee, "src_%u");
  tee_rtsp_pad = gst_element_get_request_pad(tee, "src_%u");
  display_queue_pad = gst_element_get_static_pad(display_queue, "sink");
  rtsp_queue_pad = gst_element_get_static_pad(rtsp_queue, "sink");
  if (!tee_display_pad || !tee_rtsp_pad || !display_queue_pad || !rtsp_queue_pad) {
    g_printerr("Failed to get tee branch pads.\n");
    goto cleanup;
  }
  if (gst_pad_link(tee_display_pad, display_queue_pad) != GST_PAD_LINK_OK ||
      gst_pad_link(tee_rtsp_pad, rtsp_queue_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link tee branches.\n");
    goto cleanup;
  }

  tracker_src_pad = gst_element_get_static_pad(nvtracker, "src");
  if (!tracker_src_pad) {
    g_printerr("Failed to get tracker src pad.\n");
    goto cleanup;
  }
  gst_pad_add_probe(tracker_src_pad, GST_PAD_PROBE_TYPE_BUFFER, tracker_src_pad_buffer_probe, &state, NULL);

  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, bus_call, &state);
  gst_object_unref(bus);

  if (!start_rtsp_streaming(&state)) {
    goto cleanup;
  }

  g_print("Starting prototype_v2 DeepStream RTSP + target-control preview on CSI sensor %d\n", state.cfg.sensor_id);
  g_print("Infer config: %s\n", state.cfg.infer_config);
  g_print("Tracker config: %s\n", state.cfg.tracker_config);
  g_print("State file: %s\n", state.cfg.state_file);
  g_print("RTSP URL: rtsp://localhost:%d/ds-test\n", state.cfg.rtsp_port);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(state.loop);

cleanup:
  if (bus_watch_id > 0) {
    g_source_remove(bus_watch_id);
  }
  if (tracker_src_pad) {
    gst_object_unref(tracker_src_pad);
  }
  if (tee_display_pad) {
    gst_object_unref(tee_display_pad);
  }
  if (tee_rtsp_pad) {
    gst_object_unref(tee_rtsp_pad);
  }
  if (display_queue_pad) {
    gst_object_unref(display_queue_pad);
  }
  if (rtsp_queue_pad) {
    gst_object_unref(rtsp_queue_pad);
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
  if (encoder_caps) {
    gst_caps_unref(encoder_caps);
  }
  if (pipeline) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
  }
  if (state.rtsp_server) {
    g_object_unref(state.rtsp_server);
  }
  if (state.state_fp) {
    fclose(state.state_fp);
  }
  if (state.loop) {
    g_main_loop_unref(state.loop);
  }
  free_config(&state.cfg);
  return 0;
}
