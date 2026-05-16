// Separate DeepStream prototype_v2 combined RTSP + tracked-metadata control preview.
// This app keeps video streaming and metadata-only control preview in one pipeline.

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp/gstrtspmessage.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "gstnvdsmeta.h"

#define DEFAULT_WIDTH 2028
#define DEFAULT_HEIGHT 1112
#define DEFAULT_FPS_N 60
#define DEFAULT_FPS_D 1
#define DEFAULT_TRACKER_WIDTH 960
#define DEFAULT_TRACKER_HEIGHT 544
#define DEFAULT_RTSP_PORT 8554
#define DEFAULT_UDP_PORT 5400
#define DEFAULT_RTSP_BITRATE 8000000
#define DEFAULT_RAW_RECORD_BITRATE 40000000
#define DEFAULT_VIDEO_QUEUE_BUFFERS 2
#define DEFAULT_RTSP_QUEUE_BUFFERS 4
#define DEFAULT_RAW_RECORD_QUEUE_BUFFERS 8
#define DEFAULT_RTSP_VIEWER_LATENCY_MS 80
#define METADATA_IPC_MAGIC 0x5056324dU
#define METADATA_IPC_VERSION 3U
#define METADATA_IPC_PAYLOAD_MAX 8192U
#define MAX_CANDIDATES 256
#define FRAME_SNAPSHOT_RING_SIZE 512

typedef struct AppConfig {
  gint sensor_id;
  gint width;
  gint height;
  gint fps_n;
  gint fps_d;
  gboolean show;
  gboolean print_frame_logs;
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
  gdouble pan_feedforward_gain;
  gdouble tilt_feedforward_gain;
  gdouble feedforward_alpha;
  gdouble feedforward_limit;
  gdouble feedforward_activation_zone;
  gdouble max_command;
  gboolean invert_pan;
  gboolean invert_tilt;
  gboolean rtsp_enable;
  gint rtsp_port;
  gint udp_port;
  gint bitrate;
  gboolean raw_record_enable;
  gint raw_record_bitrate;
  gint video_queue_buffers;
  gint rtsp_queue_buffers;
  gint raw_record_queue_buffers;
  gint rtsp_viewer_latency_ms;
  gboolean monitoring_errors_fatal;
  gchar *rtsp_mount;
  gchar *raw_record_file;
  gchar *infer_config;
  gchar *tracker_config;
  gchar *metadata_ipc_file;
  gchar *state_file;
  gboolean state_file_flush;
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

typedef struct FrameTargetSnapshot {
  gboolean has_target;
  guint64 target_id;
  gint class_id;
  gchar label[64];
  gdouble confidence;
  gdouble cx;
  gdouble cy;
  guint visible_tracks;
  guint lost_frames;
  gchar status[32];
  gchar command_status[32];
  gdouble dx_norm;
  gdouble dy_norm;
} FrameTargetSnapshot;

typedef struct FrameTargetSnapshotSlot {
  gboolean valid;
  guint frame_idx;
  FrameTargetSnapshot snapshot;
} FrameTargetSnapshotSlot;

typedef struct AppState {
  AppConfig cfg;
  GMainLoop *loop;
  GstElement *pipeline;
  guint frame_number;
  FILE *state_fp;
  gint metadata_ipc_fd;
  gpointer metadata_ipc_map;
  gsize metadata_ipc_size;
  GMutex latency_stage3_branch_lock;
  GMutex frame_snapshot_lock;
  FrameTargetSnapshotSlot frame_snapshots[FRAME_SNAPSHOT_RING_SIZE];
  guint video_queue_frame_idx;
  gint64 video_queue_src_mono_ns;
  guint nvosd_frame_idx;
  gint64 nvosd_src_mono_ns;
  guint display_queue_frame_idx;
  gint64 display_queue_src_mono_ns;
  guint rtsp_queue_frame_idx;
  gint64 rtsp_queue_src_mono_ns;
  gboolean has_locked_target;
  guint64 locked_target_id;
  guint lost_frames;
  gdouble smooth_pan_error;
  gdouble smooth_tilt_error;
  gdouble smooth_pan_feedforward_rate;
  gdouble smooth_tilt_feedforward_rate;
  gdouble last_pan_error;
  gdouble last_tilt_error;
  gint64 last_control_time_usec;
  GstRTSPServer *rtsp_server;
} AppState;

typedef struct MetadataIpcRegion {
  guint32 magic;
  guint32 version;
  guint32 payload_capacity;
  guint32 reserved0;
  guint64 sequence;
  guint64 frame_idx;
  gint64 ds_write_mono_ns;
  guint32 payload_len;
  guint32 reserved1;
  gchar payload[METADATA_IPC_PAYLOAD_MAX];
} MetadataIpcRegion;

G_STATIC_ASSERT(sizeof(MetadataIpcRegion) == 48 + METADATA_IPC_PAYLOAD_MAX);

typedef struct _MetadataIpcPayload {
  gint64 c_probe_start_mono_ns;
  gint64 c_control_done_mono_ns;
  gint64 mux_src_mono_ns;
  gint64 pgie_src_mono_ns;
  gint64 tracker_src_mono_ns;
  guint64 frame_pts_ns;
  guint64 frame_ntp_ns;
  guint32 video_queue_frame_idx;
  gint64 video_queue_src_mono_ns;
  guint32 nvosd_frame_idx;
  gint64 nvosd_src_mono_ns;
  guint32 display_queue_frame_idx;
  gint64 display_queue_src_mono_ns;
  guint32 rtsp_queue_frame_idx;
  gint64 rtsp_queue_src_mono_ns;
  guint32 visible_tracks;
  guint32 lost_frames;
  guint8 has_target;
  guint8 reserved0[7];
  guint64 target_id;
  gint32 class_id;
  guint32 reserved1;
  gdouble confidence;
  gdouble dx_norm;
  gdouble dy_norm;
  gchar class_name[64];
  gchar status[32];
  gchar command_status[32];
} __attribute__((__packed__)) MetadataIpcPayload;

G_STATIC_ASSERT(sizeof(MetadataIpcPayload) == 288);
G_STATIC_ASSERT(sizeof(MetadataIpcPayload) <= METADATA_IPC_PAYLOAD_MAX);

typedef struct _Mk15RTSPClient Mk15RTSPClient;
typedef struct _Mk15RTSPClientClass Mk15RTSPClientClass;
typedef struct _Mk15RTSPServer Mk15RTSPServer;
typedef struct _Mk15RTSPServerClass Mk15RTSPServerClass;

struct _Mk15RTSPClient {
  GstRTSPClient parent;
};

struct _Mk15RTSPClientClass {
  GstRTSPClientClass parent_class;
};

struct _Mk15RTSPServer {
  GstRTSPServer parent;
};

struct _Mk15RTSPServerClass {
  GstRTSPServerClass parent_class;
};

G_DEFINE_TYPE(Mk15RTSPClient, mk15_rtsp_client, GST_TYPE_RTSP_CLIENT)
G_DEFINE_TYPE(Mk15RTSPServer, mk15_rtsp_server, GST_TYPE_RTSP_SERVER)

/* ===== LATENCY STAGE 3 ADDED START =====
 * Stage 3 records coarse DeepStream element timing on the same GstBuffer.
 * These timestamps are best-effort: they measure when the buffer passes selected
 * GStreamer pads, not the exact internal GPU kernel duration.
 */
typedef struct LatencyStage3BufferTimes {
  gint64 mux_src_mono_ns;
  gint64 pgie_src_mono_ns;
  gint64 video_queue_src_mono_ns;
  gint64 nvosd_src_mono_ns;
  gint64 display_queue_src_mono_ns;
  gint64 rtsp_queue_src_mono_ns;
} LatencyStage3BufferTimes;

static GQuark
latency_stage3_quark(void)
{
  static GQuark quark = 0;
  if (G_UNLIKELY(quark == 0)) {
    quark = g_quark_from_static_string("prototype-v2-latency-stage3-times");
  }
  return quark;
}

static gint64
latency_mono_ns(void)
{
  return g_get_monotonic_time() * 1000LL;
}

static LatencyStage3BufferTimes *
latency_stage3_get_or_create(GstBuffer *buf)
{
  LatencyStage3BufferTimes *times = NULL;
  if (!buf) {
    return NULL;
  }
  times = (LatencyStage3BufferTimes *) gst_mini_object_get_qdata(
      GST_MINI_OBJECT(buf), latency_stage3_quark());
  if (!times) {
    times = g_new0(LatencyStage3BufferTimes, 1);
    gst_mini_object_set_qdata(
        GST_MINI_OBJECT(buf), latency_stage3_quark(), times, g_free);
  }
  return times;
}

static gboolean
latency_stage3_extract_frame_idx(GstBuffer *buf, guint *frame_idx_out)
{
  NvDsBatchMeta *batch_meta = NULL;
  NvDsFrameMeta *frame_meta = NULL;

  if (!buf || !frame_idx_out) {
    return FALSE;
  }
  batch_meta = gst_buffer_get_nvds_batch_meta(buf);
  if (!batch_meta || !batch_meta->frame_meta_list) {
    return FALSE;
  }
  frame_meta = (NvDsFrameMeta *) batch_meta->frame_meta_list->data;
  if (!frame_meta) {
    return FALSE;
  }
  *frame_idx_out = frame_meta->frame_num;
  return TRUE;
}

static void
latency_stage3_update_branch_timing(AppState *state,
                                    guint *frame_idx_slot,
                                    gint64 *mono_ns_slot,
                                    guint frame_idx,
                                    gint64 mono_ns)
{
  if (!state) {
    return;
  }
  g_mutex_lock(&state->latency_stage3_branch_lock);
  *frame_idx_slot = frame_idx;
  *mono_ns_slot = mono_ns;
  g_mutex_unlock(&state->latency_stage3_branch_lock);
}

static void
latency_stage3_snapshot_branch_timing(AppState *state,
                                      guint *video_queue_frame_idx,
                                      gint64 *video_queue_src_mono_ns,
                                      guint *nvosd_frame_idx,
                                      gint64 *nvosd_src_mono_ns,
                                      guint *display_queue_frame_idx,
                                      gint64 *display_queue_src_mono_ns,
                                      guint *rtsp_queue_frame_idx,
                                      gint64 *rtsp_queue_src_mono_ns)
{
  if (!state) {
    return;
  }
  g_mutex_lock(&state->latency_stage3_branch_lock);
  if (video_queue_frame_idx) {
    *video_queue_frame_idx = state->video_queue_frame_idx;
  }
  if (video_queue_src_mono_ns) {
    *video_queue_src_mono_ns = state->video_queue_src_mono_ns;
  }
  if (nvosd_frame_idx) {
    *nvosd_frame_idx = state->nvosd_frame_idx;
  }
  if (nvosd_src_mono_ns) {
    *nvosd_src_mono_ns = state->nvosd_src_mono_ns;
  }
  if (display_queue_frame_idx) {
    *display_queue_frame_idx = state->display_queue_frame_idx;
  }
  if (display_queue_src_mono_ns) {
    *display_queue_src_mono_ns = state->display_queue_src_mono_ns;
  }
  if (rtsp_queue_frame_idx) {
    *rtsp_queue_frame_idx = state->rtsp_queue_frame_idx;
  }
  if (rtsp_queue_src_mono_ns) {
    *rtsp_queue_src_mono_ns = state->rtsp_queue_src_mono_ns;
  }
  g_mutex_unlock(&state->latency_stage3_branch_lock);
}
/* ===== LATENCY STAGE 3 ADDED END ===== */

static void
store_frame_target_snapshot(AppState *state, guint frame_idx, const FrameTargetSnapshot *snapshot)
{
  FrameTargetSnapshotSlot *slot = NULL;

  if (!state || !snapshot) {
    return;
  }

  slot = &state->frame_snapshots[frame_idx % FRAME_SNAPSHOT_RING_SIZE];
  g_mutex_lock(&state->frame_snapshot_lock);
  slot->valid = TRUE;
  slot->frame_idx = frame_idx;
  slot->snapshot = *snapshot;
  g_mutex_unlock(&state->frame_snapshot_lock);
}

static gboolean
lookup_frame_target_snapshot(AppState *state, guint frame_idx, FrameTargetSnapshot *snapshot_out)
{
  FrameTargetSnapshotSlot *slot = NULL;
  gboolean found = FALSE;

  if (!state || !snapshot_out) {
    return FALSE;
  }

  slot = &state->frame_snapshots[frame_idx % FRAME_SNAPSHOT_RING_SIZE];
  g_mutex_lock(&state->frame_snapshot_lock);
  if (slot->valid && slot->frame_idx == frame_idx) {
    *snapshot_out = slot->snapshot;
    found = TRUE;
  }
  g_mutex_unlock(&state->frame_snapshot_lock);
  return found;
}

static gchar *
env_strdup_default(const gchar *key, const gchar *fallback)
{
  const gchar *value = g_getenv(key);
  return g_strdup((value && *value) ? value : fallback);
}

static gchar *
env_strdup_raw(const gchar *key)
{
  const gchar *value = g_getenv(key);
  return value ? g_strdup(value) : NULL;
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

static gboolean
normalize_play_range(GstRTSPMessage *request)
{
  gchar *range = NULL;
  gchar *trimmed = NULL;
  gchar *fixed = NULL;
  gboolean changed = FALSE;

  if (gst_rtsp_message_get_header(request, GST_RTSP_HDR_RANGE, &range, 0) != GST_RTSP_OK ||
      range == NULL) {
    return FALSE;
  }

  trimmed = g_strdup(range);
  g_strstrip(trimmed);
  if (g_str_has_prefix(trimmed, "npt=") ||
      g_str_has_prefix(trimmed, "clock=") ||
      g_str_has_prefix(trimmed, "smpte=")) {
    g_free(trimmed);
    return FALSE;
  }

  if (!g_regex_match_simple("^[0-9]+(?:\\.[0-9]+)?-$", trimmed, 0, 0)) {
    g_free(trimmed);
    return FALSE;
  }

  fixed = g_strdup_printf("npt=%s", trimmed);
  gst_rtsp_message_remove_header(request, GST_RTSP_HDR_RANGE, 0);
  gst_rtsp_message_add_header(request, GST_RTSP_HDR_RANGE, fixed);
  g_print("Normalized PLAY Range header: '%s' -> '%s'\n", trimmed, fixed);
  changed = TRUE;

  g_free(fixed);
  g_free(trimmed);
  return changed;
}

static GstRTSPStatusCode
mk15_pre_play_request(GstRTSPClient *client, GstRTSPContext *ctx)
{
  GstRTSPClientClass *parent_class =
      GST_RTSP_CLIENT_CLASS(mk15_rtsp_client_parent_class);

  normalize_play_range(ctx->request);

  if (parent_class->pre_play_request != NULL) {
    return parent_class->pre_play_request(client, ctx);
  }

  return GST_RTSP_STS_OK;
}

static void
mk15_rtsp_client_class_init(Mk15RTSPClientClass *klass)
{
  GstRTSPClientClass *client_class = GST_RTSP_CLIENT_CLASS(klass);
  client_class->pre_play_request = mk15_pre_play_request;
}

static void
mk15_rtsp_client_init(Mk15RTSPClient *self)
{
  (void) self;
}

static GstRTSPClient *
mk15_rtsp_server_create_client(GstRTSPServer *server)
{
  GstRTSPClient *client = g_object_new(mk15_rtsp_client_get_type(), NULL);
  GstRTSPSessionPool *session_pool = gst_rtsp_server_get_session_pool(server);
  GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
  GstRTSPAuth *auth = gst_rtsp_server_get_auth(server);
  GstRTSPThreadPool *thread_pool = gst_rtsp_server_get_thread_pool(server);

  if (session_pool != NULL) {
    gst_rtsp_client_set_session_pool(client, session_pool);
    g_object_unref(session_pool);
  }
  if (mounts != NULL) {
    gst_rtsp_client_set_mount_points(client, mounts);
    g_object_unref(mounts);
  }
  if (auth != NULL) {
    gst_rtsp_client_set_auth(client, auth);
    g_object_unref(auth);
  }
  if (thread_pool != NULL) {
    gst_rtsp_client_set_thread_pool(client, thread_pool);
    g_object_unref(thread_pool);
  }

  gst_rtsp_client_set_content_length_limit(
      client, gst_rtsp_server_get_content_length_limit(server));
  return client;
}

static void
mk15_rtsp_server_class_init(Mk15RTSPServerClass *klass)
{
  GstRTSPServerClass *server_class = GST_RTSP_SERVER_CLASS(klass);
  server_class->create_client = mk15_rtsp_server_create_client;
}

static void
mk15_rtsp_server_init(Mk15RTSPServer *self)
{
  (void) self;
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
  cfg->print_frame_logs = env_bool_default("PRINT_FRAME_LOGS", FALSE);
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
  cfg->pan_feedforward_gain = env_double_default("PAN_FEEDFORWARD_GAIN", 0.16);
  cfg->tilt_feedforward_gain = env_double_default("TILT_FEEDFORWARD_GAIN", 0.12);
  cfg->feedforward_alpha = env_double_default("FEEDFORWARD_ALPHA", 0.35);
  cfg->feedforward_limit = env_double_default("FEEDFORWARD_LIMIT", 0.12);
  cfg->feedforward_activation_zone = env_double_default("FEEDFORWARD_ACTIVATION_ZONE", 0.08);
  cfg->max_command = env_double_default("MAX_COMMAND", 1.0);
  cfg->invert_pan = env_bool_default("INVERT_PAN", FALSE);
  cfg->invert_tilt = env_bool_default("INVERT_TILT", FALSE);
  cfg->rtsp_enable = env_bool_default("RTSP_ENABLE", TRUE);
  cfg->rtsp_port = env_int_default("RTSP_PORT", DEFAULT_RTSP_PORT);
  cfg->udp_port = env_int_default("UDP_PORT", DEFAULT_UDP_PORT);
  cfg->bitrate = env_int_default("BITRATE", DEFAULT_RTSP_BITRATE);
  cfg->raw_record_enable = env_bool_default("RAW_RECORD_ENABLE", FALSE);
  cfg->raw_record_bitrate = env_int_default("RAW_RECORD_BITRATE", DEFAULT_RAW_RECORD_BITRATE);
  cfg->video_queue_buffers = env_int_default("VIDEO_QUEUE_BUFFERS", DEFAULT_VIDEO_QUEUE_BUFFERS);
  cfg->rtsp_queue_buffers = env_int_default("RTSP_QUEUE_BUFFERS", DEFAULT_RTSP_QUEUE_BUFFERS);
  cfg->raw_record_queue_buffers = env_int_default("RAW_RECORD_QUEUE_BUFFERS", DEFAULT_RAW_RECORD_QUEUE_BUFFERS);
  cfg->rtsp_viewer_latency_ms = env_int_default("RTSP_VIEWER_LATENCY_MS", DEFAULT_RTSP_VIEWER_LATENCY_MS);
  cfg->monitoring_errors_fatal = env_bool_default("MONITORING_ERRORS_FATAL", FALSE);
  cfg->rtsp_mount = env_strdup_default("RTSP_MOUNT", "/ds-test");
  cfg->raw_record_file = env_strdup_raw("RAW_RECORD_FILE");
  if (cfg->rtsp_mount[0] != '/') {
    gchar *fixed_mount = g_strdup_printf("/%s", cfg->rtsp_mount);
    g_free(cfg->rtsp_mount);
    cfg->rtsp_mount = fixed_mount;
  }
  cfg->infer_config = env_strdup_default(
      "INFER_CONFIG",
      "/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/config/config_infer_primary_yolo26.txt");
  cfg->tracker_config = env_strdup_default(
      "TRACKER_CONFIG",
      "/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/config/tracker_config.txt");
  cfg->metadata_ipc_file = env_strdup_raw("METADATA_IPC_FILE");
  cfg->state_file = env_strdup_raw("STATE_FILE");
  cfg->state_file_flush = env_bool_default("STATE_FILE_FLUSH", FALSE);
}

static void
free_config(AppConfig *cfg)
{
  g_free(cfg->selection);
  g_free(cfg->rtsp_mount);
  g_free(cfg->raw_record_file);
  g_free(cfg->infer_config);
  g_free(cfg->tracker_config);
  g_free(cfg->metadata_ipc_file);
  g_free(cfg->state_file);
}

static gboolean
start_rtsp_streaming(AppState *state)
{
  GstRTSPMountPoints *mounts = NULL;
  GstRTSPMediaFactory *factory = NULL;
  gchar launch_desc[768];
  gchar port_str[32];

  if (!state->cfg.rtsp_enable) {
    return TRUE;
  }

  g_snprintf(
      launch_desc,
      sizeof(launch_desc),
      "( udpsrc port=%d buffer-size=%u caps=\"application/x-rtp, media=video, "
      "clock-rate=90000, encoding-name=H264, payload=96\" "
      "! rtph264depay "
      "! h264parse config-interval=-1 "
      "! rtph264pay name=pay0 pt=96 config-interval=1 mtu=1200 )",
      state->cfg.udp_port,
      512 * 1024);
  g_snprintf(port_str, sizeof(port_str), "%d", state->cfg.rtsp_port);

  state->rtsp_server = g_object_new(mk15_rtsp_server_get_type(), NULL);
  g_object_set(state->rtsp_server, "service", port_str, NULL);

  mounts = gst_rtsp_server_get_mount_points(state->rtsp_server);
  factory = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(factory, launch_desc);
  gst_rtsp_media_factory_set_shared(factory, TRUE);
  gst_rtsp_mount_points_add_factory(mounts, state->cfg.rtsp_mount, factory);
  g_object_unref(mounts);

  gst_rtsp_server_attach(state->rtsp_server, NULL);
  g_print("\n *** prototype_v2: Launched RTSP Streaming at rtsp://localhost:%d%s ***\n\n",
          state->cfg.rtsp_port,
          state->cfg.rtsp_mount);
  return TRUE;
}

static gboolean
is_disposable_branch_element_name(const gchar *name)
{
  if (!name || !*name) {
    return FALSE;
  }

  return g_str_equal(name, "video-queue") ||
         g_str_equal(name, "preosd-tee") ||
         g_str_equal(name, "video-convert") ||
         g_str_equal(name, "osd-queue") ||
         g_str_equal(name, "osd") ||
         g_str_equal(name, "branch-tee") ||
         g_str_equal(name, "display-queue") ||
         g_str_equal(name, "display") ||
         g_str_equal(name, "fakesink") ||
         g_str_has_prefix(name, "rtsp-") ||
         g_str_has_prefix(name, "raw-record-");
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
      const gchar *src_name = GST_OBJECT_NAME(msg->src);
      gst_message_parse_error(msg, &error, &debug);
      if (!state->cfg.monitoring_errors_fatal && is_disposable_branch_element_name(src_name)) {
        g_printerr("WARNING disposable video branch error from %s: %s\n",
                   src_name ? src_name : "(unknown)",
                   error ? error->message : "unknown error");
        if (debug) {
          g_printerr("Details: %s\n", debug);
        }
        g_clear_error(&error);
        g_free(debug);
        break;
      }
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
style_frame_boxes(NvDsFrameMeta *frame_meta, const AppConfig *cfg, const FrameTargetSnapshot *snapshot)
{
  NvDsMetaList *l_obj = NULL;

  if (!frame_meta || !cfg) {
    return;
  }

  for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
    NvDsObjectMeta *obj_meta = (NvDsObjectMeta *) l_obj->data;
    TrackCandidate candidate;
    gboolean selected = FALSE;

    if (!obj_meta || obj_meta->class_id != cfg->target_class_id) {
      continue;
    }

    memset(&candidate, 0, sizeof(candidate));
    candidate.obj_meta = obj_meta;
    candidate.track_id = (guint64) obj_meta->object_id;
    candidate.class_id = obj_meta->class_id;
    g_strlcpy(candidate.label,
              (obj_meta->obj_label && *obj_meta->obj_label) ? obj_meta->obj_label : "unknown",
              sizeof(candidate.label));

    if (snapshot && snapshot->has_target && candidate.track_id == snapshot->target_id) {
      selected = TRUE;
    }
    style_candidate_box(&candidate, selected);
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
    gdouble dy_norm)
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
    g_snprintf(line2,
               sizeof(line2),
               "err %+.2f,%+.2f | conf %.2f | center %.0f,%.0f",
               dx_norm,
               dy_norm,
               selected->confidence,
               selected->cx,
               selected->cy);
  } else {
    g_snprintf(line1, sizeof(line1), "%s | target none | tracks %u | lost %u", status, visible_tracks, state->lost_frames);
    g_snprintf(line2, sizeof(line2), "err n/a | metadata waiting");
  }
  g_snprintf(line3, sizeof(line3), "prototype_v2 | metadata->bridge | %s", command_status);

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

static GstPadProbeReturn
video_overlay_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
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
    FrameTargetSnapshot snapshot;
    TrackCandidate selected;
    TrackCandidate *selected_ptr = NULL;

    if (!frame_meta) {
      continue;
    }

    if (!lookup_frame_target_snapshot(state, frame_meta->frame_num, &snapshot)) {
      continue;
    }

    style_frame_boxes(frame_meta, &state->cfg, &snapshot);
    if (snapshot.has_target) {
      memset(&selected, 0, sizeof(selected));
      selected.track_id = snapshot.target_id;
      selected.class_id = snapshot.class_id;
      g_strlcpy(selected.label, snapshot.label, sizeof(selected.label));
      selected.confidence = snapshot.confidence;
      selected.cx = snapshot.cx;
      selected.cy = snapshot.cy;
      selected_ptr = &selected;
    }
    add_overlay_meta(batch_meta,
                     frame_meta,
                     state,
                     selected_ptr,
                     snapshot.visible_tracks,
                     snapshot.status,
                     snapshot.command_status,
                     snapshot.has_target ? snapshot.dx_norm : 0.0,
                     snapshot.has_target ? snapshot.dy_norm : 0.0);
  }

  return GST_PAD_PROBE_OK;
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

/* ===== LATENCY STAGE 3 ADDED START =====
 * Pad probes used to timestamp the buffer after streammux and after nvinfer.
 * The existing tracker src probe later reads these timestamps and writes them to JSON.
 */
static GstPadProbeReturn
latency_stage3_mux_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  (void) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  LatencyStage3BufferTimes *times = latency_stage3_get_or_create(buf);
  if (times) {
    times->mux_src_mono_ns = latency_mono_ns();
  }
  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
latency_stage3_pgie_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  (void) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  LatencyStage3BufferTimes *times = latency_stage3_get_or_create(buf);
  if (times) {
    times->pgie_src_mono_ns = latency_mono_ns();
  }
  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
latency_stage3_video_queue_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  AppState *state = (AppState *) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  LatencyStage3BufferTimes *times = latency_stage3_get_or_create(buf);
  guint frame_idx = 0;
  const gint64 mono_ns = latency_mono_ns();
  if (times) {
    times->video_queue_src_mono_ns = mono_ns;
  }
  if (latency_stage3_extract_frame_idx(buf, &frame_idx)) {
    latency_stage3_update_branch_timing(
        state, &state->video_queue_frame_idx, &state->video_queue_src_mono_ns, frame_idx, mono_ns);
  }
  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
latency_stage3_nvosd_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  AppState *state = (AppState *) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  LatencyStage3BufferTimes *times = latency_stage3_get_or_create(buf);
  guint frame_idx = 0;
  const gint64 mono_ns = latency_mono_ns();
  if (times) {
    times->nvosd_src_mono_ns = mono_ns;
  }
  if (latency_stage3_extract_frame_idx(buf, &frame_idx)) {
    latency_stage3_update_branch_timing(
        state, &state->nvosd_frame_idx, &state->nvosd_src_mono_ns, frame_idx, mono_ns);
  }
  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
latency_stage3_display_queue_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  AppState *state = (AppState *) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  LatencyStage3BufferTimes *times = latency_stage3_get_or_create(buf);
  guint frame_idx = 0;
  const gint64 mono_ns = latency_mono_ns();
  if (times) {
    times->display_queue_src_mono_ns = mono_ns;
  }
  if (latency_stage3_extract_frame_idx(buf, &frame_idx)) {
    latency_stage3_update_branch_timing(
        state, &state->display_queue_frame_idx, &state->display_queue_src_mono_ns, frame_idx, mono_ns);
  }
  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
latency_stage3_rtsp_queue_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  AppState *state = (AppState *) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  LatencyStage3BufferTimes *times = latency_stage3_get_or_create(buf);
  guint frame_idx = 0;
  const gint64 mono_ns = latency_mono_ns();
  if (times) {
    times->rtsp_queue_src_mono_ns = mono_ns;
  }
  if (latency_stage3_extract_frame_idx(buf, &frame_idx)) {
    latency_stage3_update_branch_timing(
        state, &state->rtsp_queue_frame_idx, &state->rtsp_queue_src_mono_ns, frame_idx, mono_ns);
  }
  return GST_PAD_PROBE_OK;
}
/* ===== LATENCY STAGE 3 ADDED END ===== */

static gsize
format_state_json(char *buf,
                  gsize buf_size,
                  guint frame_idx,
                  gint64 ds_write_mono_ns,
                  guint64 frame_pts_ns,
                  guint64 frame_ntp_ns,
                  /* ===== LATENCY STAGE 2/3 ADDED START ===== */
                  gint64 c_probe_start_mono_ns,
                  gint64 c_control_done_mono_ns,
                  gint64 mux_src_mono_ns,
                  gint64 pgie_src_mono_ns,
                  gint64 tracker_src_mono_ns,
                  guint video_queue_frame_idx,
                  gint64 video_queue_src_mono_ns,
                  guint nvosd_frame_idx,
                  gint64 nvosd_src_mono_ns,
                  guint display_queue_frame_idx,
                  gint64 display_queue_src_mono_ns,
                  guint rtsp_queue_frame_idx,
                  gint64 rtsp_queue_src_mono_ns,
                  /* ===== LATENCY STAGE 2/3 ADDED END ===== */
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
  gint written = 0;
  (void) smooth_dx;
  (void) smooth_dy;
  (void) pan_cmd;
  (void) tilt_cmd;

  written += g_snprintf(
      buf + written,
      (written < (gint) buf_size) ? buf_size - (gsize) written : 0,
      "{\"frame_idx\":%u,"
      "\"ds_write_mono_ns\":%" G_GINT64_FORMAT ","
      "\"frame_pts_ns\":%" G_GUINT64_FORMAT ","
      "\"frame_ntp_ns\":%" G_GUINT64_FORMAT ","
      "\"c_probe_start_mono_ns\":%" G_GINT64_FORMAT ","
      "\"c_control_done_mono_ns\":%" G_GINT64_FORMAT ","
      "\"mux_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"pgie_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"tracker_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"video_queue_frame_idx\":%u,"
      "\"video_queue_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"nvosd_frame_idx\":%u,"
      "\"nvosd_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"display_queue_frame_idx\":%u,"
      "\"display_queue_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"rtsp_queue_frame_idx\":%u,"
      "\"rtsp_queue_src_mono_ns\":%" G_GINT64_FORMAT ","
      "\"status\":\"%s\",\"command_status\":\"%s\",\"visible_tracks\":%u,"
      "\"lost_frames\":%u,\"target_id\":",
      frame_idx,
      ds_write_mono_ns,
      frame_pts_ns,
      frame_ntp_ns,
      c_probe_start_mono_ns,
      c_control_done_mono_ns,
      mux_src_mono_ns,
      pgie_src_mono_ns,
      tracker_src_mono_ns,
      video_queue_frame_idx,
      video_queue_src_mono_ns,
      nvosd_frame_idx,
      nvosd_src_mono_ns,
      display_queue_frame_idx,
      display_queue_src_mono_ns,
      rtsp_queue_frame_idx,
      rtsp_queue_src_mono_ns,
      status,
      command_status,
      visible_tracks,
      lost_frames);

  if (target) {
    written += g_snprintf(
        buf + written,
        (written < (gint) buf_size) ? buf_size - (gsize) written : 0,
        "%" G_GUINT64_FORMAT ",\"class_id\":%d,\"class_name\":\"%s\",\"confidence\":%.4f,"
        "\"dx_norm\":%.4f,\"dy_norm\":%.4f,",
        target->track_id,
        target->class_id,
        target->label,
        target->confidence,
        dx_norm,
        dy_norm);
  } else {
    written += g_snprintf(
        buf + written,
        (written < (gint) buf_size) ? buf_size - (gsize) written : 0,
        "null,\"class_id\":null,\"class_name\":null,\"confidence\":null,"
        "\"dx_norm\":null,\"dy_norm\":null,");
  }

  written += g_snprintf(
      buf + written,
      (written < (gint) buf_size) ? buf_size - (gsize) written : 0,
      "\"smooth_dx_norm\":null,\"smooth_dy_norm\":null,\"pan_cmd\":null,\"tilt_cmd\":null}");

  if (written < 0) {
    return 0;
  }
  if ((gsize) written >= buf_size) {
    return buf_size > 0 ? buf_size - 1 : 0;
  }
  return (gsize) written;
}

static gboolean
setup_metadata_ipc(AppState *state)
{
  MetadataIpcRegion *region = NULL;

  if (!state->cfg.metadata_ipc_file || !*state->cfg.metadata_ipc_file) {
    return TRUE;
  }

  state->metadata_ipc_size = sizeof(MetadataIpcRegion);
  state->metadata_ipc_fd = open(state->cfg.metadata_ipc_file, O_RDWR | O_CREAT | O_TRUNC, 0600);
  if (state->metadata_ipc_fd < 0) {
    g_printerr("Failed to open METADATA_IPC_FILE %s: %s\n",
               state->cfg.metadata_ipc_file,
               g_strerror(errno));
    return FALSE;
  }
  if (ftruncate(state->metadata_ipc_fd, (off_t) state->metadata_ipc_size) != 0) {
    g_printerr("Failed to size METADATA_IPC_FILE %s: %s\n",
               state->cfg.metadata_ipc_file,
               g_strerror(errno));
    close(state->metadata_ipc_fd);
    state->metadata_ipc_fd = -1;
    return FALSE;
  }

  state->metadata_ipc_map = mmap(NULL,
                                 state->metadata_ipc_size,
                                 PROT_READ | PROT_WRITE,
                                 MAP_SHARED,
                                 state->metadata_ipc_fd,
                                 0);
  if (state->metadata_ipc_map == MAP_FAILED) {
    g_printerr("Failed to mmap METADATA_IPC_FILE %s: %s\n",
               state->cfg.metadata_ipc_file,
               g_strerror(errno));
    state->metadata_ipc_map = NULL;
    close(state->metadata_ipc_fd);
    state->metadata_ipc_fd = -1;
    return FALSE;
  }

  memset(state->metadata_ipc_map, 0, state->metadata_ipc_size);
  region = (MetadataIpcRegion *) state->metadata_ipc_map;
  region->magic = METADATA_IPC_MAGIC;
  region->version = METADATA_IPC_VERSION;
  region->payload_capacity = METADATA_IPC_PAYLOAD_MAX;
  return TRUE;
}

static void
publish_metadata_ipc(AppState *state,
                     guint frame_idx,
                     gint64 ds_write_mono_ns,
                     guint64 frame_pts_ns,
                     guint64 frame_ntp_ns,
                     /* ===== LATENCY STAGE 2/3 ADDED START ===== */
                     gint64 c_probe_start_mono_ns,
                     gint64 c_control_done_mono_ns,
                     gint64 mux_src_mono_ns,
                     gint64 pgie_src_mono_ns,
                     gint64 tracker_src_mono_ns,
                     guint video_queue_frame_idx,
                     gint64 video_queue_src_mono_ns,
                     guint nvosd_frame_idx,
                     gint64 nvosd_src_mono_ns,
                     guint display_queue_frame_idx,
                     gint64 display_queue_src_mono_ns,
                     guint rtsp_queue_frame_idx,
                     gint64 rtsp_queue_src_mono_ns,
                     /* ===== LATENCY STAGE 2/3 ADDED END ===== */
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
  MetadataIpcRegion *region = (MetadataIpcRegion *) state->metadata_ipc_map;
  MetadataIpcPayload payload;
  guint64 start_seq = 0;
  (void) smooth_dx;
  (void) smooth_dy;
  (void) pan_cmd;
  (void) tilt_cmd;

  if (!region) {
    return;
  }

  start_seq = region->sequence + 1;
  if ((start_seq & 1U) == 0) {
    start_seq++;
  }
  region->sequence = start_seq;
  __sync_synchronize();

  memset(&payload, 0, sizeof(payload));
  payload.c_probe_start_mono_ns = c_probe_start_mono_ns;
  payload.c_control_done_mono_ns = c_control_done_mono_ns;
  payload.mux_src_mono_ns = mux_src_mono_ns;
  payload.pgie_src_mono_ns = pgie_src_mono_ns;
  payload.tracker_src_mono_ns = tracker_src_mono_ns;
  payload.frame_pts_ns = frame_pts_ns;
  payload.frame_ntp_ns = frame_ntp_ns;
  payload.video_queue_frame_idx = video_queue_frame_idx;
  payload.video_queue_src_mono_ns = video_queue_src_mono_ns;
  payload.nvosd_frame_idx = nvosd_frame_idx;
  payload.nvosd_src_mono_ns = nvosd_src_mono_ns;
  payload.display_queue_frame_idx = display_queue_frame_idx;
  payload.display_queue_src_mono_ns = display_queue_src_mono_ns;
  payload.rtsp_queue_frame_idx = rtsp_queue_frame_idx;
  payload.rtsp_queue_src_mono_ns = rtsp_queue_src_mono_ns;
  payload.visible_tracks = visible_tracks;
  payload.lost_frames = lost_frames;
  payload.has_target = target ? 1U : 0U;
  payload.class_id = target ? target->class_id : -1;
  payload.confidence = target ? target->confidence : 0.0;
  payload.dx_norm = target ? dx_norm : 0.0;
  payload.dy_norm = target ? dy_norm : 0.0;
  if (status) {
    g_strlcpy(payload.status, status, sizeof(payload.status));
  }
  if (command_status) {
    g_strlcpy(payload.command_status, command_status, sizeof(payload.command_status));
  }
  if (target) {
    payload.target_id = target->track_id;
    g_strlcpy(payload.class_name, target->label, sizeof(payload.class_name));
  }

  memcpy(region->payload, &payload, sizeof(payload));
  region->payload_len = (guint32) sizeof(payload);
  region->frame_idx = frame_idx;
  region->ds_write_mono_ns = ds_write_mono_ns;

  __sync_synchronize();
  region->sequence = start_seq + 1;
}

static void
write_state_json(FILE *fp,
                 guint frame_idx,
                 gint64 ds_write_mono_ns,
                 guint64 frame_pts_ns,
                 guint64 frame_ntp_ns,
                 /* ===== LATENCY STAGE 2/3 ADDED START ===== */
                 gint64 c_probe_start_mono_ns,
                 gint64 c_control_done_mono_ns,
                 gint64 mux_src_mono_ns,
                 gint64 pgie_src_mono_ns,
                 gint64 tracker_src_mono_ns,
                 guint video_queue_frame_idx,
                 gint64 video_queue_src_mono_ns,
                 guint nvosd_frame_idx,
                 gint64 nvosd_src_mono_ns,
                 guint display_queue_frame_idx,
                 gint64 display_queue_src_mono_ns,
                 guint rtsp_queue_frame_idx,
                 gint64 rtsp_queue_src_mono_ns,
                 /* ===== LATENCY STAGE 2/3 ADDED END ===== */
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
                 gdouble tilt_cmd,
                 gboolean flush_now)
{
  char line_buf[METADATA_IPC_PAYLOAD_MAX + 2];
  gsize line_len = format_state_json(line_buf,
                                     sizeof(line_buf) - 1,
                                     frame_idx,
                                     ds_write_mono_ns,
                                     frame_pts_ns,
                                     frame_ntp_ns,
                                     c_probe_start_mono_ns,
                                     c_control_done_mono_ns,
                                     mux_src_mono_ns,
                                     pgie_src_mono_ns,
                                     tracker_src_mono_ns,
                                     video_queue_frame_idx,
                                     video_queue_src_mono_ns,
                                     nvosd_frame_idx,
                                     nvosd_src_mono_ns,
                                     display_queue_frame_idx,
                                     display_queue_src_mono_ns,
                                     rtsp_queue_frame_idx,
                                     rtsp_queue_src_mono_ns,
                                     status,
                                     command_status,
                                     visible_tracks,
                                     lost_frames,
                                     target,
                                     dx_norm,
                                     dy_norm,
                                     smooth_dx,
                                     smooth_dy,
                                     pan_cmd,
                                     tilt_cmd);
  line_buf[line_len++] = '\n';
  line_buf[line_len] = '\0';
  fwrite(line_buf, 1, line_len, fp);
  if (flush_now) {
    fflush(fp);
  }
}

static GstPadProbeReturn
tracker_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  (void) pad;
  AppState *state = (AppState *) user_data;
  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  /* ===== LATENCY STAGE 2/3 ADDED START =====
   * Stage 2 starts when this tracker-src probe begins handling the buffer.
   * Stage 3 tracker timestamp is also captured here because this probe is located
   * after nvtracker in the DeepStream pipeline.
   */
  const gint64 c_probe_start_mono_ns = latency_mono_ns();
  const LatencyStage3BufferTimes *latency_stage3_times =
      buf ? (const LatencyStage3BufferTimes *) gst_mini_object_get_qdata(
                GST_MINI_OBJECT(buf), latency_stage3_quark())
          : NULL;
  const gint64 mux_src_mono_ns = latency_stage3_times ? latency_stage3_times->mux_src_mono_ns : 0;
  const gint64 pgie_src_mono_ns = latency_stage3_times ? latency_stage3_times->pgie_src_mono_ns : 0;
  const gint64 tracker_src_mono_ns = c_probe_start_mono_ns;
  guint video_queue_frame_idx = 0;
  gint64 video_queue_src_mono_ns = 0;
  guint nvosd_frame_idx = 0;
  gint64 nvosd_src_mono_ns = 0;
  guint display_queue_frame_idx = 0;
  gint64 display_queue_src_mono_ns = 0;
  guint rtsp_queue_frame_idx = 0;
  gint64 rtsp_queue_src_mono_ns = 0;
  latency_stage3_snapshot_branch_timing(
      state,
      &video_queue_frame_idx,
      &video_queue_src_mono_ns,
      &nvosd_frame_idx,
      &nvosd_src_mono_ns,
      &display_queue_frame_idx,
      &display_queue_src_mono_ns,
      &rtsp_queue_frame_idx,
      &rtsp_queue_src_mono_ns);
  /* ===== LATENCY STAGE 2/3 ADDED END ===== */
  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
  NvDsMetaList *l_frame = NULL;

  if (!batch_meta) {
    return GST_PAD_PROBE_OK;
  }

  for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
    NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) l_frame->data;
    TrackCandidate candidates[MAX_CANDIDATES];
    TrackCandidate *selected = NULL;
    FrameTargetSnapshot snapshot;
    gchar target_id_buf[32];
    const guint current_frame_idx = frame_meta ? (guint) frame_meta->frame_num : state->frame_number;
    const guint64 frame_pts_ns =
        (frame_meta && frame_meta->buf_pts != GST_CLOCK_TIME_NONE) ? frame_meta->buf_pts : 0U;
    const guint64 frame_ntp_ns = frame_meta ? frame_meta->ntp_timestamp : 0U;
    guint visible_tracks = extract_candidates(frame_meta, &state->cfg, candidates, MAX_CANDIDATES);
    const gchar *status = "tracked";
    const gchar *command_status = "no_target";
    gdouble dx_norm = 0.0;
    gdouble dy_norm = 0.0;

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
        command_status = (pan_error == 0.0 && tilt_error == 0.0) ? "deadzone" : "active";
      }
    } else {
      command_status = status;
    }

    /* ===== LATENCY STAGE 2 ADDED START =====
     * Stage 2 timestamp: target selection and raw metadata extraction are done.
     * Overlay and video rendering stay on the video-side path after this point.
     */
    const gint64 c_control_done_mono_ns = latency_mono_ns();
    /* ===== LATENCY STAGE 2 ADDED END ===== */
    const gint64 ds_write_mono_ns = latency_mono_ns();

    if (selected) {
      g_snprintf(target_id_buf, sizeof(target_id_buf), "%" G_GUINT64_FORMAT, selected->track_id);
    } else {
      g_strlcpy(target_id_buf, "n/a", sizeof(target_id_buf));
    }

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.has_target = (selected != NULL);
    snapshot.visible_tracks = visible_tracks;
    snapshot.lost_frames = state->lost_frames;
    g_strlcpy(snapshot.status, status, sizeof(snapshot.status));
    g_strlcpy(snapshot.command_status, command_status, sizeof(snapshot.command_status));
    snapshot.dx_norm = selected ? dx_norm : 0.0;
    snapshot.dy_norm = selected ? dy_norm : 0.0;
    if (selected) {
      snapshot.target_id = selected->track_id;
      snapshot.class_id = selected->class_id;
      g_strlcpy(snapshot.label, selected->label, sizeof(snapshot.label));
      snapshot.confidence = selected->confidence;
      snapshot.cx = selected->cx;
      snapshot.cy = selected->cy;
    }
    store_frame_target_snapshot(state, current_frame_idx, &snapshot);

    if (state->cfg.print_frame_logs) {
      g_print("Frame %u | status=%s | tracks=%u | target=%s | id=%s | err=%+.2f,%+.2f | publish=%s\n",
              current_frame_idx,
              status,
              visible_tracks,
              selected ? selected->label : "none",
              target_id_buf,
              selected ? dx_norm : 0.0,
              selected ? dy_norm : 0.0,
              command_status);
    }

    publish_metadata_ipc(
        state,
        current_frame_idx,
        ds_write_mono_ns,
        frame_pts_ns,
        frame_ntp_ns,
        c_probe_start_mono_ns,
        c_control_done_mono_ns,
        mux_src_mono_ns,
        pgie_src_mono_ns,
        tracker_src_mono_ns,
        video_queue_frame_idx,
        video_queue_src_mono_ns,
        nvosd_frame_idx,
        nvosd_src_mono_ns,
        display_queue_frame_idx,
        display_queue_src_mono_ns,
        rtsp_queue_frame_idx,
        rtsp_queue_src_mono_ns,
        status,
        command_status,
        visible_tracks,
        state->lost_frames,
        selected,
        dx_norm,
        dy_norm,
        0.0,
        0.0,
        0.0,
        0.0);

    if (state->state_fp) {
      write_state_json(
          state->state_fp,
          current_frame_idx,
          ds_write_mono_ns,
          frame_pts_ns,
          frame_ntp_ns,
          /* ===== LATENCY STAGE 2/3 ADDED START ===== */
          c_probe_start_mono_ns,
          c_control_done_mono_ns,
          mux_src_mono_ns,
          pgie_src_mono_ns,
          tracker_src_mono_ns,
          video_queue_frame_idx,
          video_queue_src_mono_ns,
          nvosd_frame_idx,
          nvosd_src_mono_ns,
          display_queue_frame_idx,
          display_queue_src_mono_ns,
          rtsp_queue_frame_idx,
          rtsp_queue_src_mono_ns,
          /* ===== LATENCY STAGE 2/3 ADDED END ===== */
          status,
          command_status,
          visible_tracks,
          state->lost_frames,
          selected,
          dx_norm,
          dy_norm,
          0.0,
          0.0,
          0.0,
          0.0,
          state->cfg.state_file_flush);
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
  GstElement *source_tee = NULL;
  GstElement *source_mux_queue = NULL;
  GstElement *streammux = NULL;
  GstElement *pgie = NULL;
  GstElement *nvtracker = NULL;
  GstElement *video_queue = NULL;
  GstElement *preosd_tee = NULL;
  GstElement *tee = NULL;
  GstElement *display_conv = NULL;
  GstElement *osd_queue = NULL;
  GstElement *display_osd = NULL;
  GstElement *display_queue = NULL;
  GstElement *rtsp_queue = NULL;
  GstElement *rtsp_postconv = NULL;
  GstElement *rtsp_caps = NULL;
  GstElement *encoder = NULL;
  GstElement *h264parse = NULL;
  GstElement *rtppay = NULL;
  GstElement *udpsink = NULL;
  GstElement *raw_record_queue = NULL;
  GstElement *raw_record_conv = NULL;
  GstElement *raw_record_caps = NULL;
  GstElement *raw_record_encoder = NULL;
  GstElement *raw_record_parse = NULL;
  GstElement *raw_record_mux = NULL;
  GstElement *raw_record_sink = NULL;
  GstElement *sink = NULL;
  GstBus *bus = NULL;
  GstCaps *caps = NULL;
  GstCaps *encoder_caps = NULL;
  GstCaps *raw_record_encoder_caps = NULL;
  gchar *caps_str = NULL;
  gchar *encoder_caps_str = NULL;
  GstPad *mux_sink_pad = NULL;
  GstPad *source_tee_infer_pad = NULL;
  GstPad *source_tee_raw_pad = NULL;
  GstPad *source_mux_queue_pad = NULL;
  GstPad *source_mux_queue_src_pad = NULL;
  GstPad *raw_record_queue_pad = NULL;
  /* ===== LATENCY STAGE 3 ADDED START ===== */
  GstPad *mux_src_pad = NULL;
  GstPad *pgie_src_pad = NULL;
  GstPad *video_queue_src_pad = NULL;
  GstPad *osd_queue_src_pad = NULL;
  GstPad *nvosd_src_pad = NULL;
  GstPad *display_queue_src_pad = NULL;
  GstPad *rtsp_queue_src_pad = NULL;
  /* ===== LATENCY STAGE 3 ADDED END ===== */
  GstPad *tracker_src_pad = NULL;
  GstPad *preosd_osd_pad = NULL;
  GstPad *osd_queue_pad = NULL;
  GstPad *tee_display_pad = NULL;
  GstPad *tee_rtsp_pad = NULL;
  GstPad *display_queue_pad = NULL;
  GstPad *rtsp_queue_pad = NULL;
  guint bus_watch_id = 0;
  guint streammux_timeout_us = 33000;

  memset(&state, 0, sizeof(state));
  g_mutex_init(&state.latency_stage3_branch_lock);
  g_mutex_init(&state.frame_snapshot_lock);
  load_config(&state.cfg);
  if (state.cfg.raw_record_enable &&
      (!state.cfg.raw_record_file || !*state.cfg.raw_record_file)) {
    g_printerr("RAW_RECORD_ENABLE=1 requires RAW_RECORD_FILE to be set.\n");
    free_config(&state.cfg);
    return 1;
  }

  gst_init(NULL, NULL);
  state.loop = g_main_loop_new(NULL, FALSE);

  pipeline = gst_pipeline_new("deepstream-yolo26-rtsp-target-control");
  source = gst_element_factory_make("nvarguscamerasrc", "csi-source");
  source_caps = gst_element_factory_make("capsfilter", "source-caps");
  source_conv = gst_element_factory_make("nvvideoconvert", "source-convert");
  source_tee = gst_element_factory_make("tee", "source-tee");
  source_mux_queue = gst_element_factory_make("queue", "source-mux-queue");
  streammux = gst_element_factory_make("nvstreammux", "stream-muxer");
  pgie = gst_element_factory_make("nvinfer", "primary-infer");
  nvtracker = gst_element_factory_make("nvtracker", "tracker");
  video_queue = gst_element_factory_make("queue", "video-queue");
  preosd_tee = gst_element_factory_make("tee", "preosd-tee");
  display_conv = gst_element_factory_make("nvvideoconvert", "video-convert");
  osd_queue = gst_element_factory_make("queue", "osd-queue");
  display_osd = gst_element_factory_make("nvdsosd", "osd");
  tee = gst_element_factory_make("tee", "branch-tee");
  display_queue = gst_element_factory_make("queue", "display-queue");
  sink = gst_element_factory_make(state.cfg.show ? "nv3dsink" : "fakesink", state.cfg.show ? "display" : "fakesink");

  if (state.cfg.rtsp_enable) {
    rtsp_queue = gst_element_factory_make("queue", "rtsp-queue");
    rtsp_postconv = gst_element_factory_make("nvvideoconvert", "rtsp-convert");
    rtsp_caps = gst_element_factory_make("capsfilter", "rtsp-caps");
    encoder = gst_element_factory_make("nvv4l2h264enc", "rtsp-encoder");
    h264parse = gst_element_factory_make("h264parse", "rtsp-h264parse");
    rtppay = gst_element_factory_make("rtph264pay", "rtsp-payloader");
    udpsink = gst_element_factory_make("udpsink", "rtsp-udpsink");
  }
  if (state.cfg.raw_record_enable) {
    raw_record_queue = gst_element_factory_make("queue", "raw-record-queue");
    raw_record_conv = gst_element_factory_make("nvvideoconvert", "raw-record-convert");
    raw_record_caps = gst_element_factory_make("capsfilter", "raw-record-caps");
    raw_record_encoder = gst_element_factory_make("nvv4l2h264enc", "raw-record-encoder");
    raw_record_parse = gst_element_factory_make("h264parse", "raw-record-h264parse");
    raw_record_mux = gst_element_factory_make("matroskamux", "raw-record-mux");
    raw_record_sink = gst_element_factory_make("filesink", "raw-record-sink");
  }

  if (!pipeline || !source || !source_caps || !source_conv || !source_tee || !source_mux_queue ||
      !streammux || !pgie || !nvtracker ||
      !video_queue || !preosd_tee ||
      !tee || !display_queue || !display_conv || !osd_queue || !display_osd || !sink ||
      (state.cfg.rtsp_enable &&
       (!rtsp_queue || !rtsp_postconv || !rtsp_caps ||
        !encoder || !h264parse || !rtppay || !udpsink)) ||
      (state.cfg.raw_record_enable &&
       (!raw_record_queue || !raw_record_conv || !raw_record_caps ||
        !raw_record_encoder || !raw_record_parse || !raw_record_mux || !raw_record_sink))) {
    g_printerr("Failed to create one or more GStreamer elements.\n");
    free_config(&state.cfg);
    return 1;
  }

  state.pipeline = pipeline;
  state.metadata_ipc_fd = -1;
  if (!setup_metadata_ipc(&state)) {
    free_config(&state.cfg);
    return 1;
  }
  if (state.cfg.state_file && *state.cfg.state_file) {
    state.state_fp = fopen(state.cfg.state_file, "w");
    if (!state.state_fp) {
      g_printerr("Failed to open STATE_FILE: %s\n", state.cfg.state_file);
      if (state.metadata_ipc_map) {
        munmap(state.metadata_ipc_map, state.metadata_ipc_size);
      }
      if (state.metadata_ipc_fd >= 0) {
        close(state.metadata_ipc_fd);
      }
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
  g_object_set(G_OBJECT(source_mux_queue),
               "max-size-buffers", 4,
               "max-size-bytes", 0,
               "max-size-time", 0,
               NULL);
  if (state.cfg.fps_n > 0 && state.cfg.fps_d > 0) {
    streammux_timeout_us = (guint) ((1000000LL * state.cfg.fps_d) / state.cfg.fps_n);
    if (streammux_timeout_us == 0) {
      streammux_timeout_us = 1;
    }
  }
  g_object_set(G_OBJECT(streammux),
               "width", state.cfg.width,
               "height", state.cfg.height,
               "batch-size", 1,
               "batched-push-timeout", streammux_timeout_us,
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
  g_object_set(G_OBJECT(video_queue),
               "leaky", 2,
               "max-size-buffers", MAX(1, state.cfg.video_queue_buffers),
               "max-size-bytes", 0,
               "max-size-time", 0,
               NULL);
  g_object_set(G_OBJECT(osd_queue),
               "leaky", 2,
               "max-size-buffers", 2,
               "max-size-bytes", 0,
               "max-size-time", 0,
               NULL);
  if (state.cfg.rtsp_enable) {
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
  }
  if (state.cfg.raw_record_enable) {
    encoder_caps_str = g_strdup("video/x-raw(memory:NVMM), format=(string)I420");
    raw_record_encoder_caps = gst_caps_from_string(encoder_caps_str);
    g_free(encoder_caps_str);
    g_object_set(G_OBJECT(raw_record_caps), "caps", raw_record_encoder_caps, NULL);
    g_object_set(G_OBJECT(raw_record_queue),
                 "leaky", 2,
                 "max-size-buffers", MAX(1, state.cfg.raw_record_queue_buffers),
                 "max-size-bytes", 0,
                 "max-size-time", 0,
                 NULL);
    g_object_set(G_OBJECT(raw_record_encoder),
                 "bitrate", state.cfg.raw_record_bitrate,
                 "control-rate", 1,
                 "preset-level", 1,
                 "maxperf-enable", TRUE,
                 "iframeinterval", 15,
                 "idrinterval", 15,
                 "insert-sps-pps", TRUE,
                 "insert-vui", TRUE,
                 NULL);
    g_object_set(G_OBJECT(raw_record_sink),
                 "location", state.cfg.raw_record_file,
                 "async", FALSE,
                 "sync", FALSE,
                 NULL);
  }

  gst_bin_add_many(GST_BIN(pipeline),
                   source, source_caps, source_conv, source_tee, source_mux_queue,
                   streammux, pgie, nvtracker, video_queue,
                   display_conv, preosd_tee, osd_queue, display_osd, tee, display_queue, sink, NULL);
  if (state.cfg.rtsp_enable) {
    gst_bin_add_many(GST_BIN(pipeline),
                     rtsp_queue, rtsp_postconv, rtsp_caps, encoder, h264parse, rtppay, udpsink, NULL);
  }
  if (state.cfg.raw_record_enable) {
    gst_bin_add_many(GST_BIN(pipeline),
                     raw_record_queue, raw_record_conv, raw_record_caps,
                     raw_record_encoder, raw_record_parse, raw_record_mux, raw_record_sink, NULL);
  }

  if (!gst_element_link_many(source, source_caps, source_conv, source_tee, NULL)) {
    g_printerr("Failed to link source chain.\n");
    goto cleanup;
  }

  source_tee_infer_pad = gst_element_get_request_pad(source_tee, "src_%u");
  source_mux_queue_pad = gst_element_get_static_pad(source_mux_queue, "sink");
  source_mux_queue_src_pad = gst_element_get_static_pad(source_mux_queue, "src");
  mux_sink_pad = gst_element_get_request_pad(streammux, "sink_0");
  if (state.cfg.raw_record_enable) {
    source_tee_raw_pad = gst_element_get_request_pad(source_tee, "src_%u");
    raw_record_queue_pad = gst_element_get_static_pad(raw_record_queue, "sink");
  }
  if (!source_tee_infer_pad || !source_mux_queue_pad || !source_mux_queue_src_pad || !mux_sink_pad ||
      (state.cfg.raw_record_enable && (!source_tee_raw_pad || !raw_record_queue_pad)) ||
      gst_pad_link(source_tee_infer_pad, source_mux_queue_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link source tee to streammux/raw-record.\n");
    goto cleanup;
  }
  if (state.cfg.raw_record_enable &&
      gst_pad_link(source_tee_raw_pad, raw_record_queue_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link source tee to raw-record branch.\n");
    goto cleanup;
  }
  if (gst_pad_link(source_mux_queue_src_pad, mux_sink_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link source-mux queue to streammux.\n");
    goto cleanup;
  }

  if (!gst_element_link_many(streammux, pgie, nvtracker, video_queue, preosd_tee, NULL)) {
    g_printerr("Failed to link inference/tracker/pre-OSD chain.\n");
    goto cleanup;
  }

  if (!gst_element_link_many(osd_queue, display_conv, display_osd, tee, NULL)) {
    g_printerr("Failed to link inference/tracker/tee chain.\n");
    goto cleanup;
  }

  if (!gst_element_link_many(display_queue, sink, NULL)) {
    g_printerr("Failed to link display branch.\n");
    goto cleanup;
  }

  if (state.cfg.rtsp_enable) {
    if (!gst_element_link_many(rtsp_queue, rtsp_postconv, rtsp_caps, encoder, h264parse, rtppay, udpsink, NULL)) {
      g_printerr("Failed to link RTSP branch.\n");
      goto cleanup;
    }
  }
  if (state.cfg.raw_record_enable) {
    if (!gst_element_link_many(raw_record_queue, raw_record_conv, raw_record_caps,
                               raw_record_encoder, raw_record_parse, raw_record_mux, raw_record_sink, NULL)) {
      g_printerr("Failed to link raw-record branch.\n");
      goto cleanup;
    }
  }

  preosd_osd_pad = gst_element_get_request_pad(preosd_tee, "src_%u");
  osd_queue_pad = gst_element_get_static_pad(osd_queue, "sink");
  if (!preosd_osd_pad || !osd_queue_pad) {
    g_printerr("Failed to get pre-OSD tee branch pads.\n");
    goto cleanup;
  }
  if (gst_pad_link(preosd_osd_pad, osd_queue_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link pre-OSD tee to OSD branch.\n");
    goto cleanup;
  }

  tee_display_pad = gst_element_get_request_pad(tee, "src_%u");
  display_queue_pad = gst_element_get_static_pad(display_queue, "sink");
  if (state.cfg.rtsp_enable) {
    tee_rtsp_pad = gst_element_get_request_pad(tee, "src_%u");
    rtsp_queue_pad = gst_element_get_static_pad(rtsp_queue, "sink");
  }
  if (!tee_display_pad || !display_queue_pad ||
      (state.cfg.rtsp_enable && (!tee_rtsp_pad || !rtsp_queue_pad))) {
    g_printerr("Failed to get tee branch pads.\n");
    goto cleanup;
  }
  if (gst_pad_link(tee_display_pad, display_queue_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link tee branches.\n");
    goto cleanup;
  }
  if (state.cfg.rtsp_enable &&
      gst_pad_link(tee_rtsp_pad, rtsp_queue_pad) != GST_PAD_LINK_OK) {
    g_printerr("Failed to link tee branches.\n");
    goto cleanup;
  }

  /* ===== LATENCY STAGE 3 ADDED START =====
   * Add coarse DeepStream element timing probes.
   */
  mux_src_pad = gst_element_get_static_pad(streammux, "src");
  pgie_src_pad = gst_element_get_static_pad(pgie, "src");
  video_queue_src_pad = gst_element_get_static_pad(video_queue, "src");
  osd_queue_src_pad = gst_element_get_static_pad(osd_queue, "src");
  nvosd_src_pad = gst_element_get_static_pad(display_osd, "src");
  display_queue_src_pad = gst_element_get_static_pad(display_queue, "src");
  if (state.cfg.rtsp_enable) {
    rtsp_queue_src_pad = gst_element_get_static_pad(rtsp_queue, "src");
  }
  if (mux_src_pad) {
    gst_pad_add_probe(mux_src_pad, GST_PAD_PROBE_TYPE_BUFFER, latency_stage3_mux_src_probe, &state, NULL);
  } else {
    g_printerr("Warning: failed to get streammux src pad for latency stage 3.\n");
  }
  if (pgie_src_pad) {
    gst_pad_add_probe(pgie_src_pad, GST_PAD_PROBE_TYPE_BUFFER, latency_stage3_pgie_src_probe, &state, NULL);
  } else {
    g_printerr("Warning: failed to get pgie src pad for latency stage 3.\n");
  }
  if (video_queue_src_pad) {
    gst_pad_add_probe(video_queue_src_pad, GST_PAD_PROBE_TYPE_BUFFER, latency_stage3_video_queue_src_probe, &state, NULL);
  } else {
    g_printerr("Warning: failed to get video-queue src pad for latency stage 3.\n");
  }
  if (osd_queue_src_pad) {
    gst_pad_add_probe(osd_queue_src_pad, GST_PAD_PROBE_TYPE_BUFFER, video_overlay_pad_buffer_probe, &state, NULL);
  } else {
    g_printerr("Warning: failed to get osd-queue src pad for overlay probe.\n");
  }
  if (nvosd_src_pad) {
    gst_pad_add_probe(nvosd_src_pad, GST_PAD_PROBE_TYPE_BUFFER, latency_stage3_nvosd_src_probe, &state, NULL);
  } else {
    g_printerr("Warning: failed to get nvosd src pad for latency stage 3.\n");
  }
  if (display_queue_src_pad) {
    gst_pad_add_probe(
        display_queue_src_pad,
        GST_PAD_PROBE_TYPE_BUFFER,
        latency_stage3_display_queue_src_probe,
        &state,
        NULL);
  } else {
    g_printerr("Warning: failed to get display-queue src pad for latency stage 3.\n");
  }
  if (state.cfg.rtsp_enable) {
    if (rtsp_queue_src_pad) {
      gst_pad_add_probe(rtsp_queue_src_pad, GST_PAD_PROBE_TYPE_BUFFER, latency_stage3_rtsp_queue_src_probe, &state, NULL);
    } else {
      g_printerr("Warning: failed to get rtsp-queue src pad for latency stage 3.\n");
    }
  }
  /* ===== LATENCY STAGE 3 ADDED END ===== */

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
  if (state.cfg.rtsp_enable) {
    g_print("RTSP URL: rtsp://localhost:%d%s\n", state.cfg.rtsp_port, state.cfg.rtsp_mount);
  } else {
    g_print("RTSP disabled for this run.\n");
  }
  if (state.cfg.raw_record_enable) {
    g_print("Raw pre-OSD recording: %s\n", state.cfg.raw_record_file);
  }

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(state.loop);

cleanup:
  if (bus_watch_id > 0) {
    g_source_remove(bus_watch_id);
  }
  /* ===== LATENCY STAGE 3 ADDED START ===== */
  if (mux_src_pad) {
    gst_object_unref(mux_src_pad);
  }
  if (pgie_src_pad) {
    gst_object_unref(pgie_src_pad);
  }
  if (video_queue_src_pad) {
    gst_object_unref(video_queue_src_pad);
  }
  if (osd_queue_src_pad) {
    gst_object_unref(osd_queue_src_pad);
  }
  if (nvosd_src_pad) {
    gst_object_unref(nvosd_src_pad);
  }
  if (display_queue_src_pad) {
    gst_object_unref(display_queue_src_pad);
  }
  if (rtsp_queue_src_pad) {
    gst_object_unref(rtsp_queue_src_pad);
  }
  /* ===== LATENCY STAGE 3 ADDED END ===== */
  if (tracker_src_pad) {
    gst_object_unref(tracker_src_pad);
  }
  if (source_tee_infer_pad) {
    gst_object_unref(source_tee_infer_pad);
  }
  if (source_tee_raw_pad) {
    gst_object_unref(source_tee_raw_pad);
  }
  if (source_mux_queue_pad) {
    gst_object_unref(source_mux_queue_pad);
  }
  if (source_mux_queue_src_pad) {
    gst_object_unref(source_mux_queue_src_pad);
  }
  if (preosd_osd_pad) {
    gst_object_unref(preosd_osd_pad);
  }
  if (osd_queue_pad) {
    gst_object_unref(osd_queue_pad);
  }
  if (raw_record_queue_pad) {
    gst_object_unref(raw_record_queue_pad);
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
  if (mux_sink_pad) {
    gst_object_unref(mux_sink_pad);
  }
  if (caps) {
    gst_caps_unref(caps);
  }
  if (encoder_caps) {
    gst_caps_unref(encoder_caps);
  }
  if (raw_record_encoder_caps) {
    gst_caps_unref(raw_record_encoder_caps);
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
  if (state.metadata_ipc_map) {
    munmap(state.metadata_ipc_map, state.metadata_ipc_size);
  }
  if (state.metadata_ipc_fd >= 0) {
    close(state.metadata_ipc_fd);
  }
  if (state.loop) {
    g_main_loop_unref(state.loop);
  }
  g_mutex_clear(&state.latency_stage3_branch_lock);
  g_mutex_clear(&state.frame_snapshot_lock);
  free_config(&state.cfg);
  return 0;
}
