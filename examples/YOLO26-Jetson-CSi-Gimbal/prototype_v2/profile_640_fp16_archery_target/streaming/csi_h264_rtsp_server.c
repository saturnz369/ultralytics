#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp/gstrtspmessage.h>
#include <glib.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_SENSOR_ID 0
#define DEFAULT_WIDTH 2028
#define DEFAULT_HEIGHT 1112
#define DEFAULT_FPS_N 60
#define DEFAULT_FPS_D 1
#define DEFAULT_RTSP_PORT 8554
#define DEFAULT_BITRATE 8000000
#define DEFAULT_IFRAME_INTERVAL 30

typedef struct StreamConfig {
  gint sensor_id;
  gint width;
  gint height;
  gint fps_n;
  gint fps_d;
  gint rtsp_port;
  gint bitrate;
  gint iframe_interval;
  gchar *source_mode;
  gchar *mount;
  gchar *alias_mount;
  gchar *host_ip;
  gchar *rtsp_protocols;
} StreamConfig;

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

static gint
env_int_default(const gchar *key, gint fallback)
{
  const gchar *value = g_getenv(key);
  return (value && *value) ? (gint) g_ascii_strtoll(value, NULL, 10) : fallback;
}

static gchar *
env_strdup_default(const gchar *key, const gchar *fallback)
{
  const gchar *value = g_getenv(key);
  return g_strdup((value && *value) ? value : fallback);
}

static GstRTSPLowerTrans
parse_protocols(const gchar *value)
{
  if (g_strcmp0(value, "tcp") == 0) {
    return GST_RTSP_LOWER_TRANS_TCP;
  }

  if (g_strcmp0(value, "udp") == 0) {
    return GST_RTSP_LOWER_TRANS_UDP;
  }

  return GST_RTSP_LOWER_TRANS_UDP | GST_RTSP_LOWER_TRANS_TCP;
}

static gboolean
normalize_play_range(GstRTSPMessage *request)
{
  gchar *range = NULL;

  if (gst_rtsp_message_get_header(request, GST_RTSP_HDR_RANGE, &range, 0) != GST_RTSP_OK ||
      range == NULL) {
    return FALSE;
  }

  if (g_str_has_prefix(range, "npt=") ||
      g_str_has_prefix(range, "clock=") ||
      g_str_has_prefix(range, "smpte=")) {
    return FALSE;
  }

  if (!g_regex_match_simple("^[0-9]+(?:\\.[0-9]+)?-$", range, 0, 0)) {
    return FALSE;
  }

  gchar *fixed = g_strdup_printf("npt=%s", range);
  gst_rtsp_message_remove_header(request, GST_RTSP_HDR_RANGE, 0);
  gst_rtsp_message_add_header(request, GST_RTSP_HDR_RANGE, fixed);
  g_print("Normalized PLAY Range header: '%s' -> '%s'\n", range, fixed);
  g_free(fixed);
  return TRUE;
}

static GstRTSPStatusCode
mk15_pre_play_request(GstRTSPClient *client, GstRTSPContext *ctx)
{
  GstRTSPClientClass *parent_class =
      GST_RTSP_CLIENT_CLASS(mk15_rtsp_client_parent_class);

  (void) client;
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
load_config(StreamConfig *cfg)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->sensor_id = env_int_default("SENSOR_ID", DEFAULT_SENSOR_ID);
  cfg->width = env_int_default("CAMERA_WIDTH", DEFAULT_WIDTH);
  cfg->height = env_int_default("CAMERA_HEIGHT", DEFAULT_HEIGHT);
  cfg->fps_n = env_int_default("CAMERA_FPS_N", DEFAULT_FPS_N);
  cfg->fps_d = env_int_default("CAMERA_FPS_D", DEFAULT_FPS_D);
  cfg->rtsp_port = env_int_default("RTSP_PORT", DEFAULT_RTSP_PORT);
  cfg->bitrate = env_int_default("BITRATE", DEFAULT_BITRATE);
  cfg->iframe_interval = env_int_default("IFRAME_INTERVAL", DEFAULT_IFRAME_INTERVAL);
  cfg->source_mode = env_strdup_default("VIDEO_SOURCE", "csi");
  cfg->mount = env_strdup_default("RTSP_MOUNT", "/stream");
  cfg->alias_mount = env_strdup_default("RTSP_ALIAS_MOUNT", "/main.264");
  cfg->host_ip = env_strdup_default("RTSP_HOST_IP", "192.168.144.100");
  cfg->rtsp_protocols = env_strdup_default("RTSP_PROTOCOLS", "auto");
  if (cfg->mount[0] != '/') {
    gchar *fixed_mount = g_strdup_printf("/%s", cfg->mount);
    g_free(cfg->mount);
    cfg->mount = fixed_mount;
  }
  if (cfg->alias_mount[0] != '/') {
    gchar *fixed_alias = g_strdup_printf("/%s", cfg->alias_mount);
    g_free(cfg->alias_mount);
    cfg->alias_mount = fixed_alias;
  }
}

static void
free_config(StreamConfig *cfg)
{
  g_free(cfg->source_mode);
  g_free(cfg->mount);
  g_free(cfg->alias_mount);
  g_free(cfg->host_ip);
  g_free(cfg->rtsp_protocols);
}

int
main(int argc, char *argv[])
{
  (void) argc;
  (void) argv;

  StreamConfig cfg;
  GMainLoop *loop = NULL;
  GstRTSPServer *server = NULL;
  GstRTSPMountPoints *mounts = NULL;
  GstRTSPMediaFactory *factory = NULL;
  gchar *pipeline_launch = NULL;
  gchar port[32];

  gst_init(NULL, NULL);
  load_config(&cfg);
  loop = g_main_loop_new(NULL, FALSE);

  if (g_strcmp0(cfg.source_mode, "test-pattern") == 0) {
    pipeline_launch = g_strdup_printf(
        "( "
        "videotestsrc is-live=true pattern=smpte ! "
        "video/x-raw,width=%d,height=%d,framerate=%d/%d,format=I420 ! "
        "videoconvert ! video/x-raw,format=NV12 ! "
        "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        "nvv4l2h264enc bitrate=%d control-rate=1 preset-level=1 maxperf-enable=true profile=0 num-B-Frames=0 "
        "iframeinterval=%d idrinterval=%d insert-sps-pps=true insert-vui=true ! "
        "video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! "
        "h264parse config-interval=-1 ! rtph264pay name=pay0 pt=96 mtu=1200 config-interval=1 )",
        cfg.width,
        cfg.height,
        cfg.fps_n,
        cfg.fps_d,
        cfg.bitrate,
        cfg.iframe_interval,
        cfg.iframe_interval);
  } else {
    pipeline_launch = g_strdup_printf(
        "( nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/%d,format=NV12 ! "
        "queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
        "nvv4l2h264enc bitrate=%d control-rate=1 preset-level=1 maxperf-enable=true profile=0 num-B-Frames=0 "
        "iframeinterval=%d idrinterval=%d insert-sps-pps=true insert-vui=true ! "
        "video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! "
        "h264parse config-interval=-1 ! rtph264pay name=pay0 pt=96 mtu=1200 config-interval=1 )",
        cfg.sensor_id,
        cfg.width,
        cfg.height,
        cfg.fps_n,
        cfg.fps_d,
        cfg.bitrate,
        cfg.iframe_interval,
        cfg.iframe_interval);
  }

  g_snprintf(port, sizeof(port), "%d", cfg.rtsp_port);

  server = g_object_new(mk15_rtsp_server_get_type(), NULL);
  g_object_set(server, "service", port, NULL);

  mounts = gst_rtsp_server_get_mount_points(server);
  factory = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(factory, pipeline_launch);
  gst_rtsp_media_factory_set_shared(factory, TRUE);
  gst_rtsp_media_factory_set_protocols(factory, parse_protocols(cfg.rtsp_protocols));
  gst_rtsp_mount_points_add_factory(mounts, cfg.mount, factory);
  if (g_strcmp0(cfg.alias_mount, cfg.mount) != 0) {
    gst_rtsp_mount_points_add_factory(mounts, cfg.alias_mount, g_object_ref(factory));
  }
  g_object_unref(mounts);

  if (gst_rtsp_server_attach(server, NULL) == 0) {
    g_printerr("Failed to attach RTSP server on port %d\n", cfg.rtsp_port);
    g_free(pipeline_launch);
    g_object_unref(server);
    g_main_loop_unref(loop);
    free_config(&cfg);
    return 1;
  }

  g_print("RTSP server started\n");
  g_print("Source mode: %s\n", cfg.source_mode);
  g_print("Pipeline: %s\n", pipeline_launch);
  g_print("RTSP protocols: %s\n", cfg.rtsp_protocols);
  g_print("Jetson/MK15 URL: rtsp://%s:%d%s\n", cfg.host_ip, cfg.rtsp_port, cfg.mount);
  g_print("SIYI-style URL: rtsp://%s:%d%s\n", cfg.host_ip, cfg.rtsp_port, cfg.alias_mount);
  g_print("Local URL: rtsp://127.0.0.1:%d%s\n", cfg.rtsp_port, cfg.mount);

  g_main_loop_run(loop);

  g_main_loop_unref(loop);
  g_free(pipeline_launch);
  g_object_unref(server);
  free_config(&cfg);
  return 0;
}
