/*
 * C.A.V.E. — Processing UI
 *
 * Build (Ubuntu / Raspberry Pi with Ubuntu 18.04):
 *   sudo apt-get install libgtk-3-dev        # one-time setup
 *   gcc -o cave_ui cave_ui.c $(pkg-config --cflags --libs gtk+-3.0)
 *
 * Or simply:
 *   make
 */

#include <gtk/gtk.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>

#define ROSBAGS_DIR  "rosbags"
#define MAX_PATH_LEN 2048

/* ════════════════════════════════════════════════════════════════════
 *  Data types
 * ════════════════════════════════════════════════════════════════════ */

typedef struct {
    GtkWidget *window;
    GtkWidget *drop_label;      /* shows selected file path          */
    GtkWidget *upload_status;   /* status next to the Upload button  */
    char       sel_path[MAX_PATH_LEN];
} App;

/* One instance per script button — lives for the duration of the app */
typedef struct {
    GtkWidget  *status_label;
    const char *command;
} ScriptCtx;

/* ════════════════════════════════════════════════════════════════════
 *  Helpers
 * ════════════════════════════════════════════════════════════════════ */

static void ensure_dir(const char *dir)
{
    struct stat st;
    if (stat(dir, &st) != 0 && errno == ENOENT)
        mkdir(dir, 0755);
}

/* Called by GLib when a child process exits */
static void on_child_exit(GPid pid, gint status, gpointer data)
{
    (void)status;
    gtk_label_set_text(GTK_LABEL((GtkWidget *)data), "Done!");
    g_spawn_close_pid(pid);
}

/* Spawn a shell command asynchronously.
 * Sets status_label to "In Progress..." immediately,
 * then to "Done!" when the process exits.              */
static void run_async(const char *cmd, GtkWidget *status_label)
{
    gtk_label_set_text(GTK_LABEL(status_label), "In Progress...");

    gchar *argv[] = { "/bin/sh", "-c", (gchar *)cmd, NULL };
    GPid   pid;
    GError *err = NULL;

    if (!g_spawn_async(NULL, argv, NULL,
                       G_SPAWN_DO_NOT_REAP_CHILD,
                       NULL, NULL, &pid, &err)) {
        gtk_label_set_text(GTK_LABEL(status_label), "Error!");
        g_printerr("Spawn error: %s\n", err->message);
        g_error_free(err);
        return;
    }
    g_child_watch_add(pid, on_child_exit, status_label);
}

/* ════════════════════════════════════════════════════════════════════
 *  Script button callback
 * ════════════════════════════════════════════════════════════════════ */

static void on_script_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    ScriptCtx *ctx = data;
    run_async(ctx->command, ctx->status_label);
}

/* ════════════════════════════════════════════════════════════════════
 *  Script row builder
 *
 *  Creates one horizontal row:  [ Button ]  status_label
 *  and appends it to vbox.
 *
 *  To add more pipeline steps, duplicate an add_script_row() call
 *  in main() and change the label and command strings.
 * ════════════════════════════════════════════════════════════════════ */

static void add_script_row(GtkWidget  *vbox,
                            const char *label,
                            const char *command)
{
    GtkWidget *row  = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
    GtkWidget *btn  = gtk_button_new_with_label(label);
    GtkWidget *stat = gtk_label_new("");

    ScriptCtx *ctx  = g_new(ScriptCtx, 1);   /* app-lifetime alloc */
    ctx->command      = command;
    ctx->status_label = stat;

    gtk_widget_set_size_request(btn, 220, 36);
    gtk_label_set_xalign(GTK_LABEL(stat), 0.0f);

    gtk_box_pack_start(GTK_BOX(row), btn,  FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(row), stat, TRUE,  FALSE, 0);

    /* Small vertical padding around each row */
    gtk_box_pack_start(GTK_BOX(vbox), row, FALSE, FALSE, 4);

    g_signal_connect(btn, "clicked", G_CALLBACK(on_script_clicked), ctx);
}

/* ════════════════════════════════════════════════════════════════════
 *  Browse button
 * ════════════════════════════════════════════════════════════════════ */

static void on_browse_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    App *app = data;

    GtkWidget *dlg = gtk_file_chooser_dialog_new(
        "Select a .bag file",
        GTK_WINDOW(app->window),
        GTK_FILE_CHOOSER_ACTION_OPEN,
        "_Cancel", GTK_RESPONSE_CANCEL,
        "_Open",   GTK_RESPONSE_ACCEPT,
        NULL);

    /* Prefer .bag files, but allow everything */
    GtkFileFilter *ff = gtk_file_filter_new();
    gtk_file_filter_set_name(ff, "ROS bag files (*.bag)");
    gtk_file_filter_add_pattern(ff, "*.bag");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dlg), ff);

    GtkFileFilter *fa = gtk_file_filter_new();
    gtk_file_filter_set_name(fa, "All files (*)");
    gtk_file_filter_add_pattern(fa, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dlg), fa);

    if (gtk_dialog_run(GTK_DIALOG(dlg)) == GTK_RESPONSE_ACCEPT) {
        char *path = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dlg));
        g_strlcpy(app->sel_path, path, MAX_PATH_LEN);
        gtk_label_set_text(GTK_LABEL(app->drop_label), path);
        g_free(path);
    }
    gtk_widget_destroy(dlg);
}

/* ════════════════════════════════════════════════════════════════════
 *  Drag-and-drop handler
 * ════════════════════════════════════════════════════════════════════ */

static void on_drag_data(
    GtkWidget *w, GdkDragContext *ctx,
    gint x, gint y,
    GtkSelectionData *sel,
    guint info, guint time,
    gpointer data)
{
    (void)w; (void)x; (void)y; (void)info;
    App *app = data;

    gchar **uris = gtk_selection_data_get_uris(sel);
    if (uris && uris[0]) {
        gchar *path = g_filename_from_uri(uris[0], NULL, NULL);
        if (path) {
            g_strlcpy(app->sel_path, path, MAX_PATH_LEN);
            gtk_label_set_text(GTK_LABEL(app->drop_label), path);
            g_free(path);
        }
        g_strfreev(uris);
    }
    gtk_drag_finish(ctx, TRUE, FALSE, time);
}

/* ════════════════════════════════════════════════════════════════════
 *  Upload button
 * ════════════════════════════════════════════════════════════════════ */

static void on_upload_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    App *app = data;

    if (app->sel_path[0] == '\0') {
        gtk_label_set_text(GTK_LABEL(app->upload_status),
                           "No file selected!");
        return;
    }

    ensure_dir(ROSBAGS_DIR);

    char cmd[MAX_PATH_LEN + 64];
    g_snprintf(cmd, sizeof(cmd),
               "cp -- \"%s\" \"%s/\"", app->sel_path, ROSBAGS_DIR);

    run_async(cmd, app->upload_status);
}

/* ════════════════════════════════════════════════════════════════════
 *  main — window layout
 * ════════════════════════════════════════════════════════════════════ */

int main(int argc, char *argv[])
{
    gtk_init(&argc, &argv);

    App app;
    memset(&app, 0, sizeof(app));

    /* ── Window ──────────────────────────────────────────────────── */
    app.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(app.window), "C.A.V.E.");
    gtk_window_set_default_size(GTK_WINDOW(app.window), 660, 500);
    gtk_container_set_border_width(GTK_CONTAINER(app.window), 20);
    g_signal_connect(app.window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    /* ── CSS — drop-zone styling ─────────────────────────────────── */
    GtkCssProvider *css = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css,
        "#drop_zone {"
        "  border: 2px dashed #888888;"
        "  border-radius: 8px;"
        "  background-color: #f6f6f6;"
        "}"
        "#drop_zone:drop(active) {"
        "  border-color: #3584e4;"
        "  background-color: #ddeeff;"
        "}",
        -1, NULL);
    gtk_style_context_add_provider_for_screen(
        gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(css),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    /* ── Root vertical box ───────────────────────────────────────── */
    GtkWidget *root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 16);
    gtk_container_add(GTK_CONTAINER(app.window), root);

    /* ── Title ───────────────────────────────────────────────────── */
    GtkWidget *title = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(title),
        "<span font='24' weight='bold'>C.A.V.E.</span>");
    gtk_widget_set_halign(title, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(root), title, FALSE, FALSE, 4);

    gtk_box_pack_start(GTK_BOX(root),
        gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 0);

    /* ── "Insert File" section ───────────────────────────────────── */
    GtkWidget *lbl_file = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(lbl_file), "<b>Insert File</b>");
    gtk_widget_set_halign(lbl_file, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(root), lbl_file, FALSE, FALSE, 0);

    /* Drag-and-drop zone (EventBox so it can receive drop events) */
    GtkWidget *drop_ev = gtk_event_box_new();
    gtk_widget_set_name(drop_ev, "drop_zone");
    gtk_widget_set_size_request(drop_ev, -1, 80);

    static const GtkTargetEntry drag_targets[] = {
        { "text/uri-list", 0, 0 }
    };
    gtk_drag_dest_set(drop_ev,
                      GTK_DEST_DEFAULT_ALL,
                      drag_targets,
                      G_N_ELEMENTS(drag_targets),
                      GDK_ACTION_COPY);
    g_signal_connect(drop_ev, "drag-data-received",
                     G_CALLBACK(on_drag_data), &app);

    app.drop_label = gtk_label_new(
        "Drag & drop a .bag file here  —  or use Browse below");
    gtk_label_set_ellipsize(GTK_LABEL(app.drop_label), PANGO_ELLIPSIZE_MIDDLE);
    gtk_container_add(GTK_CONTAINER(drop_ev), app.drop_label);

    gtk_box_pack_start(GTK_BOX(root), drop_ev, FALSE, FALSE, 0);

    /* Browse + Upload row */
    GtkWidget *file_row   = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    GtkWidget *browse_btn = gtk_button_new_with_label("Browse...");
    GtkWidget *upload_btn = gtk_button_new_with_label("Upload to rosbags/");
    app.upload_status     = gtk_label_new("");

    gtk_widget_set_size_request(browse_btn, 110, -1);
    gtk_widget_set_size_request(upload_btn, 170, -1);

    g_signal_connect(browse_btn, "clicked",
                     G_CALLBACK(on_browse_clicked), &app);
    g_signal_connect(upload_btn, "clicked",
                     G_CALLBACK(on_upload_clicked), &app);

    gtk_box_pack_start(GTK_BOX(file_row), browse_btn,        FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(file_row), upload_btn,        FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(file_row), app.upload_status, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(root), file_row, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(root),
        gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 0);

    /* ── Pipeline steps ──────────────────────────────────────────── */
    GtkWidget *lbl_steps = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(lbl_steps), "<b>Pipeline Steps</b>");
    gtk_widget_set_halign(lbl_steps, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(root), lbl_steps, FALSE, FALSE, 0);

    /* All script rows go inside this box */
    GtkWidget *scripts_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_box_pack_start(GTK_BOX(root), scripts_vbox, TRUE, TRUE, 0);

    /* ╔══════════════════════════════════════════════════════════════╗
     * ║  ADD / REMOVE PIPELINE STEPS BELOW                         ║
     * ║                                                              ║
     * ║  add_script_row(scripts_vbox,                               ║
     * ║      "Button label",      ← text shown on the button        ║
     * ║      "shell command"      ← any bash command or script path  ║
     * ║  );                                                          ║
     * ║                                                              ║
     * ║  The status label to the right will show:                   ║
     * ║      "In Progress..."  while the command runs               ║
     * ║      "Done!"           when it exits                        ║
     * ╚══════════════════════════════════════════════════════════════╝ */

    add_script_row(scripts_vbox,
        "Step 1 — Echo test",
        "echo 'Step 1: Hello from C.A.V.E.'; sleep 1; echo 'Step 1 complete'");

    add_script_row(scripts_vbox,
        "Step 2 — System info",
        "echo 'Step 2: System info:'; uname -a; sleep 1; echo 'Step 2 complete'");

    /* ──────────────────────────────────────────────────────────────
     * Example of a third step (uncomment to enable):
     *
     * add_script_row(scripts_vbox,
     *     "Step 3 — Launch RViz",
     *     "source /opt/ros/melodic/setup.bash && rviz");
     * ────────────────────────────────────────────────────────────── */

    gtk_widget_show_all(app.window);
    gtk_main();
    return 0;
}
