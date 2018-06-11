#include <gtk/gtk.h>

int main(int argc, char *argv[]){
    // Glade and GTK+3 init
    // Taken from:
    // https://prognotes.net/2015/06/gtk-3-c-program-using-glade-3/
    GtkBuilder  *builder;
    GtkWidget   *window;

    gtk_init(&argc, &argv);

    builder = gtk_builder_new();
    gtk_builder_add_from_file(builder, "window_main.glade", NULL);

    window = GTK_WIDGET(gtk_builder_get_object(builder, "window_main"));
    gtk_builder_connect_signals(builder, NULL);

    g_object_unref(builder);

    gtk_widget_show(window);
    gtk_main();

    return 0;
}

void on_window_main_destroy(){
    gtk_main_quit();
}
