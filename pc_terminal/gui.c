#include "../pc_terminal/pc.h"
#include "../pc_terminal/protocol.h"
#include <gtk/gtk.h>

#ifdef PC_GUI
uint8_t ctrlr_mode;
// Buttons
GtkWidget *safeBtn, *panicBtn, *manualBtn, *calibrationBtn,
	*yawcontrolBtn, *fullcontrolBtn, *rawBtn, *heightBtn, *dumplogsBtn;
// Progrss bars
GtkWidget *batteryBar, *motor0Bar, *motor1Bar, *motor2Bar, *motor3Bar;
// Text labels
GtkWidget *modeTxt, *debugLblTxt0, *debugValTxt0, *debugLblTxt1,
	*debugValTxt1, *debugLblTxt2, *debugValTxt2, *debugLblTxt3, *debugValTxt3;
// Console textview
GtkWidget *consoleTxtView;

int main(int argc, char *argv[]) {
	// Glade and GTK+3 init
	// Taken from:
	// https://prognotes.net/2015/06/gtk-3-c-program-using-glade-3/
	GtkBuilder *builder;
	GtkWidget *window;

	gtk_init(&argc, &argv);

	builder = gtk_builder_new();
	gtk_builder_add_from_file(builder, "window_main.glade", NULL);

	window = GTK_WIDGET(gtk_builder_get_object(builder, "window_main"));
	gtk_builder_connect_signals(builder, NULL);

	// Initialize variables
	ctrlr_mode = SAFE;
	// Buttons
	safeBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st0"));
	panicBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st1"));
	manualBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st2"));
	calibrationBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st3"));
	yawcontrolBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st4"));
	fullcontrolBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st5"));
	rawBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st6"));
	heightBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st7"));
	dumplogsBtn = GTK_WIDGET(gtk_builder_get_object(builder, "btn_st9"));
	// Progress bars
	batteryBar = GTK_WIDGET(gtk_builder_get_object(builder, "BatteryBar"));
	motor0Bar = GTK_WIDGET(gtk_builder_get_object(builder, "Motor0Bar"));
	motor1Bar = GTK_WIDGET(gtk_builder_get_object(builder, "Motor1Bar"));
	motor2Bar = GTK_WIDGET(gtk_builder_get_object(builder, "Motor2Bar"));
	motor3Bar = GTK_WIDGET(gtk_builder_get_object(builder, "Motor3Bar"));
	// Text labels
	modeTxt = GTK_WIDGET(gtk_builder_get_object(builder, "ModeLabel"));
	debugLblTxt0 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugLabel0"));
	debugValTxt0 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugValue0"));
	debugLblTxt1 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugLabel1"));
	debugValTxt1 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugValue1"));
	debugLblTxt2 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugLabel2"));
	debugValTxt2 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugValue2"));
	debugLblTxt3 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugLabel3"));
	debugValTxt3 = GTK_WIDGET(gtk_builder_get_object(builder, "DebugValue3"));
	// Console textview
	consoleTxtView = GTK_WIDGET(gtk_builder_get_object(builder, "ConsoleTxtView"));

	g_object_unref(builder);

	gtk_widget_show(window);
	gtk_main();

	return 0;
}

void on_window_main_destroy() {
	gtk_main_quit();
}

void on_btn_safe_press() {
	detect_term_input('0');
}

void on_btn_panic_press() {
	detect_term_input('1');
}

void on_btn_manual_press() {
	detect_term_input('2');
}

void on_btn_calibration_press() {
	detect_term_input('3');
}

void on_btn_yawcontrol_press() {
	detect_term_input('4');
}

void on_btn_fullcontrol_press() {
	detect_term_input('5');
}

void on_btn_raw_press() {
	detect_term_input('6');
}

void on_btn_height_press() {
	detect_term_input('7');
}

void on_btn_dumplogs_press() {
	detect_term_input('9');
}
#endif