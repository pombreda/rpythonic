cairo_rectangle_int = _cairo_rectangle_int

gtk_window_new.defaults[0] = GTK_WINDOW_TOPLEVEL

gtk_box_pack_start.defaults[2] = True		# expand
gtk_box_pack_start.defaults[3] = True		# fill
gtk_box_pack_end.defaults[2] = True		# expand
gtk_box_pack_end.defaults[3] = True		# fill

_RETURNS_CHARP_ = [
	gtk_accelerator_get_label,
	gtk_widget_path_iter_get_name,

	gtk_rc_get_theme_dir,
	gtk_rc_get_module_dir,
	gtk_rc_get_im_module_path,
	gtk_rc_get_im_module_file,

	gtk_icon_size_get_name,

	gtk_icon_source_get_filename,

	gtk_icon_source_get_icon_name,

	gtk_widget_get_name,

	gtk_widget_get_composite_name,

	gtk_widget_get_tooltip_text,
	gtk_widget_get_tooltip_markup,

	gtk_menu_get_accel_path,
	gtk_menu_get_title,


	gtk_action_get_name,
	gtk_action_get_accel_path,
	gtk_action_get_label,
	gtk_action_get_tooltip,
	gtk_action_group_get_name,
	gtk_action_group_translate_string,

	gtk_window_get_title,

	gtk_label_get_text,
	gtk_label_get_label,

	gtk_app_chooser_get_content_type,
	gtk_app_chooser_dialog_get_heading,

	gtk_tree_path_to_string,

	gtk_tree_model_get_string_from_iter,
	gtk_cell_area_get_current_path_string,

	gtk_tree_view_column_get_title,

	gtk_text_iter_get_slice,
	gtk_text_iter_get_text,
	gtk_text_iter_get_visible_slice,
	gtk_text_iter_get_visible_text,

	gtk_editable_get_chars,

	gtk_entry_buffer_get_text,

	gtk_combo_box_get_title,
	gtk_combo_box_text_get_active_text,
	gtk_entry_get_text,
	gtk_entry_get_icon_name,
	gtk_entry_get_icon_tooltip_text,
	gtk_entry_get_icon_tooltip_markup,

	gtk_app_chooser_button_get_heading,

	gtk_frame_get_label,

	gtk_assistant_get_page_title,

	gtk_builder_get_translation_domain,

	gtk_buildable_get_name,

	gtk_button_get_label,

	gtk_menu_item_get_accel_path,

	gtk_clipboard_wait_for_text,

	gtk_color_button_get_title,

	gtk_color_selection_palette_to_string,

	gtk_expander_get_label,

	gtk_file_filter_get_name,
	gtk_file_chooser_get_filename,
	gtk_file_chooser_get_uri,
	gtk_file_chooser_get_current_folder_uri,

	gtk_file_chooser_get_preview_filename,
	gtk_file_chooser_get_preview_uri,
	gtk_file_chooser_button_get_title,

	gtk_font_button_get_title,

	gtk_font_button_get_font_name,
	gtk_font_selection_get_font_name,
	gtk_font_selection_get_preview_text,

	gtk_font_selection_dialog_get_font_name,
	gtk_font_selection_dialog_get_preview_text,

	gtk_icon_theme_get_example_icon_name,

	gtk_icon_info_get_filename,
	gtk_icon_info_get_display_name,

	gtk_link_button_get_uri,

	gtk_tool_button_get_label,
	gtk_tool_button_get_icon_name,

	gtk_notebook_get_group_name,
	gtk_notebook_get_tab_label_text,
	gtk_notebook_get_menu_label_text,

	gtk_numerable_icon_get_label,
	gtk_numerable_icon_get_background_icon_name,

	gtk_paper_size_get_name,
	gtk_paper_size_get_ppd_name,

	gtk_print_settings_get_printer,
	gtk_print_settings_get_media_type,
	gtk_print_operation_get_status_string,

	gtk_progress_bar_get_text,

	gtk_recent_info_get_uri,
	gtk_recent_info_get_description,
	gtk_recent_info_get_mime_type,
	gtk_recent_info_last_application,
	gtk_recent_info_get_short_name,
	gtk_recent_info_get_uri_display,

	gtk_recent_filter_get_name,
	gtk_recent_chooser_get_current_uri,
	gtk_status_icon_get_icon_name,

	gtk_status_icon_get_title,
	gtk_status_icon_get_tooltip_text,

	gtk_text_mark_get_name,

	gtk_text_buffer_get_text,
	gtk_text_buffer_get_slice,

	gtk_tool_item_group_get_label,

	gdk_display_get_name,
	gdk_rgba_to_string,
	gdk_pixbuf_format_get_name,
	gdk_keyval_name,
]



for o in (GtkVBox, GtkHBox): o._rpythonic_parent_classes_.append( GtkBox )
for o in (GtkCheckButton,): o._rpythonic_parent_classes_.append( GtkToggleButton )
for o in (GtkHScale, GtkVScale): o._rpythonic_parent_classes_.append( GtkScale )
for o in (GtkCheckButton,GtkToggleButton, GtkColorButton): o._rpythonic_parent_classes_.append( GtkButton )
for o in (GtkVPaned, GtkHPaned): o._rpythonic_parent_classes_.append( GtkPaned )

GTK_WIDGET_CLASSES = {
	GtkButton : gtk_button_new_with_label,
	GtkAdjustment : gtk_adjustment_new,
	GtkHScale : gtk_hscale_new,
	GtkVScale : gtk_vscale_new,
	GtkEntry : gtk_entry_new,
	GtkLabel : gtk_label_new,

	GtkToggleButton : gtk_toggle_button_new_with_label,
	GtkCheckButton : gtk_check_button_new_with_label,
	GtkSwitch : gtk_switch_new,

	GtkComboBox : gtk_combo_box_new,
	GtkComboBoxText : gtk_combo_box_text_new,

	GtkSpinButton : gtk_spin_button_new,

	GtkSeparator : gtk_separator_new,
	GtkHSeparator : gtk_hseparator_new,
	GtkVSeparator : gtk_vseparator_new,

	GtkVolumeButton : gtk_volume_button_new,
	GtkToolPalette : gtk_tool_palette_new,
	GtkToolItemGroup : gtk_tool_item_group_new,
	GtkTextView : gtk_text_view_new,
	GtkTable : gtk_table_new,
	GtkStatusbar : gtk_statusbar_new,
	GtkSpinner : gtk_spinner_new,

	GtkViewport : gtk_viewport_new,
	GtkScaleButton : gtk_scale_button_new,
	GtkRadioToolButton : gtk_radio_tool_button_new,
	GtkToggleToolButton : gtk_toggle_tool_button_new,
	GtkRadioButton : gtk_radio_button_new,

	GtkProgressBar : gtk_progress_bar_new,

	GtkMenuToolButton : gtk_menu_tool_button_new,
	GtkToolButton : gtk_tool_button_new,
	GtkLinkButton : gtk_link_button_new,

	GtkScrollbar : gtk_scrollbar_new,
	GtkScale : gtk_scale_new,
	GtkFontButton : gtk_font_button_new,

	GtkFileChooserButton : gtk_file_chooser_button_new,
	GtkColorSelection : gtk_color_selection_new,
	GtkColorButton : gtk_color_button_new_with_color,
	GtkHSV : gtk_hsv_new,

	GtkCalendar : gtk_calendar_new,
	GtkArrow : gtk_arrow_new,
	GtkMenu : gtk_menu_new,
	GtkMenuItem: gtk_menu_item_new_with_label,
}

GTK_CONTAINER_CLASSES = {
	GtkEventBox : gtk_event_box_new,
	GtkExpander : gtk_expander_new,
	GtkFixed : gtk_fixed_new,
	GtkFrame : gtk_frame_new,
	GtkWindow : gtk_window_new,
	GtkVBox : gtk_vbox_new,
	GtkHBox : gtk_hbox_new,
	GtkNotebook : gtk_notebook_new,
	GtkScrolledWindow : gtk_scrolled_window_new,

	GtkTreeView : gtk_tree_view_new,
	GtkDrawingArea : gtk_drawing_area_new,

	GtkVPaned : gtk_vpaned_new,
	GtkHPaned : gtk_hpaned_new,

	GtkGrid : gtk_grid_new,
	GtkOffscreenWindow : gtk_offscreen_window_new,

	GtkToolbar : gtk_toolbar_new,
	GtkMenuBar : gtk_menu_bar_new,

}

################### WebKitGTK #####################
if 'WebKitWebView' in globals():
	GTK_WIDGET_CLASSES[ WebKitWebView ] = webkit_web_view_new
	_RETURNS_CHARP_.append( webkit_dom_html_element_get_inner_html )
	_RETURNS_CHARP_.append( webkit_web_frame_get_title )
	_RETURNS_CHARP_.append( webkit_web_frame_get_uri )


################# Clutter GTK ######################
if 'GtkSocket' in globals():	# this is missing with Clutter
	GTK_WIDGET_CLASSES[ GtkSocket ] = gtk_socket_new

if 'ClutterActor' in globals():
	def _clutter_actor_animate_helper(self, atype, ms, **kw):
		anims = {}
		for prop in kw:
			val = GValue()
			gtype = g_type_from_name('gdouble')	# TODO other types
			assert gtype
			val = g_value_init( val, gtype )
			val.set_double( kw[prop] )

			if _ISPYTHON2: gprop = ctypes.pointer(ctypes.c_char_p(prop))
			else: gprop = ctypes.pointer(ctypes.c_char_p(prop.encode('utf-8')))

			anims[prop] = self.animatev(
				atype, ms,
				1, #num properties
				prop, val,
			)
		return anims
	ClutterActor.animate = _clutter_actor_animate_helper

	def _get_(self):
		x = ctypes.pointer( ctypes.c_double() )
		y = ctypes.pointer( ctypes.c_double() )
		self.clutter_actor_get_scale( x, y )
		return x.contents.value, y.contents.value
	ClutterActor.get_scale = _get_
	def _get_(self):
		x = ctypes.pointer( ctypes.c_double() )
		y = ctypes.pointer( ctypes.c_double() )
		self.clutter_actor_get_size( x, y )
		return x.contents.value, y.contents.value
	ClutterActor.get_size = _get_


################################################

for func in _RETURNS_CHARP_:
	func.return_wrapper = lambda pointer=None: _CHARP2STRING(pointer)


for d in (GTK_WIDGET_CLASSES, GTK_CONTAINER_CLASSES):
	for o in d:
		o._rpythonic_parent_classes_.append( GtkWidget )
		o._rpythonic_parent_classes_.append( GtkContainer )
		d[ o ].return_wrapper = o
		s = "lambda *args, **kw: %s(*args, **kw)"%d[o].name
		globals()[ o.__name__[3:] ] = eval(s)


def rgb2hsv( r,g,b ):
	h = ctypes.pointer( ctypes.c_double() )
	s = ctypes.pointer( ctypes.c_double() )
	v = ctypes.pointer( ctypes.c_double() )
	gtk_rgb_to_hsv( r,g,b, h,s,v )
	return h.contents.value, s.contents.value, v.contents.value

def hsv2rgb( h,s,v ):
	r = ctypes.pointer( ctypes.c_double() )
	g = ctypes.pointer( ctypes.c_double() )
	b = ctypes.pointer( ctypes.c_double() )
	gtk_hsv_to_rgb( h,s,v, r,g,b )
	return r.contents.value, g.contents.value, b.contents.value

def rgb2gdk( r, g, b ): return GdkColor(0,int(r*65535),int(g*65535),int(b*65535))
def gdk2rgb( c ): return (c.red/65536.0, c.green/65536.0, c.blue/65536.0)

