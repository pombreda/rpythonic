#!/usr/bin/python
#!/usr/bin/pypy
# this also works in python3

import os, sys, time, ctypes
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
	sys.path.append(
		os.path.dirname(os.path.abspath(__file__))
	)

import libclutter_gtk as clutter
gtk = clutter
clutter.gtk_clutter_init( ctypes.pointer(ctypes.c_int(0)) )
gtk.init()	# comes after clutter init


window = gtk.Window()
window.set_title('ctypes clutter test')
#window.connect('destroy', gtk.main_quit)
vbox = gtk.VBox(False, 6)
window.add(vbox)

embed = clutter.gtk_clutter_embed_new()	# gtk-widget type
embed.set_size_request(300, 300)

sw = gtk.ScrolledWindow()
sw.set_size_request( 180, 420 )
sw.add_with_viewport( embed )
sw.set_policy(True,False)
vbox.pack_start(sw, True, True, 0)

## its ok to create actors before the stage is realized ##
box = gtk.VBox()
ACTOR = clutter.gtk_clutter_actor_new_with_contents( box )
ACTOR.set_scale( 0.75, 1.0 )

# we need to realize the widget before we get the stage #
'''
/**
 * SECTION:gtk-clutter-embed
 * @short_description: Widget for embedding a Clutter scene
 *
 * #GtkClutterEmbed is a GTK+ widget embedding a #ClutterStage. Using
 * a #GtkClutterEmbed widget is possible to build, show and interact with
 * a scene built using Clutter inside a GTK+ application.
 *
 * <note>To avoid flickering on show, you should call gtk_widget_show()
 * or gtk_widget_realize() before calling clutter_actor_show() on the
 * embedded #ClutterStage actor. This is needed for Clutter to be able
 * to paint on the #GtkClutterEmbed widget.</note>
 *
 * Since: 0.6
 */
'''
embed.realize()
print('embed realized')
stage = clutter.gtk_clutter_embed_get_stage( embed )
clr = clutter.color_new( 0,0,127,10 )
clutter.stage_set_color(stage, clr)

clutter.actor_show( stage )
clutter.container_add_actor( stage, ACTOR )

def sayhi(b, actor):
	print('hello world', actor)
	if actor.get_scale()[0] < 1.0: scale = 1.5
	else: scale = 0.75
	actor.animate(
		clutter.CLUTTER_LINEAR, 500,
		scale_x=scale,
	)

for i in range(10):
	ex = gtk.Expander('expand me')
	box.pack_start( ex )
	button = gtk.Button('Click me')
	button.connect('clicked', sayhi, ACTOR)
	ex.add( button )
box.show_all()	# required

button = gtk.Button('gtk-ok')
button.set_use_stock(True)
vbox.pack_end(button, False, False, 0)
#button.show()

window.show_all()
while True:
	#ACTOR.queue_redraw()
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

	clutter.clutter_redraw(stage)

print('exit')

