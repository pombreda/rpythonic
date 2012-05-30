#!/usr/bin/pypy
#!/usr/bin/python
# this also works in python3

import os, sys, time, ctypes
import libclutter_gtk as clutter
gtk = clutter

clutter.gtk_clutter_init( ctypes.pointer(ctypes.c_int(0)) )
gtk.init()	# comes after clutter init


window = gtk.Window()
window.set_title('ctypes clutter test')
#window.connect('destroy', gtk.main_quit)
vbox = gtk.VBox(False, 6)
window.add(vbox)

embed = clutter.gtk_clutter_embed_new()
vbox.pack_start(embed, True, True, 0)
embed.set_size_request(300, 300)

# we need to realize the widget before we get the stage
embed.realize()
print('embed realized')
stage = clutter.gtk_clutter_embed_get_stage( embed )	#embed.get_stage()
clr = clutter.color_new( 0,0,127,10 )
clutter.stage_set_color(stage, clr)

box = gtk.VBox()
gact = clutter.gtk_clutter_actor_new_with_contents( box )
clutter.container_add_actor( stage, gact )
gact.set_scale( 0.75, 1.0 )

def sayhi(b, actor):
	print('hello world', actor)
	if 1:
		val = gtk.GValue()
		gtype = gtk.g_type_from_name('gdouble')
		assert gtype
		print('GTYPE',gtype)
		val = gtk.g_value_init( val, gtype )
		val.set_double( 2.0 )
		print('VAL',val)
		anim = actor.animatev(
			clutter.CLUTTER_LINEAR, 4000,
			1, #num properties
			ctypes.pointer(ctypes.c_char_p('scale-x')), val,
		)
		print(anim)
	#actor.set_scale( 2.0, 1.0 )

for i in range(10):
	button = gtk.Button('Click me')
	button.connect('clicked', sayhi, gact)
	#vbox.pack_end(button, False, False, 0)
	#gact.set_size(100, 100)
	#gact.set_anchor_point(50, 50)
	#gact.set_position(150, 150)
	#gact.set_rotation(clutter.CLUTTER_Z_AXIS, 80.0, 0, 0, 0)
	box.pack_start( button )
box.show_all()

button = gtk.Button('gtk-ok')
button.set_use_stock(True)
vbox.pack_end(button, False, False, 0)
button.show()

window.show_all()
while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('exit')

#if __name__ == '__main__': main()


