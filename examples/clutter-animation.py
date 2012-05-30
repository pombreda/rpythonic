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
	if actor.get_scale()[0] < 1.0: scale = 2.0
	else: scale = 0.75
	actor.animate(
		clutter.CLUTTER_LINEAR, 4000,
		scale_x=scale,
	)

for i in range(10):
	button = gtk.Button('Click me')
	button.connect('clicked', sayhi, gact)
	box.pack_start( button )
box.show_all()	# required

button = gtk.Button('gtk-ok')
button.set_use_stock(True)
vbox.pack_end(button, False, False, 0)
button.show()

window.show_all()
while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('exit')

