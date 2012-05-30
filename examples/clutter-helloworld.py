#!/usr/bin/pypy
#!/usr/bin/python
# this also works in python3

import os, sys, time, ctypes
import libclutter_gtk as clutter
gtk = clutter

clutter.gtk_clutter_init( ctypes.pointer(ctypes.c_int(0)) )
gtk.init()	# comes after clutter init


window = gtk.Window()
#window.connect('destroy', gtk.main_quit)
print(window)
window.set_title('ctypes clutter test')
vbox = gtk.VBox(False, 6)
window.add(vbox)

embed = clutter.gtk_clutter_embed_new()
vbox.pack_start(embed, True, True, 0)
embed.set_size_request(300, 300)

# we need to realize the widget before we get the stage
embed.realize()
print('embed realized')
stage = clutter.gtk_clutter_embed_get_stage( embed )	#embed.get_stage()
clr = clutter.color_new( 0,0,127,100 )
clutter.clutter_stage_set_color(stage, clr)

rect = clutter.rectangle_new()	#clutter.Rectangle()
clr = clutter.color_new( 100,1,0,100 )
clutter.rectangle_set_color(rect, clr)	#rect.set_color( clr )
rect.set_size(100, 100)
rect.set_anchor_point(50, 50)
rect.set_position(150, 150)
rect.set_rotation(clutter.CLUTTER_X_AXIS, 45.0, 0, 0, 0)
clutter.container_add_actor( stage, rect )

tex = clutter.texture_new_from_file(
	'/usr/share/pixmaps/splash/gnome-splash.png', None
)
#clutter.clutter_texture_set_from_file(tex,'/usr/share/pixmaps/splash/gnome-splash.png')
tex.set_size(100, 100)
tex.set_anchor_point(50, 50)
tex.set_position(150, 150)
tex.set_rotation(clutter.CLUTTER_Z_AXIS, 60.0, 0, 0, 0)
clutter.container_add_actor( stage, tex )

# update the position of the actors when the stage changes
# size due to an allocation
#stage.connect('notify::allocation', on_stage_allocate, rect, tex)

def sayhi(b): print('hello world')
button = gtk.Button('Click me')
button.connect('clicked', sayhi)
#vbox.pack_end(button, False, False, 0)
gact = clutter.gtk_clutter_actor_new_with_contents( button )
#gact.set_size(100, 100)
gact.set_anchor_point(50, 50)
gact.set_position(150, 150)
gact.set_rotation(clutter.CLUTTER_Z_AXIS, 80.0, 0, 0, 0)
button.show()
clutter.container_add_actor( stage, gact )


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


