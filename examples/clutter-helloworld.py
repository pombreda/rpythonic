import os, sys, time, ctypes
import libclutter_gtk as clutter
gtk = cluttergtk = clutter
print('trying gtk init')
gtk.init()
cluttergtk.gtk_clutter_init( ctypes.pointer(ctypes.c_int(0)) )
print('ok........')

def on_stage_allocate (actor, pspec, rect, icon):
    (stage_w, stage_h) = actor.get_size()
    # the rectangle has its anchor point set
    rect.set_position(stage_w / 2, stage_h / 2)
    icon.set_position(stage_w / 2, stage_h / 2)


window = gtk.Window()
#window.connect('destroy', gtk.main_quit)
print(window)
window.set_title('ctypes clutter test')
vbox = gtk.VBox(False, 6)
window.add(vbox)

embed = cluttergtk.clutter_embed_new()
print('embed',embed, dir(embed))
vbox.pack_start(embed, True, True, 0)
embed.set_size_request(300, 300)

# we need to realize the widget before we get the stage
embed.realize()
print('embed realized')
stage = cluttergtk.clutter_embed_get_stage( embed )	#embed.get_stage()
clr = clutter.clutter_color_new( 0,0,127,0 )
clutter.clutter_stage_set_color(stage, clr)

rect = clutter.clutter_rectangle_new()	#clutter.Rectangle()
print(rect,dir(rect))
clr = clutter.clutter_color_new( 100,1,0,100 )
clutter.clutter_rectangle_set_color(rect, clr)	#rect.set_color( clr )
rect.set_size(100, 100)
rect.set_anchor_point(50, 50)
rect.set_position(150, 150)
rect.set_rotation(clutter.CLUTTER_X_AXIS, 45.0, 0, 0, 0)
#stage.add(rect)
rect.set_parent(stage)

#cluttergtk.Texture()
tex = clutter.clutter_texture_new_from_file(
	'/usr/share/pixmaps/splash/gnome-splash.png',
)
clutter.clutter_texture_set_from_file(tex,'/usr/share/pixmaps/splash/gnome-splash.png')
tex.set_size(100, 100)
tex.set_anchor_point(50, 50)
tex.set_position(150, 150)
tex.set_rotation(clutter.CLUTTER_Z_AXIS, 60.0, 0, 0, 0)
#stage.add(tex)
tex.set_parent( stage )

# update the position of the actors when the stage changes
# size due to an allocation
#stage.connect('notify::allocation', on_stage_allocate, rect, tex)

button = gtk.Button('Click me to quit')
#button.connect('clicked', lambda:gtk.main_quit())
vbox.pack_end(button, False, False, 0)
button.show()

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


