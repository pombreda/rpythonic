#!/usr/bin/python3
# updated Dec4 2011 - pypy compatible (almost)

import os,sys, time, ctypes
import gtk3 as gtk

def drag_begin(source, context):
	print('drag begin')

def drag_end(source,context):
	print('drag end')
	print(source,context)

def drag_motion(wid, context, x, y, time):	# not required
	print(wid,context,x,y,time)
	#context.drag_status(gtk.gdk.ACTION_COPY, time)
	return True

def drag_drop(dest, context, x, y, time):
	print('drag drop')
	print(dest,context,x,y,time)
	#source = context.get_source_widget()
	#pattern = source._drop_data
	#context.finish(True, False, time)


gtk.init()
win = gtk.Window()
print( win )

win.set_title('hello world')		# fails in pypy
win.set_default_size( 320, 240 )


def exit(args): gtk.main_quit()
win.connect( 'destroy', exit )

frame = gtk.Frame('hello world')

def enter_notify(args): print('enter nofity')
#frame.add_events( gtk.GDK_ENTER_NOTIFY_MASK )	# segfaults
#frame.connect('enter-notify-event',enter_notify)				# segfaults


frame.set_border_width(6)
frame.set_shadow_type( gtk.GTK_SHADOW_ETCHED_IN )		# not available with all themes?
win.add( frame )

box = gtk.VBox()
frame.add( box )


def callback(a,b):
	print('-----------python callback-----------')
	print( a )
	print( b )
	return 0

row = gtk.HBox()
box.pack_start( row, expand=False )
for i in range(10):
	u = "test%s"%i
	button = gtk.button_new_with_label(u)
	button.connect( 'clicked', callback, 'arg-to-cb%s'%i )
	row.pack_start( button )

adjust = gtk.Adjustment(
	value=1, 
	lower=0, upper=255, 
)
def adjust_callback(adj):
	print( adj.get_value() )
adjust.connect('value-changed', adjust_callback)
# this callback fails in pypy 
#	(<unknown>:2472): GLib-GObject-WARNING **: 
#	/build/buildd/glib2.0-2.28.6/./gobject/gsignal.c:2275: signal `?\u0008\u000e' #is invalid for instance `0xbff6288'

scale = gtk.HScale( adjust )
scale.set_value_pos(gtk.POS_RIGHT)
scale.set_digits(0)
box.pack_start( scale )


####### dnd DEST #######
a = gtk.Label('drop here')
box.pack_start( a, expand=True )

target = gtk.target_entry_new( 'test',1,gtk.TARGET_SAME_APP )
a.drag_dest_set(
	gtk.DEST_DEFAULT_ALL, #gtk.DEST_DEFAULT_MOTION
	target, 1, 
	gtk.GDK_ACTION_COPY
)

#a.connect('drag-motion', drag_motion)	# NOT REQUIRED
#a.drag_dest_set_track_motion(True)
a.connect('drag-drop', drag_drop)


####### dnd SOURCE ########
a = gtk.EventBox(); a.add( gtk.Label('drag me') )
box.pack_start( a, expand=False )

a.drag_source_set(
	gtk.GDK_BUTTON1_MASK, 
	target, 1, 
	gtk.GDK_ACTION_COPY
)
a.connect('drag-begin', drag_begin)
a.connect('drag-end', drag_end)



##############################
if 0:	# segfaults!?
	def on_motion(w,event): print('on motion',w,event)
	#a = gtk.Label('motion notify event')
	b = gtk.EventBox(); b.add( gtk.Label('drag me') )
	box.pack_start( b, expand=True )
	b.add_events( gtk.GDK_POINTER_MOTION_MASK )
	b.connect('motion-notify-event', on_motion)



def change_combo(combo):
	print('combo changed', combo)
	print( combo.get_active_text() )

combo = gtk.ComboBoxText()
box.pack_start( combo )
for txt in 'hello world'.split(): combo.append('id', txt)
combo.connect('changed', change_combo )



win.show_all()
#gtk.main()
while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('exit')

