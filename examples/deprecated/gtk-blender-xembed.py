#!/usr/bin/python
import os,sys, time, subprocess
import wnck	# apt-get install gnome-python-extras

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
gtk = rpythonic.module( 'gtk' )
assert gtk
gtk.init()

class MultiBlender(object):
	def __init__(self, blenders=2):
		self._num_blenders = blenders
		self._not_ready = self._num_blenders
		self._waiting = False
		self._window = win = gtk.Window()	#gtk.window_new( gtk.GTK_WINDOW_TOPLEVEL )
		self._screen = wnck.screen_get_default()
		print( win )
		#win.set_title( 'ctypes_ooapi_gtk_test')	# segfaults, name too long?
		win.set_title('GtkBlender')
		#win.set_default_size( 640, 480 )
		win.set_border_width( 2 )
		win.connect( 'destroy', lambda *args: gtk.main_quit() )

		frame = gtk.Frame()
		win.add( frame )
		frame.set_border_width( 4 )

		self._notebook = note = gtk.Notebook()
		frame.add( note )
		self._pages = []
		self._sockets = []
		self._procs = []
		self._plugs = 0
		for i in range( blenders ):
			exe = os.path.expanduser('~/Blender25/blender')
			self._procs.append(
				subprocess.Popen( [exe]+ '--window-geometry 0 0 640 480'.split() )
			)
			frame = gtk.Frame(); self._pages.append( frame )
			frame.set_size_request( 640, 480 )
			note.append_page( frame, gtk.Label( 'blender %s' %i ) )

		win.show_all()
		self._blender_windows = []



	def add_blender_page(self):
		self._waiting = True
		self._not_ready -= 1
		wins = None
		for i in range( 10 ): 	# only try this ten times
			time.sleep( 0.25 )
			wins = self.get_blender_windows()
			if wins: break
			if gtk.gtk_events_pending(): gtk.gtk_main_iteration()	# required
		if not wins: assert 0	# this should never happen
		else:
			w = wins[0]
			index = len(self._sockets)
			frame = self._pages[ index ]
			soc = gtk.gtk_socket_new()
			self._sockets.append( soc )
			soc.connect('plug-added', self.on_plug)
			frame.add( soc )
			frame.show_all()

			wid = w.get_xid()
			w.shade()	# shade is the trick
			w.make_below()
			w.set_skip_pager(True)
			self._blender_windows.append( w )

			assert index == self._notebook.get_current_page()
			print( 'socket add id', wid )
			soc.gtk_socket_add_id( wid )	# on_plug is called right after
			print( '----------ok---------' )

			if index+1 < len(self._pages):
				self._notebook.set_current_page( index+1 )

	def get_blender_windows(self):
		screen = self._screen
		screen.force_update()
		bwindows = []
		for w in screen.get_windows():
			wname = w.get_name()
			if w.get_name() == 'Blender':
				print( 'FOUND BLENDER WINDOW', w )
				bwindows.append( w )
		return bwindows

	def on_plug(self, *args):
		self._waiting = False
		self._plugs += 1
		print( 'on plug:', self._plugs )
		for i,soc in enumerate(self._sockets):
			sub = soc.gtk_socket_get_plug_window()
			sub.set_title('hack%s' %i)
			bwin = self._blender_windows[ i ]
			bwin.unshade()

		if self._plugs == self._num_blenders:
			print( '---embed complete---' )
		else:
			print( '...waiting for other blenders to embed...' )

mb = MultiBlender()
while True:
	if gtk.gtk_events_pending(): gtk.gtk_main_iteration()
	elif mb._not_ready and not mb._waiting:
		mb.add_blender_page()

print('exit')

