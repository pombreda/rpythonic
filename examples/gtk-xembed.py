#!/usr/bin/python
import os,sys, time, subprocess
import gtk3 as gtk


class Widget(object):
	def exit(self, arg):
		self.active = False

	def __init__(self, parent ):
		self.active = True
		self.socket = gtk.Socket()
		parent.add( self.socket )
		self.socket.connect('plug-added', self.on_plug)


	def popen( self, cmd, name ):
		self.cmd = cmd
		self.name = name
		self.proc = subprocess.Popen( cmd )
		for i in range(10):
			time.sleep(1)
			wid = self.get_window_xid(self.name)
			if wid: break
		print('XID', wid)
		self.socket.add_id( wid )

	def get_window_xid( self, name ):
		p =os.popen('xwininfo -int -name %s' %name)
		data = p.read().strip()
		p.close()
		print( data )
		if data.startswith('xwininfo: error:'): return None
		elif data:
			lines = data.splitlines()
			if sys.version_info.major == 3: return int( lines[0].split()[3] )
			else: return long( lines[0].split()[3] )

	def on_plug(self, args):
		print( 'on plug...' )
		win = self.socket.get_plug_window()
		print(win)
		#win.set_title('stolen window')
		print(dir(win))
		#win.show()
		#win.unfullscreen()
		#win.iconify()
		#win.deiconify()
		#win.resize( 640, 480 )
		print('width', win.get_width())
		print('height', win.get_height())

if __name__ == '__main__':
	gtk.init()

	win = gtk.Window()
	win.set_size_request( 640, 480 )		# xembed requires window is >=
	win.set_title( 'xembed test' )
	frame = gtk.Frame('xembed test')
	win.add( frame )
	widget = Widget( frame )

	win.connect('destroy', widget.exit )
	win.show_all()

	ok = False
	while widget.active:
		if gtk.gtk_events_pending():
			while gtk.gtk_events_pending():
				gtk.gtk_main_iteration()

		if not ok:		# must come after gtk iteration
			ok = True
			widget.popen( ['/usr/bin/gcalctool'], 'Calculator' )

			############ blender quits without --window-borderless ###########
			#cmd = [os.path.expanduser('~/Blender2.6/blender'), '--window-borderless']
			#cmd += '--window-geometry 0 0 640 480'.split()
			#widget.popen( cmd, 'Blender' )



