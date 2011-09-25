#!/usr/bin/python
# may5th 2011, test ported to ctypes opencv
import os,sys, time, ctypes

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic

cv = rpythonic.module( 'cv' )
assert cv
gui = rpythonic.module( 'highgui' )
assert gui

gtk = rpythonic.module( 'gtk' )
assert gtk
gtk.init()

#pix = gtk.gdk_pixbuf_new( gtk.GDK_COLORSPACE_RGB, True, 8, 640, 480)
#img = gtk.Image()
#assert 0

_colorspaces = '''
CV_BGR2GRAY
CV_BGR2HLS
CV_BGR2HSV
CV_BGR2Lab
CV_BGR2Luv
CV_BGR2RGB
CV_BGR2XYZ
CV_BGR2YCrCb
'''
ColorSpaces = {}
ColorSpacesByValue = {}
for name in _colorspaces.splitlines():
	name = name.strip()
	if name:
		value = getattr(cv,name)
		ColorSpacesByValue[ value ] = name
		ColorSpaces[ name ] = value

FXtypes = 'FXsplit FXstencil FXblur FXsobel FXathresh FXthresh FXdetect'.split()

class LayerConfig(object):

	def __init__(self, colorspace):
		self.colorspace = colorspace
		self.active = False
		self.alpha = 64
		self.blur = 2
		self.athresh_block_size = 3

		self.thresh_min = 32
		self.thresh_max = 200

		self.sobel_xorder = 1
		self.sobel_yorder = 1
		self.sobel_aperture = 5

		self.split_red = self.split_green = self.split_blue = False
		for fx in FXtypes: setattr(self, fx, False )

BG_RED = 0.098
BG_GREEN = 0.098
BG_BLUE = 0.099
BG_ALPHA = 0.0
BG_COLOR = (BG_RED,BG_GREEN,BG_BLUE,BG_ALPHA)

def transparent_window( win, color=BG_COLOR, decorate=False ):
	win.set_decorated( decorate )
	# Tell GTK+ that we want to draw the windows background ourself.
	# If we don't do this then GTK+ will clear the window to the
	# opaque theme default color, which isn't what we want.
	win.set_app_paintable(True)
	make_transparent( win, color )


#def expose_transparent(widget, event):
def expose_transparent():
	for widget in _TRANSPARENT_WIDGETS_:
		#cr = widget.window.cairo_create()
		cr = gtk.gdk_cairo_create( widget.window )
		#r,g,b,a = widget._trans_color_hack
		r,g,b,a = BG_COLOR
		cr.set_source_rgba(r, g, b, a) # Transparent
		# Draw the background
		cr.set_operator( gtk.cairo_operator['CAIRO_OPERATOR_SOURCE'] )
		cr.paint()
	return False

_TRANSPARENT_WIDGETS_ = []
def make_transparent(widget, color=BG_COLOR ):
	_TRANSPARENT_WIDGETS_.append( widget )
	#widget._trans_color_hack = color

	# The X server sends us an expose event when the window becomes
	# visible on screen. It means we need to draw the contents.  On a
	# composited desktop expose is normally only sent when the window
	# is put on the screen. On a non-composited desktop it can be
	# sent whenever the window is uncovered by another.
	#
	# The screen-changed event means the display to which we are
	# drawing changed. GTK+ supports migration of running
	# applications between X servers, which might not support the
	# same features, so we need to check each time.
	widget.connect('expose-event', expose_transparent)
	#win.connect('screen-changed', self.screen_changed)
	# To check if the display supports alpha channels, get the colormap
	screen = widget.get_screen()
	colormap = screen.get_rgba_colormap()
	# Now we have a colormap appropriate for the screen, use it
	widget.set_colormap(colormap)
	return False

BUFFER_TYPE = (ctypes.c_ubyte * (640*480*3))

class UI(object):
	def __init__(self, cam ):
		cam._ui = self
		self.active = True
		layers = self.layers = cam.layers		# shared

		## gtk ##
		self.window = win = gtk.window_new()		# do not use gtk.Window()
		#win.set_default_size( 900, 480 )
		win.set_title( 'remix' )
		win.connect('destroy', lambda *args: gtk.main_quit() )
		#transparent_window( win, decorate=True )
		root = gtk.HBox(); root.set_border_width( 3 )
		win.add( root )

		raw = BUFFER_TYPE()
		for x in range( 640*480*3 ): raw[x] = 64
		self.pixbuffer = ctypes.pointer( raw )
		cam._pixbuffer = self.pixbuffer
		#pix = gtk.gdk_pixbuf_new( gtk.GDK_COLORSPACE_RGB, True, 8, 640, 480)
		pix = gtk.gdk_pixbuf_new_from_data(
			self.pixbuffer, 
			gtk.GDK_COLORSPACE_RGB,
			False,	# ALPHA
			8,		# bits per sample
			640,
			480,
			640*3,	# row-stride
		)
		self._gdk_pixbuf = pix
		img = gtk.gtk_image_new_from_pixbuf( pix )	# returns GtkWidget, should return GtkImage
		self.image = img
		root.pack_start( img )
		#####################################
		#eb = gtk.EventBox()
		#root.pack_start( eb, expand=False )
		#self._drawing_area = da = gtk.DrawingArea()
		#da.set_size_request( 640,480 )
		#da.connect('realize', self.realize)
		#eb.add( da )
		#make_transparent(da)

		##################
		#eb = gtk.EventBox()
		#root.pack_start( eb, expand=True )
		#root.pack_start( eb, expand=True, fill=True, padding=0 )
		split = gtk.VBox(); #eb.add( split )
		root.pack_start( split )
		#########################make_transparent(split)

		

		header = gtk.HBox(); split.pack_start( header, expand=False )
		b = gtk.Button('open blender')
		#b.connect('clicked', lambda b: os.system('%s -P %s --blender &'%(PATH2BLENDER, sys.argv[0])) )
		header.pack_start( b, expand=False )

		b = gtk.ToggleButton('[-]'); b.set_active(True)
		#b.connect('toggled', lambda b: win.set_decorated( b.get_active() ) )
		header.pack_start( b, expand=False )
		

		#ex = gtk.Expander( 'settings' ); ex.set_expanded(False)
		#split.pack_start( ex, expand=False )
		#ex = gtk.Expander( 'adjust layers' ); ex.set_expanded(True)
		#split.pack_start( ex, expand=True)

		note = gtk.Notebook()
		note.set_tab_pos( gtk.POS_RIGHT )

		#ex.add( note )
		split.pack_start( note )
		for layer in layers:
				cspace = ColorSpacesByValue[ layer.colorspace ]
				tag = cspace.split('2')[-1]
				page = gtk.HBox()
				h = gtk.HBox()
				b = gtk.CheckButton()
				#TODO b.connect('toggled', lambda b,lay: setattr(lay,'active',bool(b.get_active())), layer)
				h.pack_start( b, expand=False )
				h.pack_start( gtk.Label(tag) )
				note.append_page( page, h )
				h.show_all()
				b.set_active( bool(layer.active) )

				col1, col2 = gtk.VBox(), gtk.VBox()
				page.pack_start( col1, expand=False )
				page.pack_start( col2, expand=True )

				fxgroups = {}
				for name in FXtypes:
					bx = gtk.VBox()
					fxgroups[ name.split('FX')[-1] ] = bx
					frame = gtk.Frame(); frame.add( bx )
					val = getattr( layer, name )
					b = gtk.CheckButton( name )
					b.set_active( bool(val) )
					# TODO b.connect('toggled', lambda b,lay,nam: setattr(lay,nam,bool(b.get_active())), layer, name)
					frame.set_label_widget( b )
					col2.pack_end( frame )

				for name in dir(layer):
					if not name.startswith('_') and name not in ['colorspace','active']+FXtypes:
						val = getattr( layer, name )
						bx = None
						for fx in fxgroups:
							if name.startswith(fx):
								bx = fxgroups[fx]
								break
						if not bx:
							adjust = gtk.Adjustment(
								value=val, 
								lower=0, upper=255, 
							)
							# TODO adjust.connect("value_changed", lambda a,lay,nam: setattr(lay,nam,int(a.value)), layer,name)
							scale = gtk.HScale( adjust ); scale.set_value_pos(gtk.POS_RIGHT)
							scale.set_digits(0)
							frame = gtk.Frame( name )
							frame.add( scale )
							col2.pack_start( frame )
						else:
							if fx=='split':
								b = gtk.CheckButton( name )
								b.set_active( bool(getattr(layer,name)) )
								# TODO b.connect('toggled', lambda b,lay,nam: setattr(lay,nam,bool(b.get_active())), layer, name)
								bx.pack_start( b )
							else:
								upper = 255
								lower = 0
								step = 1
								if name.startswith('sobel'):
									upper = 31; lower = 1; step=2
								adjust = gtk.Adjustment(
									value=val, 
									lower=lower, upper=upper, 
								)
								# TODO adjust.connect("value_changed", lambda a,lay,nam: setattr(lay,nam,int(a.value)), layer,name)
								scale = gtk.HScale( adjust ); scale.set_value_pos(gtk.GTK_POS_RIGHT)
								scale.set_digits(0)
								row = gtk.HBox()
								row.pack_start( gtk.Label( name.split(fx)[-1].replace('_',' ') ), expand=False )
								row.pack_start( scale )
								bx.pack_start( row )

		win.show_all()

class Camera(object):
	_default_spaces = [ 
		cv.CV_BGR2RGB, 
		cv.CV_BGR2HSV,
		cv.CV_BGR2Lab,
		cv.CV_BGR2YCrCb,
	]

	def __init__(self):
		self.active = False
		self.index = 0
		self.cam = gui.CreateCameraCapture(self.index)
		#self.win = gui.NamedWindow('webcam', 1)
		self.resize( 640, 480 )

		self.layers = []
		for colorspace in self._default_spaces:
			self.layers.append( LayerConfig( colorspace ) )
		self.layers[0].active = True

	def resize( self, x,y ):
		self.width = x
		self.height = y
		self.cam.SetCaptureProperty( gui.CV_CAP_PROP_FRAME_WIDTH, self.width )
		self.cam.SetCaptureProperty( gui.CV_CAP_PROP_FRAME_HEIGHT, self.height )

		self._rgb8 = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 3)
		self._rgb32 = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_32F, 3)
		self._gray8 = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 1)
		self._gray32 = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_32F, 1)

		self._R = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 1)
		self._G = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 1)
		self._B = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 1)
		self._A = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 1)

	def loop( self ):
		self.active = True

		_rgb8 = self._rgb8
		_rgb32 = self._rgb32
		_gray8 = self._gray8
		_gray32 = self._gray32

		while self.active:
			if gtk.gtk_events_pending():
				while gtk.gtk_events_pending():
					gtk.gtk_main_iteration()
			print('sync cam')
			_frame = self.cam.QueryFrame()	# IplImage from highgui lacks fancy methods

			for layer in self.layers:
				if not layer.active: continue
				a = cv.CreateImage( (self.width,self.height), cv.IPL_DEPTH_8U, 3 )		# has fancy methods
				cv.CvtColor( _frame, a, layer.colorspace )		# no _frame.CvtColor because its from highgui

				## FX
				if layer.FXsplit:
					_a = cv.CreateImage((self.width,self.height), cv.IPL_DEPTH_8U, 4)
					a.CvtColor(_a, cv.CV_RGB2RGBA)
					a.Split( self._R, self._G, self._B, self._A )
					if layer.split_red: a = self._R
					elif layer.split_green: a = self._G
					elif layer.split_blue: a = self._B


				if layer.FXblur:			# blur before threshing
					blur = layer.blur
					if blur < 1: blur = 1
					cv.Smooth( a, a, cv.CV_BLUR, blur )

				if layer.FXsobel and layer.sobel_aperture % 2 and layer.sobel_aperture < 32:
					if layer.sobel_xorder < layer.sobel_aperture and layer.sobel_yorder < layer.sobel_aperture:
						if a.nChannels == 1:
							cv.cvSobel( a, _gray32, layer.sobel_xorder, layer.sobel_yorder, layer.sobel_aperture )	#xorder, yorder, aperture
							cv.cvConvert( _gray32, a )#; cv.cvCvtColor(_sobel8, _sobel, cv.CV_GRAY2RGB)
						else:
							cv.cvSobel( a, _rgb32, layer.sobel_xorder, layer.sobel_yorder, layer.sobel_aperture )	#xorder, yorder, aperture
							cv.cvConvert( _rgb32, a )#; cv.cvCvtColor(_sobel8, _sobel, cv.CV_GRAY2RGB)

				if layer.FXthresh:
					cv.Threshold( a, a, layer.thresh_min, layer.thresh_max, cv.CV_THRESH_BINARY )

				if layer.FXathresh:
					blocksize = layer.athresh_block_size
					if blocksize <= 2: blocksize = 3
					if blocksize % 2 != 1: blocksize += 1
					if a.nChannels == 1:
						cv.AdaptiveThreshold(a, a, 255, cv.CV_ADAPTIVE_THRESH_MEAN_C, cv.CV_THRESH_BINARY, blocksize )
					else:
						cv.CvtColor(a, _gray8, cv.CV_RGB2GRAY)
						cv.AdaptiveThreshold(_gray8, _gray8, 255, cv.CV_ADAPTIVE_THRESH_MEAN_C, cv.CV_THRESH_BINARY, blocksize )
						cv.CvtColor(_gray8, a, cv.CV_GRAY2RGB)

				if a.nChannels == 1:
						cv.CvtColor(a, _rgb8, cv.CV_GRAY2RGB)
						a = _rgb8

				#gui.ShowImage( 'webcam', a )
				#gui.WaitKey(1)
				s = ctypes.string_at( a.imageData, 640*480*3 )
				#print( s )
				#ptr = ctypes.cast( a.imageData, ctypes.POINTER( BUFFER_TYPE ) )
				#ctypes.memmove( self._pixbuffer, ptr, 640*480*3 )
				#for i in range(640*480*3): self._pixbuffer.contents[i]= ord(s[i])

				raw = BUFFER_TYPE()
				for i in range( 640*480*3 ): raw[i] = ord(s[i])
				pixbuffer = ctypes.pointer( raw )
				#pix = gtk.gdk_pixbuf_new( gtk.GDK_COLORSPACE_RGB, True, 8, 640, 480)
				pix = gtk.gdk_pixbuf_new_from_data(
					pixbuffer, 
					gtk.GDK_COLORSPACE_RGB,
					False,	# ALPHA
					8,		# bits per sample
					640,
					480,
					640*3,	# row-stride
				)
				#self._ui.image.set_from_pixbuf( pix )	# TODO fix me
				gtk.image_set_from_pixbuf( self._ui.image, pix )
				#pix.copy_area( 0, 0, 640, 480, self._ui._gdk_pixbuf, 0, 0 )

		print('exit camera loop')

c = Camera()
ui = UI( c )
c.loop()
#gtk.main()

