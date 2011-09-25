#!/usr/bin/python
# updated sept 25, 2011
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

	def widget( self, notebook ):
		cspace = ColorSpacesByValue[ self.colorspace ]
		tag = cspace.split('2')[-1]
		page = gtk.HBox()
		h = gtk.HBox()
		b = gtk.CheckButton()
		#TODO b.connect('toggled', lambda b,lay: setattr(lay,'active',bool(b.get_active())), layer)
		b.set_active( bool(self.active) )
		h.pack_start( b, expand=False )
		h.pack_start( gtk.Label(tag) )
		notebook.append_page( page, h )

		h.show_all()		# required - Gtk bug

		col1, col2 = gtk.VBox(), gtk.VBox()
		page.pack_start( col1, expand=False )
		page.pack_start( col2, expand=True )

		fxgroups = {}
		for name in FXtypes:
			bx = gtk.VBox()
			fxgroups[ name.split('FX')[-1] ] = bx
			frame = gtk.Frame(); frame.add( bx )
			val = getattr( self, name )
			b = gtk.CheckButton( name )
			b.set_active( bool(val) )
			# TODO b.connect('toggled', lambda b,lay,nam: setattr(lay,nam,bool(b.get_active())), layer, name)
			frame.set_label_widget( b )
			col2.pack_end( frame )

		_skip = ['colorspace','active', 'widget']+FXtypes
		for name in dir(self):
			if not name.startswith('_') and name not in _skip:
				val = getattr( self, name )
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
						b.set_active( bool(getattr(self,name)) )
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


class App(object):
	def exit(self, *args):
		self.active = False
		self.webcam.active = False

	def __init__(self, webcam ):
		self.webcam = webcam
		self.active = True
		self.window = win = gtk.window_new()		# do not use gtk.Window()
		win.set_title( 'OpenCV+GTK' )
		win.connect('destroy', self.exit )
		root = gtk.HBox(); root.set_border_width( 3 )
		win.add( root )

		root.pack_start( webcam.gtk_image )

		split = gtk.VBox()
		root.pack_start( split )

		header = gtk.HBox(); split.pack_start( header, expand=False )
		
		note = gtk.Notebook()
		note.set_tab_pos( gtk.POS_RIGHT )
		split.pack_start( note )

		for layer in webcam.layers:
			layer.widget( note )


		win.show_all()

	def main(self):
		while self.active:
			if gtk.gtk_events_pending():
				while gtk.gtk_events_pending():
					gtk.gtk_main_iteration()

			self.webcam.iterate()

class WebCamera(object):
	BUFFER_TYPE = (ctypes.c_ubyte * (640*480*3))

	_default_spaces = [ 
		cv.CV_BGR2RGB, 
		cv.CV_BGR2HSV,
		cv.CV_BGR2Lab,
		cv.CV_BGR2YCrCb,
	]

	def __init__(self):
		self.active = True
		self.index = 0
		self.cam = gui.CreateCameraCapture(self.index)
		self.resize( 640, 480 )

		########### Layer Configs ############
		self.layers = []
		for colorspace in self._default_spaces:
			self.layers.append( LayerConfig( colorspace ) )
		self.layers[0].active = True


		raw = self.BUFFER_TYPE()
		for x in range( 640*480*3 ): raw[x] = 64
		ptr = ctypes.pointer( raw )
		pix = gtk.gdk_pixbuf_new_from_data(
			ptr, 
			gtk.GDK_COLORSPACE_RGB,
			False,	# ALPHA
			8,		# bits per sample
			640,
			480,
			640*3,	# row-stride
		)
		img = gtk.gtk_image_new_from_pixbuf( pix )	# returns GtkWidget, should return GtkImage
		self.gtk_image = img


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

	def iterate( self ):
		_rgb8 = self._rgb8
		_rgb32 = self._rgb32
		_gray8 = self._gray8
		_gray32 = self._gray32

		if self.active:

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

				self.update_preview_image( a.imageData )


	def update_preview_image(self, pointer ):
		#ptr = ctypes.pointer( BUFFER_TYPE() )
		#ctypes.memmove( ptr, pointer, 640*480*3 )	# not required
		pix = gtk.gdk_pixbuf_new_from_data(
			pointer, 
			gtk.GDK_COLORSPACE_RGB,
			False,	# ALPHA
			8,		# bits per sample
			640,
			480,
			640*3,	# row-stride
		)
		gtk.image_set_from_pixbuf( self.gtk_image, pix )


c = WebCamera()
app = App( c )
app.main()

