import os,sys

from pypy.rpython.lltypesystem import lltype, rffi

if '..' not in sys.path: sys.path.append( '..' )
from randroid.GLES.gl import *

def android_main(app):
	print( 'hello world' )
	#glClearColor( rffi.cast(rffi.FLOAT, 0.1), rffi.cast(rffi.FLOAT,0.8), rffi.cast(rffi.FLOAT,0.5), rffi.cast(rffi.FLOAT,1.0) )
	glClearColor( 0.1, 0.8, 0.5, 1.0 )
	glClear( GL_COLOR_BUFFER_BIT )
	return 1

def setup_egl( app ):
	display = e.eglGetDisplay()	# returns voidp
	major = rffi.cast( rffi.INTP, 0 )	#lltype.malloc( rffi.INTP, flavor='raw' )
	minor = rffi.cast( rffi.INTP, 0 )
	e.eglInitialize(display, major, minor)

	# Here specify the attributes of the desired configuration.
	# Below, we select an EGLConfig with at least 8 bits per color
	# component compatible with on-screen windows
	attribs = lltype.malloc( rffi.CFixedArray( rffi.INT, 9 ), flavor='raw' )
	_attribs = [
		e.EGL_SURFACE_TYPE,
		e.EGL_WINDOW_BIT,
		e.EGL_BLUE_SIZE, 8,
		e.EGL_GREEN_SIZE, 8,
		e.EGL_RED_SIZE, 8,
		e.EGL_NONE
	]
	for i in range(9): attribs[ i ] = rffi.cast( rffi.INT, _attribs[ i ] )
	#return
	#config = lltype.malloc( rffi.VOIDP.TO, flavor='raw' )
	config = lltype.nullptr( rffi.VOIDP.TO )
	# Here, the application chooses the configuration it desires. In this
	# sample, we have a very simplified selection process, where we pick
	# the first EGLConfig that matches our criteria.
	numConfigs = 1
	e.eglChooseConfig(display, attribs, config, 1, numConfigs)

	# EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
	# guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
	# As soon as we picked a EGLConfig, we can safely reconfigure the
	# ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. 
	e.eglGetConfigAttrib(display, config, e.EGL_NATIVE_VISUAL_ID, format)

	e.ANativeWindow_setBuffersGeometry( app.window, 0, 0, format )

	surface = e.eglCreateWindowSurface(display, config, app.window, rffi.NULL)
	context = e.eglCreateContext(display, config, rffi.NULL, rffi.NULL)

	e.eglMakeCurrent(display, surface, surface, context)

	#e.eglQuerySurface(display, surface, e.EGL_WIDTH, w)
	#e.eglQuerySurface(display, surface, e.EGL_HEIGHT, h)
	#self.width = w
	#self.height = h

	#self.display = display
	#self.context = context
	#self.surface = surface
	return display, context, surface
