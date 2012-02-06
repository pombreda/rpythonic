#!/usr/bin/python
# may5th 2011, test ported to ctypes opencv
import os,sys, time

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic

cv = rpythonic.module( 'cv' )
assert cv
gui = rpythonic.module( 'highgui' )
assert gui

class Camera(object):
	def __init__(self):
		self.active = False
		self.index = 0
		self.cam = gui.CreateCameraCapture(self.index)
		self.resize( 640, 480 )
		gui.NamedWindow('webcam', 1)

	def resize( self, x,y ):
		self.width = x
		self.height = y
		self.cam.SetCaptureProperty( gui.CV_CAP_PROP_FRAME_WIDTH, self.width )
		self.cam.SetCaptureProperty( gui.CV_CAP_PROP_FRAME_HEIGHT, self.height )

	def loop( self ):
		self.active = True
		while self.active:
			_frame = self.cam.QueryFrame()
			gui.ShowImage( 'webcam', _frame )
			gui.WaitKey(1)

c = Camera()
c.loop()


