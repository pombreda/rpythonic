#!/usr/bin/python
# June 2012, test ported to opencv2
import os,sys, time
import opencv_core as cv
import opencv_highgui as gui

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


