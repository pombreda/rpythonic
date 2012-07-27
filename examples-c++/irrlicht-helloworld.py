#!/usr/bin/python
# july 21, 2011
import os,sys, time, ctypes

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
irr = rpythonic.module( 'Irrlicht' )
assert irr

driverType = irr.E_DRIVER_TYPE['EDT_OPENGL']
windowSize = irr.dimension2du(0, 0, 0)
ier = irr.IEventReceiver()
abstract = irr.createDevice(driverType, windowSize, 16)	#, False, False, False, ier)
dev = irr.CIrrDeviceLinux( abstract )

abstract = dev.getVideoDriver()
driver = irr.COpenGLDriver( abstract )
#driver = irr.CNullDriver( abstract )	# scolor still random?

abstract = dev.getSceneManager()
man = irr.CSceneManager( abstract )

scolor = irr.SColor()
scolor.set( 255, 0, 0, 128 )	# should be blue - color becomes random?
assert scolor.getBlue() == 128

def draw():
	if dev.isWindowActive():
		print( 'drawing' )
		driver.beginScene(True, True, scolor)	# wrong color?
		man.drawAll()
		driver.endScene()

for i in range(20):
	dev.run()
	time.sleep(0.01)
	draw()


print('irrlicht test done')

