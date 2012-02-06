#!/usr/bin/python
# updated may26th 2011
import os, sys, time, ctypes
import libfreenect as freenect


print( 'creating kinect context' )
context = freenect.context()
#usbcon = freenect.libusb_context()
#freenect.libusb_init( usbcon )
#status = freenect.init(context, usbcon)
status = freenect.init(context, None)
print('init status', status )

freenect.set_log_level( context, 1 )

numdevs = freenect.num_devices( context )
print( 'num devices', numdevs )

#f_dev = freenect.freenect_device()
dev = freenect.device()
#devptr = ctypes.pointer( f_dev() )

status = freenect.open_device( context, ctypes.pointer(dev()), 0 )
#status = freenect.open_device( contextptr, ctypes.byref(devptr), 0 )
print('open status', status )

freenect.set_led( dev, freenect.LED_RED)
time.sleep(1)
freenect.set_led( dev, freenect.LED_YELLOW)
time.sleep(1)
freenect.set_led( dev, freenect.LED_GREEN)
time.sleep(1)

print('leds flashed')

#void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
def depth_cb( devptr, data, timestamp ):
	print( 'depth callback' )
	print( data )

def video_cb( devptr, data, timestamp ):
	print( 'rgb callback' )
	print( data )


freenect.set_depth_callback(dev, depth_cb );
#freenect_set_video_callback(devptr, video_cb);

#freenect.set_video_format(dev, freenect.VIDEO_RGB);
freenect.set_depth_format(dev, freenect.DEPTH_11BIT);

buff = (ctypes.c_ubyte * freenect.FRAME_PIX)()
rgb_back = ctypes.pointer( buff )
#freenect_set_video_buffer(devptr, ctypes.cast(rgb_back,ctypes.c_void_p));

buff = (ctypes.c_ubyte * freenect.IR_FRAME_PIX)()
buffptr = ctypes.pointer( buff )
#freenect.set_depth_buffer( dev, ctypes.cast(buffptr,ctypes.c_void_p) )
freenect.set_depth_buffer( dev, buffptr )

status = freenect.start_depth(dev)
print( 'start depth status', status )

#status = freenect_start_video(devptr);
#print( 'start video status', status )

import thread
def loop():
	print('starting...')
	time.sleep(1)
	for i in range( 100 ):
		status = freenect.process_events( context )
		if status < 0: print( 'error' ); break
		print(i)
	print('shutdown')
#thread.start_new_thread( loop, () )
loop()
#time.sleep(10)

freenect.set_led( dev, freenect.LED_RED)
print('stopped')

freenect.stop_depth(dev)
#freenect.stop_video(devptr)



freenect_close_device(devptr);
freenect_shutdown(contextptr);


print('test done')


