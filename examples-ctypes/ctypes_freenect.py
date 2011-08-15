#!/usr/bin/python
# updated may26th 2011
import os, sys, time, ctypes
if '..' not in sys.path: sys.path.append( '..' )

import rpythonic
freenect = rpythonic.module( 'libfreenect' )
assert freenect
import time


print( 'creating kinect context' )
#context = freenect_context()		# ctypes can not deal with opaque structs?
context = ctypes.c_void_p()			# this won't work either
contextptr = ctypes.pointer( context )
status = freenect.init(contextptr, None)
print('init status', status )

freenect.set_log_level( contextptr, 1 )

numdevs = freenect.num_devices( contextptr )
print( 'num devices', numdevs )

#f_dev = freenect.freenect_device()
f_dev = freenect.device()
devptr = ctypes.pointer( f_dev() )

#status = freenect_open_device( contextptr, ctypes.byref(devptr), 0 )
status = freenect.open_device( contextptr, ctypes.byref(devptr), 0 )
print('open status', status )

freenect_set_led( devptr, LED_RED)
time.sleep(0.2)
freenect_set_led( devptr, LED_YELLOW)
time.sleep(0.2)
freenect_set_led( devptr, LED_GREEN)
time.sleep(0.2)


#void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
def depth_cb( devptr, data, timestamp ):
	print( 'depth callback' )
	print( data )

def video_cb( devptr, data, timestamp ):
	print( 'rgb callback' )
	print( data )


freenect_set_depth_callback(devptr, depth_cb );
#freenect_set_video_callback(devptr, video_cb);

freenect_set_video_format(devptr, FREENECT_VIDEO_RGB);
freenect_set_depth_format(devptr, FREENECT_DEPTH_11BIT);

buff = (ctypes.c_ubyte * FREENECT_FRAME_PIX)()
rgb_back = ctypes.pointer( buff )
#freenect_set_video_buffer(devptr, ctypes.cast(rgb_back,ctypes.c_void_p));

buff = (ctypes.c_ubyte * FREENECT_IR_FRAME_PIX)()
buffptr = ctypes.pointer( buff )
freenect_set_depth_buffer( devptr, ctypes.cast(buffptr,ctypes.c_void_p) )

status = freenect_start_depth(devptr);
print( 'start depth status', status )

#status = freenect_start_video(devptr);
#print( 'start video status', status )
print('starting...')
time.sleep(1)
for i in range( 900 ):
	status = freenect_process_events( contextptr )
	if status < 0: print( 'error' ); break
	time.sleep(0.01)	# segmentation fault without this??
print('shutdown')

freenect_set_led( devptr, LED_RED)

freenect_stop_depth(devptr);
freenect_stop_video(devptr);

print('stopped')

freenect_close_device(devptr);
freenect_shutdown(contextptr);


print('test done')


