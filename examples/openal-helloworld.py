#!/usr/bin/python

import os,sys, time, random, ctypes
import openal as al

dev = al.OpenDevice()
print(dev)
ctx = al.CreateContext( dev )
print(ctx)
al.MakeContextCurrent( ctx )
eax = al.IsExtensionPresent('EAX2.0')
print('has EAX2.0', eax)

devenum = al.IsExtensionPresent('ALC_ENUMERATION_EXT')
print( 'has Device Enumeration Extension:', devenum)


cycle_buffers = 3
buffs = (ctypes.c_uint * cycle_buffers)()
al.GenBuffers( cycle_buffers, buffs )

error = al.GetError()
assert error == al.AL_NO_ERROR

for i in range( cycle_buffers ):
	print('buffer id', buffs[i] )

n = 1024*40
data = (ctypes.c_byte*n)()
for i in range(n):
	data[i] = 128 + int(random.uniform( -100, 100 ))
ptr = ctypes.pointer( data )
al.BufferData(
	buffs[0],
	al.AL_FORMAT_MONO8, 
	ptr, 
	n, # size in bytes
	11025,
)

sourceids = (ctypes.c_uint * 4)()
#ptr = ctypes.pointer( sourceids )
al.GenSources( 4, sourceids )

al.Sourcei( sourceids[0], al.AL_BUFFER, buffs[0] )
#sid = sourceids[0]
#bid = buffs[0]
#al.SourceQueueBuffers( sid, 1, ctypes.pointer(ctypes.c_uint(bid)) )

print('playing test sound')
al.SourcePlay( sourceids[0] )	# non blocking

seconds = ctypes.pointer( ctypes.c_float(0.0) )
info = {}
ret = ctypes.pointer( ctypes.c_int(0) )
for i in range(500):
	al.GetSourcef( sourceids[0], al.AL_SEC_OFFSET, seconds )
	print('seconds', seconds.contents.value)

	for tag in 'AL_BYTE_OFFSET AL_SOURCE_TYPE AL_LOOPING AL_BUFFER AL_SOURCE_STATE AL_BUFFERS_QUEUED AL_BUFFERS_PROCESSED'.split():
		param = getattr(al, tag)
		al.GetSourcei( sid, param, ret )
		print(tag, ret.contents.value)
		info[tag] = ret.contents.value

	time.sleep(0.01)
	if i and not info['AL_BYTE_OFFSET']: break

print('test sound played')


#ctx=al.GetCurrentContext()
#dev = al.GetContextsDevice(ctx)
al.DestroyContext( ctx )
al.CloseDevice( dev )

print('openal test complete')

