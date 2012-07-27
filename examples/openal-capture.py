#!/usr/bin/python

import os,sys, time, random, ctypes
import openal as al
import fftw


class Audio(object):
	def generate_stream_buffers(self):
		if self.output_buffer_ids:
			self.buffer_trash.insert( 0, self.output_buffer_ids )

		self.output_buffer_ids = (ctypes.c_uint * self.num_output_buffers)()
		al.GenBuffers( self.num_output_buffers, ctypes.pointer(self.output_buffer_ids) )
		assert al.GetError() == al.AL_NO_ERROR

	def __init__(self, frequency=22050):
		self.format=al.AL_FORMAT_MONO16
		self.output_buffer_index = 0
		self.playing = False

		self.frequency = frequency
		self.buffersize = 1024

		self.output = al.OpenDevice()
		self.context = al.CreateContext( self.output )
		al.MakeContextCurrent( self.context )

		self.output_buffer_ids = None
		self.num_output_buffers = 16
		self.buffer_trash = []
		self.generate_stream_buffers()


		self.num_speakers = 4
		self.speaker_ids = (ctypes.c_uint * self.num_speakers)()
		al.GenSources( 4, self.speaker_ids )
		assert al.GetError() == al.AL_NO_ERROR

		self.fftw_buffer_type = ( ctypes.c_double * 2 * self.buffersize )


		######## capture #######
		self.input = al.CaptureOpenDevice( None, self.frequency, self.format, self.buffersize*2 )

		#buff = (ctypes.c_byte *(self.buffersize*2))()
		self.input_buffer = (ctypes.c_int16 * self.buffersize)()
		self.input_buffer_ptr = ctypes.pointer( self.input_buffer )


	def sync(self):
		ready = ctypes.pointer(ctypes.c_int())
		al.alcGetIntegerv( self.input, al.ALC_CAPTURE_SAMPLES, 1, ready )
		print( ready.contents.value )
		if ready.contents.value >= self.buffersize:
			n = self.buffersize
			al.CaptureSamples( self.input, self.input_buffer_ptr, n )
		#if ready.contents.value > self.buffersize/2:
		#	n = int(self.buffersize / 2)
		#	al.CaptureSamples( self.input, self.input_buffer, n )
		else:
			print('waiting')
			return

		#al.MakeContextCurrent( self.context )

		sid = self.speaker_ids[0]


		self.output_buffer_index += 1
		if self.output_buffer_index == self.num_output_buffers:
			self.output_buffer_index = 0
			#al.SourceUnqueueBuffers( sid, self.num_output_buffers-1, self.output_buffer_ids )
			self.generate_stream_buffers()
		id = self.output_buffer_ids[ self.output_buffer_index ]

		#print('N',n)
		#print('ID',id)
		al.BufferData(
			id,
			self.format, 
			self.input_buffer_ptr, 
			n*2, 	# size in bytes
			self.frequency,
		)
		assert al.GetError() == al.AL_NO_ERROR


		#array = (ctypes.c_uint*2)()
		#array[0] = id
		#al.SourceQueueBuffers( sid, 1, array )
		al.SourceQueueBuffers( sid, 1, ctypes.pointer(ctypes.c_uint(id)) )
		error = al.GetError()
		if error != al.AL_NO_ERROR:
			print('ERROR', error)
			return

		if not self.playing:
			self.playing = True
			al.SourcePlay( sid )
			#al.Sourcei( sid, al.AL_LOOPING, al.AL_TRUE )


		seconds = ctypes.pointer( ctypes.c_float(0.0) )
		al.GetSourcef( sid, al.AL_SEC_OFFSET, seconds )
		print('seconds:', seconds.contents.value)

		info = {}
		ret = ctypes.pointer( ctypes.c_int(0) )
		for tag in 'AL_BYTE_OFFSET AL_SOURCE_TYPE AL_LOOPING AL_BUFFER AL_SOURCE_STATE AL_BUFFERS_QUEUED AL_BUFFERS_PROCESSED'.split():
			param = getattr(al, tag)
			al.GetSourcei( sid, param, ret )
			print(tag, ret.contents.value)
			info[tag] = ret.contents.value

		#assert info['AL_SOURCE_STATE'] == al.AL_PLAYING
		if info['AL_SOURCE_STATE'] != al.AL_PLAYING:
			al.SourcePlay( sid )
		if info['AL_BUFFERS_PROCESSED'] > self.num_output_buffers:
			buff = self.buffer_trash.pop()
			ptr = ctypes.pointer( buff )
			print('clearing trash - unqueue', ptr)
			al.SourceUnqueueBuffers( 
				sid,
				self.num_output_buffers,
				ptr
			)
			e = al.GetError()
			print('error',e)
			assert e == al.AL_NO_ERROR

			print('clearing trash', ptr)
			al.DeleteBuffers( 
				self.num_output_buffers,
				ptr
			)
			e = al.GetError()
			print('error',e)
			assert e == al.AL_NO_ERROR



		complex = ctypes.c_double*2
		inbuff = self.fftw_buffer_type( *[complex(v,.0) for v in self.input_buffer] )
		outbuff = self.fftw_buffer_type()

		plan = fftw.plan_dft_1d( self.buffersize, inbuff, outbuff, fftw.FORWARD, fftw.ESTIMATE )
		print(plan)
		fftw.execute( plan )
		#for i in range( self.buffersize ):
		#	print(outbuff[i][0], outbuff[i][1])
		print('-'*80)

	def start_capture( self ):
		al.CaptureStart( self.input )

	def stop_capture( self ):
		al.CaptureStop( self.input )

	def exit(self):
		#ctx=al.GetCurrentContext()
		#dev = al.GetContextsDevice(ctx)
		al.DestroyContext( self.context )
		al.CloseDevice( self.output )
		al.CloseDevice( self.input )

		print('openal test complete')

def test():
	a = Audio()
	a.start_capture()
	for i in range(900):
		a.sync()
		time.sleep(0.033)
	a.stop_capture()
	a.exit()
	print('test complete')

def test_threaded1():
	a = Audio()
	a.start_capture()
	while True:	# while fails if not in a thread
		a.sync()
		time.sleep(0.033)
	a.stop_capture()
	a.exit()
	print('thread test complete')

def test_threaded2():
	a = Audio()
	a.start_capture()
	while True:	# while fails if not in a thread
		a.sync()
		time.sleep(0.033)
	a.stop_capture()
	a.exit()
	print('thread test complete')


import threading
#threading._start_new_thread(test_threaded1, () )
#time.sleep(60)

a = Audio()
threading._start_new_thread(a.start_capture, ())
for i in range(900):
	a.sync()
	time.sleep(0.033)
a.stop_capture()
a.exit()




