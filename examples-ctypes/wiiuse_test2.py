#!/usr/bin/python
# updated may21 2011
import os,sys, time, ctypes

if '..' not in sys.path: sys.path.append( '..' )
try:
	import rpythonic
	wii = rpythonic.module( 'wiiuse' )
except:
	import wiiuse as wii
assert wii

class Wiimote(object):
	def __init__(self):
		self.buttons = {}
		for char in 'ABUDLR-+H': self.buttons[char] = 0
		self.x = .0
		self.y = .0
		self.z = .0
	def update( self, wm ):
		bs = self.buttons
		for tag in 'UDLR+H-12AB': bs[ tag ] = 0
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_A)): bs['A'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_B)): bs['B'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_UP)): bs['U'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_DOWN)): bs['D'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_LEFT)): bs['L'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_RIGHT)): bs['R'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_MINUS)): bs['-'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_PLUS)): bs['+'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_ONE)): bs['1'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_TWO)): bs['2'] = 1
		if (wii.IS_PRESSED(wm, wii.WIIMOTE_BUTTON_HOME)): bs['H'] = 1
		print( bs )
		self.x = wm.contents.accel.x
		self.y = wm.contents.accel.y
		self.z = wm.contents.accel.z
		print( self.x, self.y, self.z )



class WiiMan(object):
	def __init__( self ):
		self._active = False
		self._nmotes = 2
		self._pointer = wii.wiiuse_init( self._nmotes )
		self.wiimotes = [ Wiimote() for i in range(self._nmotes) ]


	def exit( self ):
		self._active = False
		time.sleep(1)
		wii.wiiuse_cleanup( self._pointer, self._nmotes)


	def start( self ):
		found = wii.find( self._pointer, self._nmotes, 5 )
		assert found
		print( 'found wiimotes', found )
		connected = wii.wiiuse_connect( self._pointer, self._nmotes )
		assert connected
		print( 'connected wiimotes', connected )

		for i in range( self._nmotes ):
			wm = self._pointer[i]
			wii.wiiuse_motion_sensing(wm, 1)
			wii.wiiuse_set_leds( wm, wii.WIIMOTE_LED_2)

		self._active = True
		return found

		
	def callback( self, mote ):
		print( mote )
		self.wiimotes[ mote.contents.uid - 1 ].update( mote )

	def iterate( self ):
		status = wii.wiiuse_update( self._pointer, self._nmotes, self.callback )



w = WiiMan()
w.start()
while True:
	w.iterate()

