#!/usr/bin/python
# updated Nov 2011
import os,sys, time
import wiiuse as wii

MAX_WIIMOTES = 2
context = wii.init( MAX_WIIMOTES )
print( context )

found = wii.find( context, MAX_WIIMOTES, 5 )
assert found
print( 'found wiimotes', found )

#connected = wii.connect( context, found )	# dont get confused by other connect function from bluez
connected = wii.wiiuse_connect( context, MAX_WIIMOTES )
assert connected

wii.set_leds( context[0], wii.WIIMOTE_LED_1 )



