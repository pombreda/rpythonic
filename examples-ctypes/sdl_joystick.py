#!/usr/bin/python
# updated may20th 2011
import os,sys, time

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
sdl = rpythonic.module( 'SDL' )
assert sdl

sdl.SDL_Init( sdl.SDL_INIT_JOYSTICK )

n = sdl.SDL_NumJoysticks()
assert n

joy = sdl.SDL_JoystickOpen(0)
print(joy)

print(dir(joy))

print( 'running loop' )
for i in range( 2800 ):
	time.sleep(0.01)
	sdl.SDL_JoystickUpdate()
	for j in range( joy.NumAxes() ):
		print( 'axis', j, joy.GetAxis(j) / 32767.0 )	# -32768 to 32767
	for j in range( joy.NumButtons() ):
		print( 'button', j, joy.GetButton(j) )
	for j in range( joy.NumHats() ):
		print( 'hat', j, joy.GetHat(j) )

sdl.SDL_Quit()

