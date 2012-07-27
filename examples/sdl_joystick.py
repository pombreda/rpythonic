#!/usr/bin/python
# updated nov 2011
import os,sys, time
import SDL as sdl

sdl.Init( sdl.SDL_INIT_JOYSTICK )

n = sdl.NumJoysticks()
assert n

joy = sdl.JoystickOpen(0)
print(joy, dir(joy))

print( 'running loop' )
for i in range( 2800 ):
	time.sleep(0.01)
	sdl.JoystickUpdate()
	for j in range( joy.NumAxes() ):
		print( 'axis', j, joy.GetAxis(j) / 32767.0 )	# -32768 to 32767
	for j in range( joy.NumButtons() ):
		print( 'button', j, joy.GetButton(j) )
	for j in range( joy.NumHats() ):
		print( 'hat', j, joy.GetHat(j) )

sdl.Quit()

