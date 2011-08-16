#!/usr/bin/python
import os,sys, time

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
rpy = rpythonic.RPython( 'rpySDLtest', platform='linux' )
gl = rpy.rimport( 'openGL' )
sdl = rpy.rimport( 'SDL' )


@rpy.standalone
def myentrypoint():
	sdl.SDL_Init( sdl.SDL_INIT_VIDEO )
	sdl.SDL_SetVideoMode( 320, 240, 32, sdl.SDL_GL_ACCELERATED_VISUAL )#SDL_GL_DOUBLEBUFFER )
	sdl.SDL_WM_SetCaption( 'hello world', None )

	for i in range( 1000 ):
		sdl.SDL_PumpEvents()
		gl.glClearColor( 0.1, 0.8, 0.5, 1.0 )
		gl.glClear( gl.GL_COLOR_BUFFER_BIT )
		sdl.SDL_GL_SwapBuffers()
		time.sleep(0.01)

	sdl.SDL_Quit()


package = rpy.compile()
package.test()
package.save( './rpy_sdl_opengl_test' )


