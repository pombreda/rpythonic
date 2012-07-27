#!/usr/bin/python
# july 21, 2011
import os,sys, time, ctypes

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
ogre = rpythonic.module( 'Ogre' )
assert ogre


# defaults: pluginFileName = "plugins.cfg" , configFileName = "ogre.cfg" , logFileName = "Ogre.log"
root = ogre.Root('data/plugins.cfg', 'data/ogre.cfg', 'data/ogre.log')
root.showConfigDialog()
win = root.initialise( True, 'mywindow' )
win.setActive(True)
man = root.createSceneManager('OctreeSceneManager', 'mymanager')
cam = man.createCamera( 'mycamera' )
cam.setPosition( 0, 0, 100 )
cam.lookAt( 0, 0, 0 )
view = win.addViewport( cam )
bg = view.getBackgroundColour()
bg.setHSB( 0.5, 0.5, 0.9 )	#Set a colour value from Hue, Saturation and Brightness.
view.setBackgroundColour( bg )

rgman = ogre.ResourceGroupManager.getSingleton()
rgman.addResourceLocation( './data', 'FileSystem', 'General' )	# can be FileSystem or Zip
rgman.initialiseAllResourceGroups()

rootnode = man.getRootSceneNode()
e = man.createEntity( 'mycube', 'Cube.mesh', groupName='General' )
node = rootnode.createChildSceneNode( ogre.Vector3(), ogre.Quaternion() )
node.attachObject( e )

for i in range(200):
	time.sleep(0.01)
	root.renderOneFrame()
	win.swapBuffers()

print('ogre test done')

