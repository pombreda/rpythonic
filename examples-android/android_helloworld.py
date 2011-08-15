import os, sys, time
sys.path.append('..')
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
rpythonic.set_android_sdk_root( '../../android-sdk-linux_x86/' )
rpythonic.set_android_ndk_root( '../../android-ndk-r5b/' )
rpy = rpythonic.RPython('android')

@rpy.standalone
def main():
	rpy.setup_display()


package = rpy.compile()
print( package.apk )



