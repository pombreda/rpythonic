#!/usr/bin/pypy
#!/usr/bin/python

#http://parmanoir.com/Taming_JavascriptCore_within_and_without_WebView
#http://gxjones.blogspot.sg/2008/02/javascript-core-in-leopard-basics.html


import os, sys, time, ctypes
import webkit

a = 400
b = 20.0

ctx = webkit.JSGlobalContextCreate( None )
body = webkit.JSStringCreateWithUTF8CString( 'return %s + %s' %(a,b) )
func = webkit.JSObjectMakeFunction(
	ctx,
	None,	# name
	0,		# number of arguments
	None,	# argument names
	body,	# function body
	None,	# source url
	1,		# starting line number
	None	# exception object
)
print(func)

result = webkit.JSObjectCallAsFunction( ctx, func, None, 0, None, None )
value = webkit.JSValueToNumber( ctx, result, None )
assert value == a+b

print(result, value)
print('webkit test complete')

