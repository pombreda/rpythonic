#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try:
    cppHeader = CppHeaderParser.CppHeader("ogretest4.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

print( cppHeader.namespaces )
print( cppHeader.typedefs )
