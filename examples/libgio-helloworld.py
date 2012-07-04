#!/usr/bin/pypy
# this also works in python3

import os, sys, time, ctypes
import libgio as gio
print(gio)
gio.g_type_init()

file = gio.file_new_for_path( '/tmp' )
info = gio.file_query_info(file, "standard::*", gio.G_FILE_QUERY_INFO_NONE, None, None)
print(info)

print( info.get_name() )
print( info.get_size() )
print( info.get_icon() )
print( info.get_content_type() )
#print( info.get_attribute_string() )
#print( info.get_attribute_stringv() )

