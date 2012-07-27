#!/usr/bin/pypy
import os,sys, time, ctypes
import libappindicator as app
app.g_type_init()
app.gtk_init()
gtk = app

print(app)
ind = app.indicator_new('myapp', 'folder', app.APP_INDICATOR_CATEGORY_APPLICATION_STATUS)
menu = gtk.Menu()
#menu = gtk.menu_new()
ind.set_menu(menu)
ind.set_status(app.APP_INDICATOR_STATUS_ACTIVE)

item = gtk.MenuItem( 'hello world' )
#item = gtk.menu_item_new_with_label('hello world')
#menu.append( item )
gtk.menu_shell_append( menu, item )

menu.show_all()

while True:
	while gtk.events_pending(): gtk.main_iteration()

print('test complete')
