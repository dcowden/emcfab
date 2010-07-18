#slicer settings
settings = {
	#setting 1
	'setting1' : True,

	#setting 2
	'setting2' : 0,
	
	#setting 3
	'setting3' : 'test value',
	
	#setting 4
	'setting4' : [
		'val1','val2','val3'
	]
}
"var1 is a something"
var1 = True;

"var2 is a something else"
var2 = False;

ConfigItem:
	name
	description
	type
	required
	defaultValue

	toDict
	fromDict
	value
	
ConfigGroup
	name
	description
	toDict
	fromDict
	items
	

