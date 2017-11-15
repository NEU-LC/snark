#!/usr/bin/python

import argparse

class set_dictionary_action( argparse.Action ):
    """
auxiliary class to parse dictionaries passed on command line

expected input format:
    path=value;another/path=other

usage with argparse module:
    parser.add_argument( '--dict-of-int', help='populate a dictionary', type=str, action=set_dictionary_action, value_type=int, delimiter=';', default={} )

optional argument value_type restricts dictionary values to entries of that type, e.g., int or str
optional argument delimiter allows to deparate dictionary entries by delimiter rather then ','
"""
    def __init__(self, option_strings, dest, nargs=None, value_type=None, delimiter=',', *args, **kwargs):
        if nargs is not None: raise ValueError("nargs not allowed")
        super( set_dictionary_action, self ).__init__( option_strings, dest, *args, **kwargs )
        self.delimiter = delimiter
        self.value_type = value_type
    def __call__( self, parser, namespace, values, option_string=None ):
        di = {}
        for c in values.split( self.delimiter ):
            av = c.split( '=' )
            if len(av) != 2: raise RuntimeError( "arguments to %s shall be %s-separated <path>=<value> pairs" % ( option_string, self.delimiter ) )
            if self.value_type:
                try: di[ av[0] ] = self.value_type( av[1] )
                except: raise ValueError( "value of '%s' shall be of '%s', not '%s'" % ( av[0], self.value_type, av[1] ) )
            else:
                di[ av[0] ] = av[1]
        setattr( namespace, self.dest, di )

def remove_argparse_options( parser, options ):
    """
argparse does not provide a standard way to remove or override options
this function handles it manually
"""
    for option in options:
        for action in parser._actions:
            for option_string in vars(action)['option_strings']:
                if option_string == option:
                    parser._handle_conflict_resolve( None, [(option, action)] )
                    break

def lengths_of_strings( fields, formats, delimiter=',' ):
    """
given delimiter-separated strings of field names and formats, returns a dictionary of { 'field': string_length } entries for all the string fields
if the arguments are not strings, assume that they have been already split by the delimiter

example:
    lengths_of_strings( 'foo,bar,baz/blah', 's[2],d,s[3]' )
returns:
    { 'foo': 2, 'baz/blah': 3 }
"""
    format_types = type(formats) == str and formats.split( delimiter ) or formats
    field_names = type(fields) == str and fields.split( delimiter ) or fields
    if len( format_types ) != len( field_names ):
        raise ValueError( "fields '%s' and formats '%s' must have the same number of elements" % ( fields, formats ) )
    lengths = {}
    import re
    regex = r'^s\[(\d+)\]$'
    for k, v in zip( field_names, format_types ):
        m = re.match( regex, v )
        if m:
            lengths[k] = int(m.group(1))
    return lengths
