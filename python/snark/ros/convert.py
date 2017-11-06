#!/usr/bin/python

import numpy
import rosbag, rospy, rostopic
import comma

try:
    import rospy_message_converter
except ImportError:
    msg = """
cannot import rospy_message_converter module; usually you can install it as
    sudo apt-get install ros-kinetic-rospy-message-converter
(use your ROS distro name in place of kinetic). If the module is not available
in your package manager, build and install the module manually.
"""
    raise ImportError( msg )

def ros_message_to_csv_record( message, lengths={} ):
    """
    Takes a ROS message and returns a comma.csv.struct (Python type) representing this message
    and a lambda function converting the message into an instance of the new comma.csv.struct
    Optional second argument allows to specify explicitly the length of variable-length values,
    such as strings. By default, take the lengths from the message itself.
"""
    record_t, record_ctor = _ros_message_to_csv_record( message, lengths=lengths, prefix='' )
    for k, v in lengths.items():
        try:
            pos = record_t.fields.index( k )
            if record_t.types[ pos ][0] != 'S':
                raise RuntimeError( "length %d specified for field '%s' that is not a string" % ( v, k ) )
        except ValueError:
            raise RuntimeError( "length %d specified for unknown field '%s'" % ( v, k ) )
    return ( record_t, record_ctor )

def _ros_message_to_csv_record( message, lengths={}, prefix='' ):
    """
    Private implementation of ros_message_to_csv_record. Called recursively.
"""
    from rospy_message_converter import message_converter as mc

    full_path = lambda name: prefix and prefix + "/" + name or name

    message_fields = mc._get_message_fields(message)
    fields = []
    types = []
    ctors = []
    # see Python programming FAQ why-do-lambdas-defined-in-a-loop-with-different-values-all-return-the-same-result
    # for the explanation of all the lambda signatures (and some function signatures in case of time)
    for field_name, field_type in message_fields:
        fields.append( field_name )
        if field_type in mc.ros_binary_types:
            ctor = lambda msg, field_name=field_name, field_type=field_type: mc._convert_to_ros_binary( field_type, getattr( msg, field_name ) )
            current_path = full_path( field_name )
            try:
                l = lengths[ current_path ]
            except KeyError:
                l = len( ctor( message ) )
            element_t = "S%d" % l
        elif field_type in mc.ros_primitive_types:
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )
            if field_type == 'string':
                current_path = full_path( field_name )
                try:
                    l = lengths[ current_path ]
                except KeyError:
                    l = len( ctor( message ) )
                element_t = "S%d" % l
            else:
                element_t = field_type
        elif field_type == 'time':
            def ctor( msg, field_name=field_name ):
                ts = getattr( msg, field_name )
                return numpy.datetime64( datetime.datetime.fromtimestamp( ts.secs + 1.0e-9 * ts.nsecs ) )
            element_t = 'datetime64[us]'
        elif field_type == 'duration':
            def ctor( msg, field_name=field_name ):
                ts = getattr( msg, field_name )
                return numpy.timedelta64( ts.secs, 's' ) + numpy.timedelta64( ts.nsecs, 'ns' )
            element_t = 'timedelta64[us]'
        elif mc._is_field_type_an_array(field_type):
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )
            m = mc.list_brackets.search( field_type )
            element_t = ( field_type[:m.start()], ( int(m.group()[1:-1]), ) )
        else:
            element_t, element_ctor = _ros_message_to_csv_record( getattr( message, field_name ), lengths=lengths, prefix=full_path( field_name ) )
            ctor = lambda msg, field_name=field_name, element_ctor=element_ctor: element_ctor( getattr( msg, field_name ) )
        ctors.append( ctor )
        types.append( element_t )

    new_t = comma.csv.struct( ','.join( fields ), *types )
    return ( new_t, lambda msg, new_t=new_t: numpy.array( [ tuple( [ c(msg) for c in ctors ] ) ], dtype = new_t ) )
