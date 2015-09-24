// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author vsevolod vlaskine

#include "error.h"
#include "node.h"

namespace snark { namespace jai {

static const char* type_as_string( J_NODE_TYPE type )
{
    switch( type )
    {
        case J_UnknowNodeType: return "unknown";
        case J_INode: return "INode";
        case J_ICategory: return "ICategory";
        case J_IInteger: return "integer";
        case J_IEnumeration: return "enumeration";
        case J_IEnumEntry: return "IEnumEntry";
        case J_IMaskedIntReg: return "IMaskedIntReg";
        case J_IRegister: return "IRegister";
        case J_IIntReg: return "IIntReg";
        case J_IFloat: return "float";
        case J_IFloatReg: return "IFloatReg";
        case J_ISwissKnife: return "ISwissKnife";
        case J_IIntSwissKnife: return "IIntSwissKnife";
        case J_IIntKey: return "IIntKey";
        case J_ITextDesc: return "ITextDesc";
        case J_IPort: return "IPort";
        case J_IConfRom: return "IConfRom";
        case J_IAdvFeatureLock: return "IAdvFeatureLock";
        case J_ISmartFeature: return "ISmartFeature";
        case J_IStringReg: return "IStringReg";
        case J_IBoolean: return "IBoolean";
        case J_ICommand: return "ICommand";
        case J_IConverter: return "IConverter";
        case J_IIntConverter: return "IIntConverter";
        case J_IChunkPort: return "IChunkPort";
        case J_INodeMap: return "INodeMap";
        case J_INodeMapDyn: return "INodeMapDyn";
        case J_IDeviceInfo: return "IDeviceInfo";
        case J_ISelector: return "ISelector";
        case J_IPortConstruct: return "IPortConstruct";
        default: return "undefined";
    }
}

static const char* access_as_string( J_NODE_ACCESSMODE access )
{
    switch( access )
    {
        case NI: return "not implemented";
        case NA: return "not available";
        case WO: return "write-only";
        case RO: return "read-only";
        case RW: return "read-write";
        case _UndefinedAccesMode: return "undefined";
        default: return "unknown";
    }
}

static bool readable( J_NODE_ACCESSMODE access ) { return access == RO || access == RW; }

static bool writable( J_NODE_ACCESSMODE access ) { return access == WO || access == RW; }

static bool implemented( J_NODE_TYPE type ) { return type == J_IInteger || type == J_IFloat || type == J_IEnumeration; }
    
node::node( const std::string& name, const std::string& value ) : name( name ), value( value ), access( _UndefinedAccesMode ), type( J_UnknowNodeType ) {}

void node::send_to( const camera& c ) const
{
    NODE_HANDLE handle;
    validate( "getting node by name", J_Camera_GetNodeByName( c.handle(), ( int8_t * )&name[0], &handle ) );
    if( !handle ) { COMMA_THROW( comma::exception, "node \"" << name << "\" not found" ); }
    J_NODE_ACCESSMODE a;
    validate( "getting node access mode", J_Node_GetAccessMode( handle, &a ) );
    if( !jai::writable( a ) ) { COMMA_THROW( comma::exception, "node \"" << name << "\" is not writable (" << jai::access_as_string( a ) << ")" ); }
    J_NODE_TYPE t;
    validate( "getting node type", J_Node_GetType( handle, &t ) );
    if( !jai::implemented( t ) ) { COMMA_THROW( comma::exception, "node \"" << name << "\" is not implemented in snark (" << jai::type_as_string( t ) << ")" ); }
    switch( t )
    {
        case J_IInteger:
        {
            int64_t v = boost::lexical_cast< int64_t >( value );
            validate( "setting integer", J_Node_SetValueInt64( handle, true, v ) );
            break;
        }
        case J_IFloat:
        {
            double v = boost::lexical_cast< double >( value );
            validate( "setting double", J_Node_SetValueDouble( handle, true, v ) );
            break;
        }
        case J_IEnumeration:
        {
            validate( "setting enumeration as string", J_Node_SetValueString( handle, true, ( int8_t* )&value[0] ) );
            break;
        }
        default:
            return; // unimplemented
    }
}

bool node::readable() const { return jai::readable( access ); }

bool node::writable() const { return jai::writable( access ); }

bool node::implemented() const { return jai::implemented( type ); }

void node::get_from( const camera& c )
{
    if( !readable() || !implemented() ) { return; }
    NODE_HANDLE handle;
    validate( "getting node by name", J_Camera_GetNodeByName( c.handle(), ( int8_t * )&name[0], &handle ) );
    get_from( c, handle );
}

void node::get_from( const camera& c, NODE_HANDLE handle )
{
    validate( "getting node type", J_Node_GetType( handle, &type ) );
    validate( "getting node access mode", J_Node_GetAccessMode( handle, &access ) );
    if( !readable() || !implemented() ) { return; }
    switch( type )
    {
        case J_IInteger:
        {
            int64_t v;
            validate( "getting integer", J_Node_GetValueInt64( handle, false, &v ) );
            value = boost::lexical_cast< std::string >( v );
            break;
        }
        case J_IFloat:
        {
            double v;
            validate( "getting integer", J_Node_GetValueDouble( handle, false, &v ) );
            value = boost::lexical_cast< std::string >( v );
            break;
        }
        case J_IEnumeration:
        {
            boost::array< char, 1024 > buf;
            uint32_t size = buf.size();
            validate( "getting enumeration as string", J_Node_GetValueString( handle, false, ( int8_t* )&buf[0], &size ) );
            value = std::string( &buf[0] );
            break;
        }
        default:
            return; // unimplemented
    }
}

const char* node::type_as_string() const { return jai::type_as_string( type ); }

const char* node::access_as_string() const { return jai::access_as_string( access ); }

std::vector< node > nodes( const camera& c )
{
    uint32_t number_of_nodes;
    validate( "getting number of nodes", J_Camera_GetNumOfNodes( c.handle(), &number_of_nodes ) );
    std::vector< node > n( number_of_nodes );
    for( uint32_t i = 0; i < number_of_nodes; ++i )
    {
        NODE_HANDLE handle;
        validate( "getting node by index", J_Camera_GetNodeByIndex( c.handle(), i, &handle ) );
        uint32_t size = 256;
        boost::array< char, 256 > name;
        validate( "getting node name", J_Node_GetName( handle, ( int8_t* )&name[0], &size, 0 ) );
        n[i].name = std::string( &name[0] );
        n[i].get_from( c, handle );
    }
    return n;
}
    
} } // namespace snark { namespace jai {
