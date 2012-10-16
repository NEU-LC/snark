// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PROTOCOL_H_
#define SNARK_ACTUATORS_QUICKSET_ptcr_PROTOCOL_H_

#include <string>
#include <boost/noncopyable.hpp>
#include "./commands.h"
#include "./packet.h"

namespace snark { namespace quickset { namespace ptcr {

class protocol : public boost::noncopyable
{
    public:
        protocol( const std::string& name );

        ~protocol();
        
        template < typename C >
        const packet< typename C::response >* send( const C& command );

        void close();
        
    private:
        class impl;
        impl* pimpl_;
};

} } } // namespace snark { namespace quickset { namespace ptcr {

#endif // #ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PROTOCOL_H_
