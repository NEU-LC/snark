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

#ifndef SNARK_NAVIGATION_TRIMBLE_BD9XX_GSOF_H_
#define SNARK_NAVIGATION_TRIMBLE_BD9XX_GSOF_H_

#include <vector>
#include "packets/gsof.h"

namespace snark { namespace trimble { namespace bd9xx { namespace gsof {

class transmission
{
    public:
        /// constructor
        transmission();
        
        /// append packet body
        void append( const char* buf, unsigned int size );
        
        /// return true, if transmission is complete
        bool complete() const;
        
        /// return records; throw, if transmission is incomplete
        const std::vector< char >& records() const;
        
        class const_iterator
        {
            public:
                const_iterator() : current_( NULL ) {}
                
                template < typename T > const T& as() const { return *reinterpret_cast< const T* >( current_ ); }
                
                bool operator==( const const_iterator& rhs ) const { return current_ == rhs.current_; }
                
                bool operator!=( const const_iterator& rhs ) const { return !operator==( rhs ); }
                
                transmission::const_iterator& operator++(); // todo
                
                const packets::gsof::header* operator->() const { return &as< packets::gsof::header >(); }
                
                const packets::gsof::header& operator*() const { return as< packets::gsof::header >(); }
                
            private:
                friend class transmission;
                const char* current_;
        };
        
        /// return iterator pointing to first record
        const_iterator begin() const;
        
        /// return iterator pointing beyond the last record
        const_iterator end() const;
        
    private:
        typedef trimble::bd9xx::packets::gsof::transmission::header header_t_;
        boost::optional< header_t_ > header_;
        std::vector< char > records_;
};
    
} } } } // namespace snark { namespace trimble { namespace bd9xx { namespace gsof {

#endif // SNARK_NAVIGATION_TRIMBLE_BD9XX_GSOF_H_
