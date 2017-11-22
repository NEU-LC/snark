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
//    documentation and/or other materials provided with the distribution.ll
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

#include "laser_map.h"

namespace snark { namespace velodyne { namespace puck {

/*
 * reference:
 * 
 * VLP-16 User's Manual
 * Appendix F: Mechanical Drawing
 * 63-9243 Rev B User Manual and Programming Guide,VLP-16.pdf
 */
laser_map::laser_map()
{
    map[0]=0;
    map[2]=1;
    map[4]=2;
    map[6]=3;
    map[8]=4;
    map[10]=5;
    map[12]=6;
    map[14]=7;
    map[1]=8;
    map[3]=9;
    map[5]=10;
    map[7]=11;
    map[9]=12;
    map[11]=13;
    map[13]=14;
    map[15]=15;
    
    reverse_map[0]=0;
    reverse_map[1]=2;
    reverse_map[2]=4;
    reverse_map[3]=6;
    reverse_map[4]=8;
    reverse_map[5]=10;
    reverse_map[6]=12;
    reverse_map[7]=14;
    reverse_map[8]=1;
    reverse_map[9]=3;
    reverse_map[10]=5;
    reverse_map[11]=7;
    reverse_map[12]=9;
    reverse_map[13]=11;
    reverse_map[14]=13;
    reverse_map[15]=15;
}
 
unsigned int laser_map::id_to_index( unsigned int id ) const
{
//     if(id%2)
//     {
//         return (id/2)+8;
//     }
//     else
//     {
//         return id/2;
//     }
    return map[id];
}

unsigned int laser_map::index_to_id( unsigned int index ) const
{
//     if(index>7)
//     {
//         return (index-8)*2+1;
//     }
//     else
//     {
//         return index*2;
//     }
    return reverse_map[index];
}
    
} } } // namespace snark { namespace velodyne { namespace puck {
    
