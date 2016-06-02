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

#pragma once

#include <vector>
#include <Eigen/Core>
#include <comma/base/exception.h>
#include <cuda_runtime.h>

cudaError_t snark_cuda_squared_norms( double x, double y, double z, const double *points, double *square_norms, unsigned int size );

namespace snark { namespace cuda {
    
template < unsigned int Factor >
struct buffer
{
    std::vector< double > in;
    std::vector< double > out;
    double* cuda_in;
    double* cuda_out;

    buffer() : cuda_in( NULL ), cuda_out( NULL ) {}
    ~buffer() { deallocate(); }
    void deallocate()
    { 
        if( cuda_in ) { cudaFree( cuda_in ); cuda_in = NULL; }
        if( cuda_out ) { cudaFree( cuda_out ); cuda_out = NULL; }
    }
    void copy_once()
    {
        if( cuda_in ) { return; }
        cudaError_t err = cudaMalloc( ( void ** )&cuda_in, in.size() * sizeof( double ) );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: failed to allocate " << in.size() << " of doubles for input values; " << cudaGetErrorString( err ) ); }
        err = cudaMalloc( ( void ** )&cuda_out, in.size() / Factor * sizeof( double ) );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: failed to allocate " << ( in.size() / Factor ) << " of doubles for output values; " << cudaGetErrorString( err ) ); }
        err = cudaMemcpy( cuda_in, &in[0], in.size() * sizeof( double ), cudaMemcpyHostToDevice );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: copy failed; " << cudaGetErrorString( err ) ); }
    }
};

void squared_norms( const Eigen::Vector3d& v, buffer< 3 >& b, bool deallocate = true );

} } // namespace snark { namespace cuda {
