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

#include <assert.h>
#include "points_join_cuda.h"

__global__ void snark_cuda_square_norms_impl( double x, double y, double z, const double *points, double *square_norms, unsigned int size )
{
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if( i >= size ) { return; }
    unsigned int k = i * 3;
    x -= points[k];
    y -= points[k+1];
    z -= points[k+2];
    square_norms[i] = x * x + y * y + z * z;
}

cudaError_t snark_cuda_square_norms( double x, double y, double z, const double *points, double *square_norms, unsigned int size )
{
    int threads = 256; // quick and dirty
    int blocks = ( size - 1 ) / threads + 1;
    snark_cuda_square_norms_impl<<<blocks, threads>>>( x, y, z, points, square_norms, size );
    return cudaGetLastError();
}

namespace snark { namespace cuda {

void square_norms( const Eigen::Vector3d& v, buffer< 3 >& b, bool deallocate )
{
    b.copy_once();
    unsigned int size = b.in.size() / 3;
    cudaError_t err = snark_cuda_square_norms( v.x(), v.y(), v.z(), b.cuda_in, b.cuda_out, size );
    if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: square norm calculation failed; " << cudaGetErrorString( err ) ); }
    b.out.resize( size );
    err = cudaMemcpy( b.cuda_out, &b.out[0], size * sizeof( double ), cudaMemcpyDeviceToHost );
    if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: copy failed; " << cudaGetErrorString( err ) ); }
    if( deallocate ) { b.deallocate(); }
}

} } // namespace snark { namespace cuda {
