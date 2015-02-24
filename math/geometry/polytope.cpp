// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2014 The University of Sydney
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


#include <cmath>
#include <Eigen/Eigen>
#include <comma/base/exception.h>
#include "polytope.h"

namespace snark{ namespace geometry {

const Eigen::MatrixXd& convex_polytope::normals() const { return normals_; }
    
const Eigen::VectorXd& convex_polytope::offsets() const { return offsets_; }

convex_polytope::convex_polytope( const std::vector< Eigen::VectorXd >& normals, const std::vector< double >& offsets )
{
    if(!normals.size() || normals[0].size()) { COMMA_THROW( comma::exception, "normals size cannot be zero" ); }
    if(normals.size()!=offsets.size()) { COMMA_THROW( comma::exception, "normals and offsets should be of same size, got "<< normals.size()<<" and "<<offsets.size() ); }
    normals_.resize(normals.size(),normals[0].size());
    offsets_.resize(offsets.size());
    for(unsigned int i=0; i<normals.size(); ++i )
    {
        normals_.row(i)=normals[i].transpose();
        offsets_(i)=offsets[i];
    }
}

convex_polytope::convex_polytope( const Eigen::MatrixXd& normals, const Eigen::VectorXd& offsets )
    : normals_( normals )
    , offsets_( offsets )
{
    if(!normals.size()) { COMMA_THROW( comma::exception, "normals size cannot be zero" ); }
    if(normals.rows()!=offsets.rows()) { COMMA_THROW( comma::exception, "normals and offsets should be of same size, got "<< normals.rows()<<" and "<<offsets.rows() ); }
}

convex_polytope::convex_polytope( const Eigen::MatrixXd& planes )
{
    if(!planes.rows() || planes.cols() < 2) { COMMA_THROW( comma::exception, "planes rows cannot be zero and cols must be at least two, got "<<planes.rows()<<" and "<<planes.cols() ); }
    normals_=planes.leftCols( static_cast< unsigned int >( planes.cols() - 1 ) );
    offsets_=planes.rightCols(1);
}

//static Eigen::VectorXd symmat2vec(const Eigen::MatrixXd& A)
//{
//    unsigned int n = A.rows();
//    Eigen::VectorXd v( n * ( n + 1 ) / 2 );
//    unsigned int m = 0;
//    for( unsigned int i=0; i<n; ++i )
//    {
//        for( unsigned int j = 0; j <= i; v(m) = A(i,j), ++j, ++m );
//    }
//    return v;
//}

//static Eigen::VectorXd symmat2vec(const Eigen::MatrixXd& A)
//{
//    int n=A.rows();
//    Eigen::VectorXd v(n*(n+1)/2);

//    for(int i=0; i<n; i++)
//    {
//        for(int j=0; j<=i; j++)
//        {
//            v(i*(i+1)/2+j)=A(i,j);
//        }
//    }
//    return v;
//}
// This function uses the SDP solver provided by the DSDP library
// It reformulates the problem of finding whether a point is inside a convex polygon as an SDP (Semi-Definite Program)
// Problem:
// min ||y-x||^2 s.t. Ay<=b
// SDP Formulation:
// [I y-x; (y-xy)' t] is psd
// diag(b)-y1*A1-...-yn*An is psd

/// This function checks whether a point is inside a convex polytope: Ay<=b
bool convex_polytope::has(const Eigen::VectorXd &x)
{
    int dimx=x.size();
    if(dimx!=normals_.cols())
    {
        COMMA_THROW( comma::exception, "point should be of same dimension as polytope, polytope dimension: "<<normals_.cols()<<" point dimension: "<<dimx );
    }

    const Eigen::MatrixXd& A=normals_;
    const Eigen::VectorXd& b=offsets_;

    Eigen::VectorXd d=A*x-b;

    return((d.array()>=0).all());

//    int nofaces=A.rows();

//    tolerance=tolerance*tolerance; //square tolerance

//    DSDP solver;
//    DSDPCreate(dimx+1,&solver);


//    for(int cntr=1; cntr<=dimx; cntr++)
//    {
//        DSDPSetDualObjective(solver,cntr,0);
//    }
//    DSDPSetDualObjective(solver,dimx+1,-1); // objective is inverted

//    SDPCone cone;
//    DSDPCreateSDPCone(solver,2,&cone);

//    Eigen::MatrixXd A0(nofaces,nofaces);
//    Eigen::MatrixXd A1(dimx+1,dimx+1);

//    std::vector< Eigen::VectorXd > a0(dimx+2);
//    std::vector< Eigen::VectorXd > a1(dimx+2);

//    //constant terms
//    A0.setZero();
//    A1.setZero();
//    A0=b.asDiagonal();A0=-A0;
//    A1.block(0,0,dimx,dimx)=Eigen::MatrixXd::Identity(dimx,dimx);
//    A1.block(0,dimx,dimx,1)=-x;
//    A1.block(dimx,0,1,dimx)=-x.transpose();
//    a0[0] = Eigen::VectorXd(symmat2vec(A0));
//    a1[0] = Eigen::VectorXd(symmat2vec(A1));
//    SDPConeSetADenseVecMat(cone,0,0,nofaces,1.0,a0[0].data(),a0[0].size());
//    SDPConeSetADenseVecMat(cone,1,0,dimx+1,1.0,a1[0].data(),a1[0].size());


//    //yi's, A's are negated
//    for(int cntr=0; cntr<dimx; cntr++)
//    {
//        A0.setZero();
//        A1.setZero();
//        A0=A.col(cntr).asDiagonal();
//        A1(dimx,cntr)=1;
//        A1(cntr,dimx)=1;
//        a0[cntr+1] = Eigen::VectorXd(symmat2vec(-A0));
//        a1[cntr+1] = Eigen::VectorXd(symmat2vec(-A1));
//        SDPConeSetADenseVecMat(cone,0,cntr+1,nofaces,1.0,a0[cntr+1].data(),a0[cntr+1].size());
//        SDPConeSetADenseVecMat(cone,1,cntr+1,dimx+1,1.0,a1[cntr+1].data(),a1[cntr+1].size());
//    }

//    //t
//    A0.setZero();
//    A1.setZero();
//    A1(dimx,dimx)=1;
//    a0[dimx+1] = Eigen::VectorXd(symmat2vec(-A0));
//    a1[dimx+1] = Eigen::VectorXd(symmat2vec(-A1));
//    SDPConeSetADenseVecMat(cone,0,dimx+1,nofaces,1.0,a0[dimx+1].data(),a0[dimx+1].size());
//    SDPConeSetADenseVecMat(cone,1,dimx+1,dimx+1,1.0,a1[dimx+1].data(),a1[dimx+1].size());

// //    DSDPSetY0(solver,1,1);
// //    DSDPSetY0(solver,2,1);
// //    DSDPSetY0(solver,3,1);
// //    DSDPSetY0(solver,4,10);
// //    SDPConeViewDataMatrix(cone,1,1);
// //    std::cout<<std::endl;
// //    SDPConeViewDataMatrix(cone,1,2);
// //    std::cout<<std::endl;
// //    SDPConeViewDataMatrix(cone,1,3);
// //    std::cout<<std::endl;
// //    SDPConeViewDataMatrix(cone,1,4);
// //    std::cout<<std::endl;

//    DSDPSetup(solver);
//    DSDPSetGapTolerance(solver,1e-10);
// //    DSDPSetStandardMonitor(solver,1);
//    DSDPSolve(solver);

// //    DSDPTerminationReason reason;
// //    DSDPStopReason(solver,&reason);
// //    std::cout<<"reason: "<<reason<<std::endl;

//    Eigen::VectorXd y(dimx+1);

//    DSDPGetY(solver,y.data(),y.size());

// //    Eigen::VectorXd smat((dimx+1)*(dimx+2)/2);
// //    SDPConeComputeS(cone,1,1, y.data(), y.size(), 0, dimx+1, smat.data(), smat.size());
// //    Eigen::MatrixXd S=vec2symmat(smat);
// //    std::cout<<S<<std::endl;
// //    std::cout<<y<<std::endl;

//    double t=y(dimx);
//    return(t<tolerance);
}

}} // namespace snark{ { namespace geometry {
