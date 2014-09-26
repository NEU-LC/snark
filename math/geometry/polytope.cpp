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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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
#include <dsdp/dsdp5.h>
#include <boost/shared_ptr.hpp>
#include "./polytope.h"

static Eigen::VectorXd symmat2vec(const Eigen::MatrixXd& A)
{
    int n=A.rows();
    Eigen::VectorXd v(n*(n+1)/2);

    for(int i=0; i<n; i++)
    {
        for(int j=0; j<=i; j++)
        {
            v(i*(i+1)/2+j)=A(i,j);
        }
    }
    return v;
}

namespace snark{ namespace geometry {

/// This function uses the SDP solver provided by the DSDP library
/// It reformulates the problem of finding whether a point is inside a convex polygon as an SDP (Semi-Definite Program)
/// Problem:
/// min ||y-x||^2 s.t. Ay<=b
/// SDP Formulation:
/// [I y-x; (y-xy)' t] is psd
/// diag(b)-y1*A1-...-yn*An is psd
bool convex_polytope::has(const Eigen::VectorXd &x, double tolerance)
{
    int dimx=x.size();
    if(dimx!=normals.cols())
    {
        COMMA_THROW( comma::exception, "point should be of same dimension as polytope, polytope dimension: "<<normals.cols()<<" point dimension: "<<dimx );
    }

    const Eigen::MatrixXd& A=normals;
    const Eigen::VectorXd& b=offsets;

    int nofaces=A.rows();

    tolerance=tolerance*tolerance; //square tolerance

    DSDP solver;
    DSDPCreate(dimx+1,&solver);


    for(int cntr=1; cntr<=dimx; cntr++)
    {
        DSDPSetDualObjective(solver,cntr,0);
    }
    DSDPSetDualObjective(solver,dimx+1,-1); // objective is inverted

    SDPCone cone;
    DSDPCreateSDPCone(solver,2,&cone);

    Eigen::MatrixXd A0(nofaces,nofaces);
    Eigen::MatrixXd A1(dimx+1,dimx+1);

    std::vector< boost::shared_ptr<Eigen::VectorXd> > a0(dimx+2);
    std::vector< boost::shared_ptr<Eigen::VectorXd> > a1(dimx+2);

    //constant terms
    A0.setZero();
    A1.setZero();
    A0=b.asDiagonal();A0=-A0;
    A1.block(0,0,dimx,dimx)=Eigen::MatrixXd::Identity(dimx,dimx);
    A1.block(0,dimx,dimx,1)=-x;
    A1.block(dimx,0,1,dimx)=-x.transpose();
    a0[0].reset(new Eigen::VectorXd(symmat2vec(A0)));
    a1[0].reset(new Eigen::VectorXd(symmat2vec(A1)));
    SDPConeSetADenseVecMat(cone,0,0,dimx+1,1.0,a0[0]->data(),a0[0]->size());
    SDPConeSetADenseVecMat(cone,1,0,nofaces,1.0,a1[0]->data(),a1[0]->size());


    //yi's, A's are negated
    for(int cntr=0; cntr<dimx; cntr++)
    {
        A0.setZero();
        A1.setZero();
        A0=A.col(cntr).asDiagonal();
        A1(dimx,cntr)=1;
        A1(cntr,dimx)=1;
        a0[cntr+1].reset(new Eigen::VectorXd(symmat2vec(-A0)));
        a1[cntr+1].reset(new Eigen::VectorXd(symmat2vec(-A1)));
        SDPConeSetADenseVecMat(cone,0,cntr+1,dimx+1,1.0,a0[cntr+1]->data(),a0[cntr+1]->size());
        SDPConeSetADenseVecMat(cone,1,cntr+1,nofaces,1.0,a1[cntr+1]->data(),a1[cntr+1]->size());
    }

    //t
    A0.setZero();
    A1.setZero();
    A1(dimx,dimx)=1;
    a0[dimx+1].reset(new Eigen::VectorXd(symmat2vec(-A0)));
    a1[dimx+1].reset(new Eigen::VectorXd(symmat2vec(-A1)));
    SDPConeSetADenseVecMat(cone,0,dimx+1,dimx+1,1.0,a0[dimx+1]->data(),a0[dimx+1]->size());
    SDPConeSetADenseVecMat(cone,1,dimx+1,nofaces,1.0,a1[dimx+1]->data(),a1[dimx+1]->size());

//    DSDPSetY0(solver,1,1);
//    DSDPSetY0(solver,2,1);
//    DSDPSetY0(solver,3,1);
//    DSDPSetY0(solver,4,10);
//    SDPConeViewDataMatrix(cone,1,1);
//    std::cout<<std::endl;
//    SDPConeViewDataMatrix(cone,1,2);
//    std::cout<<std::endl;
//    SDPConeViewDataMatrix(cone,1,3);
//    std::cout<<std::endl;
//    SDPConeViewDataMatrix(cone,1,4);
//    std::cout<<std::endl;

    DSDPSetup(solver);
//    DSDPSetStandardMonitor(solver,1);
    DSDPSolve(solver);

//    DSDPTerminationReason reason;
//    DSDPStopReason(solver,&reason);
//    std::cout<<"reason: "<<reason<<std::endl;

    Eigen::VectorXd y(dimx+1);

    DSDPGetY(solver,y.data(),y.size());

//    Eigen::VectorXd smat((dimx+1)*(dimx+2)/2);
//    SDPConeComputeS(cone,1,1, y.data(), y.size(), 0, dimx+1, smat.data(), smat.size());
//    Eigen::MatrixXd S=vec2symmat(smat);
//    std::cout<<S<<std::endl;
//    std::cout<<y<<std::endl;

    double t=y(dimx);
    return(t<tolerance);
}
}} // namespace snark{ { namespace geometry {
