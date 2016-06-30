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


/// @author Cedric Wohlleber

#include <Qt3D/qglbuilder.h>
#include <Qt3D/qglcylinder.h>
#include "coordinates.h"

namespace snark { namespace graphics { namespace qt3d {

coordinates::coordinates( float size, float thickness ):
    m_node( new QGLSceneNode() )
{
    QGLBuilder builder;
    builder << QGLCylinder( thickness, thickness, size, 12 );
    QGLSceneNode* cylinder = builder.finalizedSceneNode();
    cylinder->setPosition( QVector3D( 0, 0, 0.5 * size ) );
    cylinder->setEffect( QGL::LitMaterial );

    QGLBuilder arrowBuilder;
    arrowBuilder << QGLCylinder( 0.01, 2 * thickness, size / 3, 12 );
    QGLSceneNode* arrow = arrowBuilder.finalizedSceneNode();
    arrow->setPosition( QVector3D( 0, 0, size ) );
    arrow->setEffect( QGL::LitMaterial );
    
    
    QGLSceneNode* x = new QGLSceneNode( m_node );
    x->addNode(cylinder);
    QMatrix4x4 matrix;
    QQuaternion q = QQuaternion::fromAxisAndAngle(0.0f, 1.0f, 0.0f, 90.0f);
    matrix.rotate( q );
    x->setLocalTransform( matrix );
    QGLSceneNode* xArrow = new QGLSceneNode( m_node );
    xArrow->addNode( arrow );
    xArrow->setLocalTransform( matrix );
    QGLMaterial* xMaterial = new QGLMaterial;
    xMaterial->setDiffuseColor( QColor( 255, 0, 0, 128 ) );
    x->setMaterial( xMaterial );
    xArrow->setMaterial( xMaterial );
    
    QGLSceneNode* y = new QGLSceneNode( m_node );
    y->addNode(cylinder);
    q = QQuaternion::fromAxisAndAngle(1.0f, 0.0f, 0.0f, -90.0f);
    matrix.rotate( q );
    y->setLocalTransform( matrix );
    QGLSceneNode* yArrow = new QGLSceneNode( m_node );
    yArrow->addNode( arrow );
    yArrow->setLocalTransform( matrix );
    QGLMaterial* yMaterial = new QGLMaterial;
    yMaterial->setDiffuseColor(Qt::green);
    y->setMaterial( yMaterial );
    QGLMaterial* yArrowMaterial = new QGLMaterial;
    yArrowMaterial->setDiffuseColor( QColor( 0, 255, 0, 128 ) );
    yArrow->setMaterial( yArrowMaterial );
    
    QGLSceneNode* z = new QGLSceneNode( m_node );
    z->addNode(cylinder);
    QGLSceneNode* zArrow = new QGLSceneNode( m_node );
    zArrow->addNode( arrow );
    QGLMaterial* zMaterial = new QGLMaterial;
    zMaterial->setDiffuseColor(Qt::blue);
    z->setMaterial( zMaterial );
    QGLMaterial* zArrowMaterial = new QGLMaterial;
    zArrowMaterial->setDiffuseColor( QColor( 0, 0, 255, 128 ) );
    zArrow->setMaterial( zArrowMaterial );
}

QGLSceneNode* coordinates::node() const
{
    return m_node;
}


} } } // namespace snark { namespace graphics { namespace qt3d {

