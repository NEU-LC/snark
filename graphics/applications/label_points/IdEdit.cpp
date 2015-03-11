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


#include <sstream>
#include <boost/lexical_cast.hpp>
#include <QHBoxLayout>
#include "IdEdit.h"
#include "Tools.h"
#include "Viewer.h"

namespace snark { namespace graphics { namespace View {

IdEdit::IdEdit( Viewer* viewer )
    : m_viewer( viewer )
    , m_edit( new QLineEdit )
    , m_size( new QLabel )
{
    QPalette p = palette();
    p.setColor( QPalette::Normal, QPalette::Text, QColor( 255, 255, 255 ) );
    p.setColor( QPalette::Normal, QPalette::Base, QColor( 0, 0, 0 ) );
    m_edit->setMaxLength( 10 );
    m_edit->setPalette( p );
    QSizePolicy policy;
    policy.setHorizontalPolicy( QSizePolicy::Minimum );
    m_edit->setSizePolicy( policy );
    connect( m_edit, SIGNAL( returnPressed() ), this, SLOT( handleReturnPressed() ) );
    connect( this, SIGNAL( valueChanged( comma::uint32 ) ), this, SLOT( setValue( comma::uint32 ) ) );
    QLabel* label = new QLabel( "id:" );
    label->setMargin( 5 );
    m_size->setMargin( 5 );
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget( label );
    layout->addWidget( m_edit );
    layout->addWidget( m_size );
    setLayout( layout );
    setToolTip( "numeric id, e.g. 123<br>\"min\" for min id<br>\"max\" for max id<br>\"new\" (Ctrl-N) for max id + 1" );
}

void IdEdit::handleReturnPressed()
{
    comma::uint32 id = 0;
    std::string s = m_edit->text().toStdString();
    if( s == "" ) { return; }
    if( s == "new" || s == "min" || s == "max" )
    {
        boost::optional< comma::uint32 > minId; // quick and dirty, max is a macro on windows
        boost::optional< comma::uint32 > maxId;
        for( std::size_t i = 0; i < m_viewer->datasets().size(); ++i )
        {
            if( !m_viewer->datasets()[i]->visible() || m_viewer->datasets()[i]->partitions().empty() ) { continue; }
            if( !minId || m_viewer->datasets()[i]->partitions().begin()->first < *minId ) { minId = m_viewer->datasets()[i]->partitions().begin()->first; }
            if( !maxId || m_viewer->datasets()[i]->partitions().rbegin()->first > *maxId ) { maxId = m_viewer->datasets()[i]->partitions().rbegin()->first; }
        }
        if( s == "new" ) { id = maxId ? *maxId + 1 : 0; }
        else if( s == "min" ) { id = minId ? *minId : 0; }
        else if( s == "max" ) { id = maxId ? *maxId : 0; }
    }
    else
    {
        try { id = boost::lexical_cast< comma::uint32 >( m_edit->text().toStdString() ); }
        catch( ... ) { m_edit->setText( m_id ? boost::lexical_cast< std::string >( *m_id ).c_str() : "" ); return; }
    }
    //if( m_id && *m_id == id ) { return; }
    emit valueChanged( id );
}

void IdEdit::setValue( comma::uint32 id )
{
    m_id = id;
    QPalette p = palette();
    QColor4ub color = Tools::colorFromId( id );
    QColor background;
    QColor foreground = color.red() + color.green() + color.blue() > 640 ? QColor( 0, 0, 0 ) : QColor( 255, 255, 255 );
    background.setRgb( color.red(), color.green(), color.blue() );
    p.setColor( QPalette::Normal, QPalette::Text, foreground );
    p.setColor( QPalette::Normal, QPalette::Base, background );
    m_edit->setText( boost::lexical_cast< std::string >( id ).c_str() );
    m_edit->setPalette( p );
    comma::uint32 size = 0;
    std::size_t num = 0;
    for( std::size_t i = 0; i < m_viewer->datasets().size(); ++i )
    {
        if( !m_viewer->datasets()[i]->visible() ) { continue; }
        Dataset::Partitions::const_iterator it = m_viewer->datasets()[i]->partitions().find( id );
        if( it != m_viewer->datasets()[i]->partitions().end() )
        {
            std::cerr << "label-points: id " << id << ": " << it->second.size() << ( it->second.size() == 1 ? " point" : " points" ) << " in " << m_viewer->datasets()[i]->filename() << std::endl;
            size += it->second.size();
        }
        ++num;
    }
    std::ostringstream oss;
    oss << size << ( size == 1 ? " point" : " points" );
    if( size > 0 && m_viewer->datasets().size() > 1 ) { oss << " in " << num << " visible file" << ( num == 1 ? "" : "s" ); }
    m_size->setText( oss.str().c_str() );
}

} } } // namespace snark { namespace graphics { namespace View {
