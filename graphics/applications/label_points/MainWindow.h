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


#ifndef SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_MAINWINDOW_H
#define SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_MAINWINDOW_H

#include <string>
#include <QMainWindow>
#include <QGridLayout>
#include <QMenu>
#include <QCheckBox>
#ifndef Q_MOC_RUN
#include <comma/base/types.h>
#endif
#include "Actions.h"
#include "IdEdit.h"
#include "Viewer.h"

namespace snark { namespace graphics { namespace view {

class CheckBox;
    
class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow( const std::string& title, snark::graphics::view::Viewer* viewer );
    
    public slots:
        void viewerInitialized();
    
    private:
        Viewer& m_viewer;
        QActionGroup m_paintToolGroup;
        IdEdit* m_idEdit;
        QMenu* m_viewMenu;
        QMenu* m_writableMenu;
        QFrame* m_fileFrame;
        QGridLayout* m_fileLayout;
        bool m_fileFrameVisible;
        typedef std::map< std::string, std::vector< std::pair< CheckBox*, CheckBox* > > > FileGroupMap;
        FileGroupMap m_fileGroups; // quick and dirty
    
        void closeEvent( QCloseEvent* event );
        void saveAs();
        void updateFileFrame();
        void newId() const;
        void toggleFileFrame( bool shown );
        void makeFileGroups();
        void showFileGroup( std::string name, bool shown );
        void setWritableFileGroup( std::string name, bool writable );
        void keyPressEvent(QKeyEvent *e);
};

class CheckBox : public QCheckBox // quick and dirty
{
    Q_OBJECT
    
    public:
        CheckBox( boost::function< void( bool ) > f );
    
    public slots:
        void action( bool checked );
    
    private:
        boost::function< void( bool ) > m_f;
};

} } } // namespace snark { namespace graphics { namespace view {

#endif
