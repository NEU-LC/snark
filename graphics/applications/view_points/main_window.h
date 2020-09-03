// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine

#pragma once

#include <QCheckBox>
#include <QMainWindow>

#include "controller.h"

QT_BEGIN_NAMESPACE
class QAction;
class QActionGroup;
class QFrame;
class QGridLayout;
class QMenu;
class QToolBar;
QT_END_NAMESPACE

namespace snark { namespace graphics { namespace view {

class CheckBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow( const std::string& title, const std::shared_ptr<snark::graphics::view::controller>& controller, const std::string& double_right_click_mode );

private slots:
    void update_view();
    private:
        QMenu* m_viewMenu;
        std::shared_ptr<snark::graphics::view::controller> controller;
        QFrame* m_fileFrame;
        QGridLayout* m_fileLayout;
        bool m_fileFrameVisible;
        typedef std::map< std::string, std::vector< CheckBox* > > FileGroupMap;
        FileGroupMap m_userGroups; // quick and dirty
        FileGroupMap m_fieldsGroups; // quick and dirty

        void closeEvent( QCloseEvent* event );
        void updateFileFrame();
        void toggleFileFrame( bool shown );
        void makeFileGroups();
        void showFileGroup( std::string const& name, bool shown );
        void load_camera_config();
        void save_camera_config();
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
