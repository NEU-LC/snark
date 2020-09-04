// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine

#include "qglobal.h"
#if QT_VERSION >= 0x050000
#include <QtWidgets>
#else
#include <QtGui>
#endif
#include <QFileDialog>
#include <QFrame>
#include <QLabel>
#include <QLayout>
#include <QShortcut>
#include <comma/base/exception.h>
#include "action.h"
#include "main_window.h"

#include <fstream>

namespace snark { namespace graphics { namespace view {

MainWindow::MainWindow( const std::string& title, const std::shared_ptr<snark::graphics::view::controller>& c, const std::string& click_mode )
    : controller( c )
    , m_fileFrameVisible( controller->readers.size() > 1 )
{
    new QShortcut( QKeySequence( Qt::Key_Escape ), this, SLOT( close() ) );
    QMenu* fileMenu = menuBar()->addMenu( "File" );
    menuBar()->addMenu( fileMenu );
    fileMenu->addAction(new Action("Load Camera Config...", boost::bind(&MainWindow::load_camera_config,this)));
    fileMenu->addAction( new Action( "Save Camera Config...", boost::bind( &MainWindow::save_camera_config, this ) ) );

    m_fileFrame = new QFrame;
    m_fileFrame->setFrameStyle( QFrame::Plain | QFrame::NoFrame );
    m_fileFrame->setFixedWidth( 300 );
    m_fileFrame->setContentsMargins( 0, 0, 0, 0 );
    m_fileLayout = new QGridLayout;
    m_fileLayout->setSpacing( 0 );
    m_fileLayout->setContentsMargins( 5, 5, 5, 0 );

    QLabel* filenameLabel = new QLabel( "<b>filename</b>" );
    QLabel* visibleLabel = new QLabel( "<b>view</b>" );
    m_fileLayout->addWidget( filenameLabel, 0, 0, Qt::AlignLeft | Qt::AlignTop );
    m_fileLayout->addWidget( visibleLabel, 0, 1, Qt::AlignRight | Qt::AlignTop );
    m_fileLayout->setRowStretch( 0, 0 );
    m_fileLayout->setColumnStretch( 0, 10 ); // quick and dirty
    m_fileLayout->setColumnStretch( 1, 1 );
    m_fileFrame->setLayout( m_fileLayout );

    QFrame* frame = new QFrame;
    frame->setFrameStyle( QFrame::Plain | QFrame::NoFrame );
    frame->setContentsMargins( 0, 0, 0, 0 );
    QGridLayout* layout = new QGridLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->setSpacing( 0 );
    layout->addWidget( m_fileFrame, 0, 0 );
    viewer_t* viewer = controller_traits<snark::graphics::view::controller>::get_widget(controller);
#if QT_VERSION >= 0x050000
#if Qt3D_VERSION==1
    layout->addWidget( QWidget::createWindowContainer( viewer ), 0, 1 );
#elif Qt3D_VERSION>=2
    layout->addWidget(viewer,0,1);
#endif
#else
    layout->addWidget( viewer , 0, 1 );
#endif
    layout->setColumnStretch( 0, 0 );
    layout->setColumnStretch( 1, 1 );
    frame->setLayout( layout );
    setCentralWidget( frame );
    resize( 640, 480 );

    m_viewMenu = menuBar()->addMenu( "View" );
    ToggleAction* action = new ToggleAction( "File Panel", boost::bind( &MainWindow::toggleFileFrame, this, _1 ) );
    action->setChecked( m_fileFrameVisible );
    m_viewMenu->addAction( action );

    updateFileFrame();
    toggleFileFrame( m_fileFrameVisible );
    setWindowTitle( &title[0] );

    auto modeMenu = menuBar()->addMenu( "Modes" );
    action = new ToggleAction( "Block Mode", boost::bind( &snark::graphics::view::viewer_t::toggle_block_mode, viewer, _1 ) );
    action->setShortcut( QKeySequence( "Ctrl+B" ) );
    action->setChecked( click_mode == "block" );
    modeMenu->addAction( action );
    action = new ToggleAction( "Label Mode", boost::bind( &snark::graphics::view::viewer_t::toggle_label_mode, viewer, _1 ) );
    action->setShortcut( QKeySequence( "Ctrl+L" ) );
    action->setChecked( click_mode == "label" );
    modeMenu->addAction( action );

    viewer->setFocus();
}

CheckBox::CheckBox( boost::function< void( bool ) > f ) : m_f( f ) { connect( this, SIGNAL( toggled( bool ) ), this, SLOT( action( bool ) ) ); }

void CheckBox::action( bool checked ) { m_f( checked ); }

void MainWindow::showFileGroup( std::string const& name, bool shown )
{
    FileGroupMap::iterator it = m_userGroups.find( name );
    FileGroupMap::iterator end = m_userGroups.end();
    if( it == m_userGroups.end() )
    {
        it = m_fieldsGroups.find( name );
        end = m_fieldsGroups.end();
    }
    if( it == end ) { std::cerr << "view-points: warning: file group \"" << name << "\" not found" << std::endl; }
    for( std::size_t i = 0; i < it->second.size(); ++i ) { it->second[i]->setCheckState( shown ? Qt::Checked : Qt::Unchecked ); }
}

void MainWindow::updateFileFrame() // quick and dirty
{
    for( std::size_t i = 0; i < controller->readers.size(); ++i ) // quick and dirty: clean
    {
        for( unsigned int k = 0; k < 2; ++k )
        {
            if( m_fileLayout->itemAtPosition( i + 1, k ) == NULL ) { continue; }
            QWidget* widget = m_fileLayout->itemAtPosition( i + 1, k )->widget();
            m_fileLayout->removeWidget( widget );
            delete widget;
        }
    }
    bool sameFields = true;
    std::string fields;
    for( std::size_t i = 0; sameFields && i < controller->readers.size(); ++i )
    {
        if( i == 0 ) { fields = controller->readers[0]->options.fields; } // quick and dirty
        else { sameFields = controller->readers[i]->options.fields == fields; }
    }
    m_userGroups.clear();
    m_fieldsGroups.clear();
    for( std::size_t i = 0; i < controller->readers.size(); ++i )
    {
        static const std::size_t maxLength = 30; // arbitrary
        std::string title = controller->readers[i]->title;

        if( title.length() > maxLength )
        {
            #ifdef WIN32
            std::string leaf = comma::split( title, '\\' ).back();
            #else
            std::string leaf = comma::split( title, '/' ).back();
            #endif
            title = leaf.length() >= maxLength ? leaf : std::string( "..." ) + title.substr( title.length() - maxLength );
        }
        if( !sameFields ) { title += ": \"" + controller->readers[i]->options.fields + "\""; }
        m_fileLayout->addWidget( new QLabel( title.c_str() ), i + 1, 0, Qt::AlignLeft | Qt::AlignTop );
        CheckBox* viewBox = new CheckBox( boost::bind( &Reader::show, boost::ref( *controller->readers[i] ), _1 ) );
        viewBox->setCheckState( controller->readers[i]->show() ? Qt::Checked : Qt::Unchecked );
        connect( viewBox, SIGNAL( toggled( bool ) ), this, SLOT( update_view() ) ); // redraw when box is toggled
        viewBox->setToolTip( ( std::string( "check to make " ) + title + " visible" ).c_str() );
        m_fileLayout->addWidget( viewBox, i + 1, 1, Qt::AlignRight | Qt::AlignTop );
        m_fileLayout->setRowStretch( i + 1, i + 1 == controller->readers.size() ? 1 : 0 );
        m_fieldsGroups[ controller->readers[i]->options.fields ].push_back( viewBox );
        if ( !controller->readers[i]->groups.empty() )
        {
            auto group_list = comma::split( controller->readers[i]->groups, ',' );
            for( auto& gi : group_list ) { m_userGroups[ gi ].push_back( viewBox ); }
        }
    }
    std::size_t i = 1 + controller->readers.size();
    m_fileLayout->addWidget( new QLabel( "<b>groups</b>" ), i++, 0, Qt::AlignLeft | Qt::AlignTop );
    for( FileGroupMap::const_iterator it = m_userGroups.begin(); it != m_userGroups.end(); ++it, ++i )
    {
        m_fileLayout->addWidget( new QLabel( ( "\"" + it->first + "\"" ).c_str() ), i, 0, Qt::AlignLeft | Qt::AlignTop );
        CheckBox* viewBox = new CheckBox( boost::bind( &MainWindow::showFileGroup, this, it->first, _1 ) );
        //viewBox->setCheckState( Qt::Checked );
        viewBox->setToolTip( ( std::string( "check to make files within group \"" ) + it->first + "\" visible" ).c_str() );
        m_fileLayout->addWidget( viewBox, i, 1, Qt::AlignRight | Qt::AlignTop );
        m_fileLayout->setRowStretch( i, i + 1 == controller->readers.size() ? 1 : 0 );
    }
    m_fileLayout->addWidget( new QLabel( "<b>groups by fields</b>" ), i++, 0, Qt::AlignLeft | Qt::AlignTop );
    for( FileGroupMap::const_iterator it = m_fieldsGroups.begin(); it != m_fieldsGroups.end(); ++it, ++i )
    {
        m_fileLayout->addWidget( new QLabel( ( "\"" + it->first + "\"" ).c_str() ), i, 0, Qt::AlignLeft | Qt::AlignTop );
        CheckBox* viewBox = new CheckBox( boost::bind( &MainWindow::showFileGroup, this, it->first, _1 ) );
        //viewBox->setCheckState( Qt::Checked );
        viewBox->setToolTip( ( std::string( "check to make files with fields \"" ) + it->first + "\" visible" ).c_str() );
        m_fileLayout->addWidget( viewBox, i, 1, Qt::AlignRight | Qt::AlignTop );
        m_fileLayout->setRowStretch( i, i + 1 == controller->readers.size() + fields.size() ? 1 : 0 );
    }
}

void MainWindow::update_view()
{
    controller->update_view();
}
void MainWindow::toggleFileFrame( bool visible )
{
    m_fileFrameVisible = visible;
    if( visible ) { m_fileFrame->show(); } else { m_fileFrame->hide(); }
}

void MainWindow::closeEvent( QCloseEvent * )
{
    controller->shutdown();
}

void MainWindow::load_camera_config()
{
    QString filename=QFileDialog::getOpenFileName(this,"Load Camera Config");
//     std::cerr<<"MainWindow::load_camera_config "<<filename<<std::endl;
    if(!filename.isNull())
    {
        controller->load_camera_config(filename.toStdString());
    }
}

void MainWindow::save_camera_config()
{
    QString filename=QFileDialog::getSaveFileName(this, "Save Camera Config");
//     std::cerr<<"MainWindow::save_camera_config "<<filename<<std::endl;
    if(!filename.isNull())
    {
        std::ofstream fs(filename.toStdString());
        controller->write_camera_config(fs);
    }
}

} } } // namespace snark { namespace graphics { namespace view {
