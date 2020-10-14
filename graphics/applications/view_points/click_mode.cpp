// Copyright (c) 2020 Vsevolod Vlaskine

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "click_mode.h"

namespace snark { namespace graphics { namespace view {

click_mode::double_right_click_t::double_right_click_t( const std::string& options ) // todo? quick and dirty; implement using traits and visiting
    : block_( 0 )
    , blocks_index_( 0 )
    , labels_index_( 0 )
{
    const auto& v = comma::split( options, ';', true );
    if( v.empty() ) { COMMA_THROW( comma::exception, "got empty double click options" ); }
    if( v[0] == "none" ) { mode_ = modes::none; }
    else if( v[0] == "block" ) { mode_ = modes::block; }
    else if( v[0] == "label" ) { mode_ = modes::label; }
    for( unsigned int i = 1; i < v.size(); ++i )
    {
        const auto& w = comma::split( v[i], '=' );
        if( w[0].empty() || w.size() != 2 ) { COMMA_THROW( comma::exception, "got invalid double click options: '" << options << "'" ); }
        if( w[0] == "blocks" )
        {
            blocks_ = comma::split_as< comma::int32 >( w[1], ',' );
            block_ = blocks_[0];
        }
        else if( w[0] == "labels" )
        {
            labels_ = comma::split( w[1], ',' );
            label_ = labels_[0];
        }
    }
}
    
std::string click_mode::double_right_click_t::to_output_string() const
{
    std::ostringstream oss; // quick and dirty, watch performance
    switch( mode_ )
    {
        case modes::block: oss << ',' << block_; break;
        case modes::label: oss << ",\"" << label_ << "\""; break;
        case modes::none: break;
    }
    return oss.str();
}

std::string click_mode::double_right_click_t::to_info_string() const
{
    std::ostringstream oss; // quick and dirty, watch performance
    switch( mode_ )
    {
        case modes::block: oss << "block: " << block_; break;
        case modes::label: oss << "label: \"" << label_ << "\""; break;
        case modes::none: break;
    }
    return oss.str();
}

void click_mode::double_right_click_t::toggle( modes::values m, bool flag )
{
    if( flag ) { mode_ = m; }
    else if( mode_ == m ) { mode_ = modes::none; }
}

void click_mode::double_right_click_t::on_key_press( QKeyEvent* event )
{
    switch( mode_ )
    {
        case modes::block:
            if( blocks_.empty() )
            {
                if( event->modifiers() == Qt::ControlModifier || event->modifiers() == ( Qt::ControlModifier | Qt::ShiftModifier ) )
                {
                    switch( event->key() )
                    {
                        case Qt::Key_Plus: ++block_; break;
                        case Qt::Key_Minus: --block_; break;
                        case Qt::Key_C: block_ = 0; break;
                        default: break;
                    }
                }
                else if( event->modifiers() == Qt::NoModifier )
                {
                    const std::string& s = event->text().toStdString();
                    switch( event->key() )
                    {
                        case Qt::Key_Minus: block_ = -block_; break;
                        case Qt::Key_Up: ++block_; break;
                        case Qt::Key_Down: --block_; break;
                        case Qt::Key_Backspace: block_ = block_ / 10; break;
                        default: if( '0' <= s[0] && s[0] <= '9' ) { block_ = block_ * 10 + ( s[0] - '0' ) * ( block_ < 0 ? -1 : 1 ); } break;
                    }
                }
            }
            else
            {
                if( event->modifiers() == Qt::NoModifier )
                {
                    switch( event->key() )
                    {
                        case Qt::Key_Up:
                            ++blocks_index_;
                            if( blocks_index_ == blocks_.size() ) { blocks_index_ = 0; }
                            block_ = blocks_[ blocks_index_ ];
                            break;
                        case Qt::Key_Down:
                            blocks_index_ = ( blocks_index_ == 0 ? blocks_.size() : blocks_index_ ) - 1;
                            block_ = blocks_[ blocks_index_ ];
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        case modes::label:
            if( labels_.empty() )
            {
                if( event->modifiers() == Qt::ControlModifier )
                {
                    switch( event->key() )
                    {
                        case Qt::Key_C: label_ = ""; break;
                        default: break;
                    }
                }
                else if( event->modifiers() == Qt::NoModifier || event->modifiers() == Qt::ShiftModifier )
                {
                    switch( event->key() )
                    {
                        case Qt::Key_Backspace: if( !label_.empty() ) { label_.pop_back(); } break;
                        case Qt::Key_Comma: label_ += "\\,"; break;
                        case Qt::Key_QuoteDbl: label_ += "\\\""; break;
                        default: if( event->text()[0].isPrint() ) { label_ += event->text().toStdString(); } break;
                    }
                }
            }
            else
            {
                if( event->modifiers() == Qt::NoModifier )
                {
                    switch( event->key() )
                    {
                        case Qt::Key_Up:
                            ++labels_index_;
                            if( labels_index_ == labels_.size() ) { labels_index_ = 0; }
                            label_ = labels_[ labels_index_ ];
                            break;
                        case Qt::Key_Down:
                            labels_index_ = ( labels_index_ == 0 ? labels_.size() : labels_index_ ) - 1;
                            label_ = labels_[ labels_index_ ];
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        case modes::none:
            break;
    }
}
                    
} } } // namespace snark { namespace graphics { namespace view {
