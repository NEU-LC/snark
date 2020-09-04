// Copyright (c) 2020 Vsevolod Vlaskine

#pragma once

#include <string>
#include <vector>
#include <QKeyEvent>
#include <comma/base/types.h>

namespace snark { namespace graphics { namespace view {

struct click_mode // quick and dirty
{
    class double_right_click_t
    {
        public:
            struct modes { enum values { none, block, label }; };
            double_right_click_t( const std::string& options );
            modes::values mode() const { return mode_; }
            comma::int32 block() const { return block_; }
            const std::string& label() const { return label_; }
            std::string to_output_string() const; // quick and dirty
            std::string to_info_string() const;
            void on_key_press( QKeyEvent* event );
            void toggle( modes::values m, bool flag );
            
        private:
            modes::values mode_;
            comma::int32 block_;
            std::vector< comma::int32 > blocks_;
            unsigned int blocks_index_;
            std::string label_;
            std::vector< std::string > labels_;
            unsigned int labels_index_;
    };
    
    click_mode( const std::string& options ): double_right_click( options ) {}
    
    double_right_click_t double_right_click;
};

} } } // namespace snark { namespace graphics { namespace view {
