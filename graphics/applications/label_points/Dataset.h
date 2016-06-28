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


#ifndef SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_DATASET_H_
#define SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_DATASET_H_

#include <deque>
#ifndef Q_MOC_RUN
#include <comma/base/types.h>
#include <comma/csv/options.h>
#include "../../../math/interval.h"
#endif
#include "../../../graphics/qt3d/vertex_buffer.h"
#include "PointMap.h"
#include "PointWithId.h"
#include <Qt3D/qglpainter.h>

namespace snark { namespace graphics { namespace view {

class BasicDataset
{
    public:
        struct Data
        {
            comma::uint32 id;
            std::size_t index;
            Data() {}
            Data( const Data& rhs ) { operator=( rhs ); }
            Data( comma::uint32 id, std::size_t index ) : id( id ), index( index ) {}
        };
        typedef PointMap< Eigen::Vector3d, Data > Points;
        typedef std::map< comma::uint32, Points > Partitions;    
        BasicDataset();
        BasicDataset( const Eigen::Vector3d& offset );
        const Points& points() const;
        const Partitions& partitions() const;
        const Eigen::Vector3d& offset() const;
        const math::closed_interval< double, 3 >& extents() const;
        void init();
        void draw( QGLPainter* painter ) const;
        void visible( bool visible );
        bool visible() const;
        void clear();
        void insert( const Points& m );
        void erase( const Points& m );
    
    protected:
        bool m_visible;
        Points m_points;
        Partitions m_partitions;
        boost::scoped_ptr< qt3d::vertex_buffer > m_vertices;
        boost::optional< Eigen::Vector3d > m_offset;
        boost::optional< math::closed_interval< double, 3 > > m_extents;
        void insert( const Eigen::Vector3d& p, const Data& data );
};

class Dataset : public BasicDataset
{
    public:
        Dataset( const std::string& filename, const comma::csv::options& options, bool relabelDuplicated );
        Dataset( const std::string& filename, const comma::csv::options& options, const Eigen::Vector3d& offset, bool relabelDuplicated );
        void save();
        void saveAs( const std::string& f );
        void backup();
        void label( const Eigen::Vector3d& p, comma::uint32 id );
        void label( const Points& p, comma::uint32 id );
        void writable( bool enabled );
        bool writable() const;
        bool modified() const;
        void commit();
        BasicDataset& selection();
        const BasicDataset& selection() const;
        const std::string& filename() const;
        const comma::csv::options& options() const;
        bool valid() const;
        static void repair( const comma::csv::options& options );
    
    private:
        void load();
        void insert( const Points& m );
        void erase( const Points& m );
        void clear();
        std::size_t labelimpl( const Eigen::Vector3d& p, comma::uint32 id );
        void labelDuplicated();
        //typedef std::deque< std::pair< PointWithId, std::vector< std::string > > > Deque;
        typedef std::deque< std::pair< PointWithId, std::string > > Deque;
        Deque m_deque;
        std::string m_filename;
        const comma::csv::options m_options;
        boost::scoped_ptr< BasicDataset > m_selection;
        bool m_writable;
        bool m_modified;
        bool m_valid;
};

} } } // namespace snark { namespace graphics { namespace view {

#endif // SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_DATASET_H_
