#ifndef SNARK_IMAGING_CVMAT_SERIALIZATION_H_
#define SNARK_IMAGING_CVMAT_SERIALIZATION_H_

#include <iostream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <comma/base/types.h>
#include <comma/csv/binary.h>
#include <comma/visiting/traits.h>


namespace snark{ namespace cv_mat {

class serialization
{
    public:
        /// header for cv::Mat serialization
        struct header
        {
            boost::posix_time::ptime timestamp;
            comma::uint32 rows;
            comma::uint32 cols;
            comma::uint32 type; /// cv::Mat type (see http://opencv.willowgarage.com/documentation/cpp/basic_structures.html)
            comma::uint32 size; /// data size, convenience member equal to rows*cols
            
            header();
            header( const cv::Mat& m );
            header( const std::pair< boost::posix_time::ptime, cv::Mat >& p ); /// constructor
        };
        
        /// options, a helper class
        struct options
        {
            std::string fields;
            comma::uint32 rows;
            comma::uint32 cols;
            std::string type;
            bool no_header;
            bool header_only;
            
            options() : no_header( false ), header_only( false ) {}
            header get_header() const; /// make header (to be used as default)
            static std::string usage();
            static std::string type_usage();
        };

        /// default constructor
        serialization();

        /// constructor
        serialization( const std::string& fields, const comma::csv::format& format, bool headerOnly = false, const header& default_header = header() );
        
        /// constructor
        serialization( const options& options );

        /// serialize cv::Mat, return image size in buffer
        std::size_t put( const std::pair< boost::posix_time::ptime, cv::Mat >& m, char* buf ) const;

        /// deserialize cv::Mat, return image size in buffer
        /// @note get timestamp from header()
        std::size_t get( std::pair< boost::posix_time::ptime, cv::Mat >& m, const char* buf ) const;

        /// deserialize cv::Mat, convenience class
        std::pair< boost::posix_time::ptime, cv::Mat > get( const char* buf ) const;
        
        /// deserialize header only
        header get_header( const char* buf ) const;

        /// return usage
        static const std::string& usage();

        /// return necessary buffer size
        std::size_t size( const cv::Mat& m ) const;

        /// same as above
        std::size_t size( const std::pair< boost::posix_time::ptime, cv::Mat >& m ) const;

        /// read from stream, if eof, return empty cv::Mat
        std::pair< boost::posix_time::ptime, cv::Mat > read( std::istream& is );

        /// write to stream
        void write( std::ostream& os, const std::pair< boost::posix_time::ptime, cv::Mat >& m );

    private:
        boost::scoped_ptr< comma::csv::binary< header > > m_binary;
        std::vector< char > m_buffer;
        bool m_headerOnly;
        header m_header; /// default header
};

} }  // namespace snark{ namespace cv_mat {
    
namespace comma { namespace visiting {

template <> struct traits< snark::cv_mat::serialization::header >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cv_mat::serialization::header& h, V& v )
    {
        v.apply( "t", h.timestamp );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "size", h.size );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cv_mat::serialization::header& h, V& v )
    {
        v.apply( "t", h.timestamp );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "size", h.size );
    }
};

template <> struct traits< snark::cv_mat::serialization::options >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cv_mat::serialization::options& h, V& v )
    {
        v.apply( "fields", h.fields );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "no-header", h.no_header );
        v.apply( "header-only", h.header_only );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cv_mat::serialization::options& h, V& v )
    {
        v.apply( "fields", h.fields );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "no-header", h.no_header );
        v.apply( "header-only", h.header_only );
    }
};
    
} } // namespace comma { namespace visiting {

#endif // SNARK_IMAGING_CVMAT_SERIALIZATION_H_
