#include <comma/name_value/to_xml.h>
#include <comma/visiting/apply.h>
#include "./document.h"
#include "./traits.h"

namespace snark { namespace kml {

struct const_document_tag
{
    const kml::document* document;
    const_document_tag() : document( NULL ) {}
    const_document_tag( const kml::document& d ) : document( &d ) {}
};

} } // namespace snark { namespace kml {

namespace comma { namespace visiting {

template <> struct traits< snark::kml::const_document_tag >
{
    template< typename K, typename V > static void visit( const K&, const snark::kml::const_document_tag& t, V& v )
    {
        v.apply( "Document", *t.document );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace kml {

static const std::string header_ = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                                   "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\">\n";

static const std::string footer_ = "</kml>\n";

const std::string& header() { return header_; }

const std::string& footer() { return footer_; }

void write( std::ostream& os, const document& d )
{
    os << header_;
    comma::to_xml x( os );
    comma::visiting::apply( x ).to( const_document_tag( d ) );
    os << footer_;
}

} } // namespace snark { namespace kml {
