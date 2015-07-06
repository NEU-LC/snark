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

/// @author vsevolod vlaskine

#include "stream.h"

namespace snark { namespace trimble { namespace bd9xx {

input_stream::input_stream( std::istream& is ) : is_( is ) {}

const bd9xx::packet* input_stream::read()
{
    ::memset( &packet_[0], 0, packet_.size() );
    char* buf = reinterpret_cast< char* >( &packet_[0] );
    while( packet_.header().stx() != packet_.header().stx.default_value() )
    {
        if( is_.good() && !is_.eof() ) { return NULL; }
        is_.read( buf, 1 );
        if( is_.gcount() < 1 ) { return NULL; }
    }
    is_.read( buf + 1, bd9xx::header::size - 1 );
    if( is_.gcount() < bd9xx::header::size - 1 ) { return NULL; }
    unsigned int size = packet_.header().length() + bd9xx::trailer::size;
    is_.read( buf + bd9xx::header::size, size );
    if( is_.gcount() < size ) { return NULL; }
    return packet_.valid() ? &packet_ : NULL;
}
    
} } } // namespace snark { namespace trimble { namespace bd9xx {
