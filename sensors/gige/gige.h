// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_SENSORS_GIGE_H_
#define SNARK_SENSORS_GIGE_H_

#include <PvApi.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

#include <opencv2/core/core.hpp>


namespace snark{ namespace camera{

/// image acquisition from gige camera
class gige
{
    public:
        /// attributes map type
        typedef std::map< std::string, std::string > attributes_type;
        
        /// constructor; default id: connect to any available camera
        gige( unsigned int id = 0, const attributes_type& attributes = attributes_type() );

        /// destructor
        ~gige();

        /// return attributes
        attributes_type attributes() const;

        /// get timestamped frame
        std::pair< boost::posix_time::ptime, cv::Mat > read();

        /// return camera id
        unsigned int id() const;

        /// return total bytes per frame
        unsigned long total_bytes_per_frame() const;

        /// close
        void close();

        /// list cameras
        static std::vector< tPvCameraInfo > list_cameras();

        /// callback
        class callback
        {
            public:
                /// constructor: start capture, call callback on frame update
                callback( gige& gige, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame );

                /// destructor: stop capture
                ~callback();
                
                /// implementation class, a hack: has to be public to use pvAPI callback, sigh...
                class Impl_;
                
                /// return true, if callback status is ok
                bool good() const;
                
            private:
                friend class gige;
                Impl_* pimpl_;
        };
        
    private:
        friend class callback::Impl_;
        class Impl_;
        Impl_* pimpl_;
};

} } // namespace snark{ namespace camera{

#endif // SNARK_SENSORS_GIGE_H_
