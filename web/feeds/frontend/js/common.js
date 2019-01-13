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

/**
 * Created by vrushali on 28/10/15.
 */
/**
 * Created by vrushali on 10/04/15.
 * common.js
 *
 * <p>
 *        Description:
 *            This module represents the controller for this application.
 *
 *            Params:
 *                Parameters are loaded from static file located in include/configuration/param.config.json
 *
 *        Requirements:
 *            Require.js
 *        Example Usage:
 *            <script data-main="../include/js/controller" src="../include/lib/requirejs-2.1.8/require.js"></script>
 * </p>
 *
 * @author    : v.satpute
 * @version    : 1.0
 * @since    : 06/03/2015
 */
//--------------------------------------------//
//
// Configuration
//
// --------------------------------------------//

require.config({
    //deps: [],
    //waitSeconds: 120,
    baseUrl: '',
    //enforceDefine: true,
    paths: {
        // -----------------------------//
        // libraries
        // -----------------------------//
        jquery: 'lib/jquery/dist/jquery.min',
        jquery_ui: 'lib/jquery-ui/jquery-ui.min',
        jquery_mobile: "lib/jquery-mobile/jquery.mobile-1.4.5",
        jquery_timeago: 'lib/jquery-timeago/jquery.timeago',
        bootstrap: ['lib/bootstrap/dist/js/bootstrap.min'],
        dat_gui: ['lib/dat-gui/build/dat.gui.snark'],
        dat_gui_mobile: ['lib/dat-gui/build/dat.gui.mobile'],
        mobile_detect: ['lib/Mobile-Detect/mobile-detect'],
        ol: ['lib/OpenLayers/build/ol'],
        // --------------------------------------------//
        //
        // app scripts
        //
        // --------------------------------------------//
        base_controller: 'js/base_controller',
        Feed: 'js/model/Feed',
        CsvFeed: 'js/model/CsvFeed',
        GraphFeed: 'js/model/GraphFeed',
        ImageFeed: 'js/model/ImageFeed',
        ImageStreamFeed: 'js/model/ImageStreamFeed',
        TextFeed: 'js/model/TextFeed',
        TrackFeed: 'js/model/TrackFeed',
        TrackOptions: 'js/model/TrackOptions',
        MapFeed: 'js/model/MapFeed',
        StartStopFeed: 'js/model/StartStopFeed',
        MapOptions: 'js/model/MapOptions',
        Grid: 'js/model/Grid',
        GridOptions: 'js/model/GridOptions',
        // -----------------------------//
        // models
        // -----------------------------//
        utils: "js/utils"

    },
// -----------------------------//
// shim
// -----------------------------//
    shim: {
        jquery: {
            exports: "$"
        },
        bootstrap: ["jquery", "jquery_ui"],
        jquery_mobile: ["jquery", "jquery_ui"],
        'jquery_timeago': ["jquery"],
        //'dat_gui': ['jquery_timeago', 'bootstrap'],
        'feed': {
            deps: ["dat_gui", "jquery_mobile"]
//exports: "Feed"
        }
//        'controller': {
//            deps: ["dat_gui", "Feed", "utils"],
//            exports: 'controller'
//        }
    }
});

