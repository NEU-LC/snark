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
        FormFeed: 'js/model/FormFeed',
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

