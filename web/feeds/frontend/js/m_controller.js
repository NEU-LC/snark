/**
 * Created by vrushali on 10/04/15.
 *
 *
 * @author    : v.satpute
 * @version    : 1.0
 * @since    : 06/03/2015
 */

//---------------------------------------------//
//
// Global Constants
//
// --------------------------------------------//

//---------------------------------------------//
//
// Global Utils
//
// --------------------------------------------//
//-----------------------------//
// util
// -----------------------------//

//-----------------------------//
// model
// -----------------------------//
var feed;
//var globals = {
//    refresh: function () {
//        $.each(feeds, function (index, feed) {
//            feed.refresh();
//        });
//    },
//    start_auto_refresh: function () {
//        $.each(feeds, function (index, feed) {
//            if (feed.show) {
//                gui.setProperty('auto', true, feed.feed_name);
//            }
//        });
//    },
//    stop_auto_refresh: function () {
//        $.each(feeds, function (index, feed) {
//            gui.setProperty('auto', false, feed.feed_name);
//        });
//    },
//    start_streams: function () {
//        $.each(feeds, function (index, feed) {
//            if (feed.config.type === 'stream') {
//                feed.reset();
//            }
//        });
//    },
//    stop_streams: function () {
//        $.each(feeds, function (index, feed) {
//            if (feed.config.type === 'stream') {
//                feed.stop();
//            }
//        });
//    },
//    start: function () {
//        this.start_auto_refresh();
//        this.start_streams();
//    },
//    stop: function () {
//        this.stop_auto_refresh();
//        this.stop_streams();
//    },
//    show: function () {
//        $.each(feeds, function (index, feed) {
//            if (feed.config.type == 'stream') {
//                gui.setProperty('show', true, feed.feed_name);
//            } else {
//                gui.setProperty('view', 'show', feed.feed_name);
//            }
//        });
//    },
//    compact: function () {
//        $.each(feeds, function (index, feed) {
//            if (feed.config.type != 'stream') {
//                gui.setProperty('view', 'compact', feed.feed_name);
//            }
//        });
//    },
//    hide: function () {
//        $.each(feeds, function (index, feed) {
//            if (feed.config.type == 'stream') {
//                gui.setProperty('show', false, feed.feed_name);
//            } else {
//                gui.setProperty('view', 'hide', feed.feed_name);
//            }
//        });
//    },
//    reset: function () {
//        reset(globals.config_file);
//        $('#container').width(container_width);
//        load_config(globals.config_file);
//    },
//    enable_alerting: function () {
//        $.each(feeds, function (index, feed) {
//            gui.setProperty('alert', true, feed.feed_name);
//            gui.setProperty('threshold_alert', true, feed.feed_name);
//        });
//    },
//    disable_alerting: function () {
//        $.each(feeds, function (index, feed) {
//            gui.setProperty('alert', false, feed.feed_name);
//            gui.setProperty('threshold_alert', false, feed.feed_name);
//        });
//    },
//    clear_alerts: function () {
//        $.each(feeds, function (index, feed) {
//            feed.alert(false);
//        });
//    },
//    config_dir: 'config',
//    config_file: 'config/tab.frontend.json',
//    alert_beep: true,
//    beep: function () {
//        if (audio_context != undefined) {
//            var oscillator = audio_context.createOscillator();
//            oscillator.connect(audio_context.destination);
//            oscillator.start(0);
//            oscillator.stop(audio_context.currentTime + 0.2);
//        }
//    }
//};
//var audio_context = audioContextCheck();

//function audioContextCheck() {
//    if (typeof AudioContext !== "undefined") {
//        return new AudioContext();
//    } else if (typeof webkitAudioContext !== "undefined") {
//        return new webkitAudioContext();
//    } else if (typeof mozAudioContext !== "undefined") {
//        return new mozAudioContext();
//    } else {
//
//        // Do stuff with soundmanager or something else if Web Audio API is not supported
//    }
//}

//---------------------------------------------//
//
// Global Variables
//
// --------------------------------------------//
var feeds = {};
var pending = {};
var config_files = [];
var gui;


require(['jquery', 'jquery_mobile',
    "jquery_timeago", "bootstrap", "ol", "dat_gui_mobile", "Feed", "CsvFeed",
    'TextFeed', 'ImageFeed', 'GraphFeed', 'ImageStreamFeed', 'TrackFeed', 'TrackOptions', 'MapFeed', 'MapOptions', 'GridOptions', 'utils'], function ($) {
    var Feed = require('Feed');
    var ImageFeed = require('ImageFeed');
    var TextFeed = require('TextFeed');
    var GraphFeed = require('GraphFeed');
    var ImageStreamFeed = require('ImageStreamFeed');
    var CsvFeed = require('CsvFeed');
    var TrackFeed = require('TrackFeed');
    var TrackOptions = require('TrackOptions');
    var MapFeed = require('MapFeed');
    var MapOptions = require('MapOptions');
    var GridOptions = require('GridOptions');

    globals.config_file = 'config/tab.frontend.json';
    globals.isMobile = true;
    var current_config_file;
    var container_width;

    $(function () {
        container_width = $('#container').width();
        //load menu
        //TODO: remove this later
        //ismobile = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
        //ismobile = true;
        init_load_config();

    });

    function init_load_config() {
        $.ajax({
            url: globals.config_dir
        }).done(function (data, textStatus, jqXHR) {
            $(data).find('a').each(function (index) {
                var href = $(this).attr('href');
                if (href.match(/[.]json$/)) {
                    config_files.push(globals.config_dir + '/' + href);
                }
            });
            if ((config_files.length > 0) && config_files.indexOf(globals.config_file) < 0) {
                globals.config_file = config_files[0];
            }
        }).fail(function (jqXHR, textStatus, errorThrown) {
        }).always(function (data_jqXHR, textStatus, errorThrown) {
            var file = query_string['config'];
            if (file) {
                globals.config_file = file;
            } else {
                var config_file = load_last_config_file();
                if (config_file && config_files.indexOf(config_file) != -1) {
                    globals.config_file = config_file;
                }
            }
            if (config_files.indexOf(globals.config_file) == -1) {
                config_files.push(globals.config_file);
            }
            load_config(globals.config_file);
        });
    }

    function get_feed_name(element) {
        return $(element).closest('ul').find('li.title').first().text();
    }

    function add_gui_track_options(folder, config) {
        var track_folder = folder.addFolder('track options');
        $(track_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
        track_folder.add(config, 'image').onFinishChange(function (value) {
            var feed_name = get_feed_name(track_folder.domElement);
            feeds[feed_name].set_background();
        });
        track_folder.add(config, 'extent').onFinishChange(function (value) {
            var feed_name = get_feed_name(track_folder.domElement);
            var feed = feeds[feed_name];
            feed.set_extent();
        });
        track_folder.add(config, 'scale', 1, 300).name('scale (%)').step(0.01).onChange(function (value) {
            var feed_name = get_feed_name(track_folder.domElement);
            if (value < feeds[feed_name].min_scale()) {
                return;
            }
            feeds[feed_name].resize();
        });
        track_folder.add(config, 'trail').onChange(function (value) {
            var feed_name = get_feed_name(track_folder.domElement);
            if (!value) {
                feeds[feed_name].remove_trail();
            }
        });
        track_folder.add(config, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value) {
            var feed_name = get_feed_name(track_folder.domElement);
            feeds[feed_name].reset_draw_interval();
        });
        track_folder.add(config, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
        track_folder.add(config, 'radius', 0.5, 20).step(0.1);
        track_folder.addColor(config, 'fill');
        track_folder.addColor(config, 'stroke');
        track_folder.add(config, 'stroke_width', 0, 5).name('stroke width').step(0.5);
    }

    function add_gui_map_options(folder, config) {
        var map_folder = folder.addFolder('map options');
        $(map_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
        map_folder.add(config, 'imagery_set', ['', 'Aerial', 'AerialWithLabels', 'Road']).onFinishChange(function (value) {
            var feed_name = get_feed_name(map_folder.domElement);
            var feed = feeds[feed_name];
            if (value) {
                feed.set_base_tile();
            } else {
                feed.set_base_layer();
            }
        });
        map_folder.add(config, 'bing_maps_key').name('bing maps key').onFinishChange(function (value) {
            var feed_name = get_feed_name(map_folder.domElement);
            var feed = feeds[feed_name];
            feed.set_base_tile();
        });
        map_folder.add(config, 'image').onFinishChange(function (value) {
            var feed_name = get_feed_name(map_folder.domElement);
            feeds[feed_name].set_base_layer();
        });
        map_folder.add(config, 'extent').onFinishChange(function (value) {
            var feed_name = get_feed_name(map_folder.domElement);
            feeds[feed_name].set_base_layer();
        });
        map_folder.add(config, 'follow');
        map_folder.add(config, 'trail');
        map_folder.add(config, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value) {
            var feed_name = get_feed_name(map_folder.domElement);
            feeds[feed_name].reset_draw_interval();
        });
        map_folder.add(config, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
        map_folder.add(config, 'radius', 0.5, 20).step(0.1);
        map_folder.addColor(config, 'fill');
        map_folder.addColor(config, 'stroke');
        map_folder.add(config, 'stroke_width', 0, 5).name('stroke width').step(0.5);
    }

    function add_gui_grid_options(folder, config) {
        var grid_folder = folder.addFolder('grid options');
        $(grid_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
        grid_folder.add(config, 'show').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            value ? feed.add_grid() : feed.remove_grid();
        });
        grid_folder.add(config.x, 'min', -100, 100).step(1).name('x min').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.x, 'max', -100, 100).step(1).name('x max').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.x, 'step', 0, 100).step(1).name('x step').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.addColor(config.x, 'color').name('x color').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.y, 'min', -100, 100).step(1).name('y min').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.y, 'max', -100, 100).step(1).name('y max').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.y, 'step', 0, 100).step(1).name('y step').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.addColor(config.y, 'color').name('y color').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config, 'axis_width', 0.5, 20).step(0.5).name('axis width').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.grid_lines, 'show').name('grid lines').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.addColor(config.grid_lines, 'color').name('grid lines color').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config.grid_lines, 'width', 0.5, 10).step(0.1).name('grid lines width').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config, 'x_offset', -100, 100).step(1).name('x offset (pixels)').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
        grid_folder.add(config, 'y_offset', -100, 100).step(1).name('y offset (pixels)').onChange(function (value) {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed = feeds[feed_name];
            feed.draw_grid();
        });
    }

    function initialize(frontend_config) {
        current_config_file = globals.config_file;
        globals.stop();
        $('#container').empty();
        try {
            $('#container').sortable('destroy');
        } catch (e) {
        }
        try {
            $('#container').resizable('destroy');
        } catch (e) {
        }
        feeds = {};
        pending = {};
        if (gui) {
            gui.destroy();
        }

        gui = new dat.GUI({
            width: 300

            //autoPlace: false
        });
        gui.add(globals, 'config_file', config_files).name('config file').onFinishChange(function (value) {
            load_config(globals.config_file);
        });
        var folder = gui.addFolder('globals');
        folder.add(globals, "refresh");
        folder.add(globals, "start");
        folder.add(globals, "stop");
        folder.add(globals, "show");
        folder.add(globals, "compact");
        folder.add(globals, "hide");
        folder.add(globals, "save");
        folder.add(globals, "reset");
        folder.add(globals, "enable_alerting").name("enable alerting");
        folder.add(globals, "disable_alerting").name("disable alerting");
        folder.add(globals, "clear_alerts").name("clear alerts");
        folder.add(globals, "alert_beep").name("alert beep");

        var grid_types = ['image', 'stream', 'track'];

        for (var feed_name in frontend_config.feeds) {
            var config = frontend_config.feeds[feed_name];
            if (!('type' in config)) {
                config.type = 'image';
            }
            if (grid_types.indexOf(config.type) >= 0) {
                config.grid = new GridOptions(config.grid);
            }
            if (config.type == 'stream') {
                config.show = true;
                if (!('stream' in config)) {
                    config.stream = {autoplay: false};
                }
                if (!('url' in config)) {
                    var xpath = config.xpath || feed_name;
                    config.url = frontend_config.websocket + '?xpath=' + xpath + '&data_uri=true';
                }
            } else {
                config.view = Feed.views[0];
                if (!('refresh' in config)) {
                    config.refresh = {};
                }
                if (!('auto' in config.refresh)) {
                    config.refresh.auto = 'interval' in config.refresh && config.refresh.interval > 0;
                }
                if (!('interval' in config.refresh)) {
                    config.refresh.interval = 2;
                }
                if (!('url' in config)) {
                    config.url = frontend_config.host + '/' + feed_name;
                }
            }
            if (config.type == 'text') {
                if (!('text' in config)) {
                    config.text = {};
                }
                if (!('show_items' in config.text)) {
                    config.text.show_items = 1;
                }
            } else if (config.type == 'csv' || config.type == 'csv-table') {
                if (!('csv' in config)) {
                    config.csv = {};
                }
                if (!('show_items' in config.csv)) {
                    config.csv.show_items = 1;
                }
                if (!('fields' in config.csv)) {
                    config.csv.fields = '';
                }
                if (!('min' in config.csv)) {
                    config.csv.min = '';
                }
                if (!('max' in config.csv)) {
                    config.csv.max = '';
                }
                if (!('min_color' in config.csv) || !config.csv.min_color) {
                    config.csv.min_color = 'orange';
                }
                if (!('max_color' in config.csv) || !config.csv.max_color) {
                    config.csv.max_color = 'red';
                }
                if (!('threshold_alert' in config.csv)) {
                    config.csv.threshold_alert = false;
                }
            } else if (config.type == 'graph') {
                if (!('graph' in config)) {
                    config.graph = {};
                }
                if (!('min' in config.graph)) {
                    config.graph.min = 0;
                }
                if (!('max' in config.graph)) {
                    config.graph.max = 100;
                }
                if (!('thresholds' in config.graph)) {
                    config.graph.thresholds = [];
                }
                config.graph.thresholds.sort(function (a, b) {
                    return a.value - b.value;
                });
            } else if (config.type == 'track') {
                config.track = new TrackOptions(config.track);
            } else if (config.type == 'map') {
                config.map = new MapOptions(config.map, frontend_config);
            }
            if (!('alert' in config)) {
                config.alert = true;
            }
            var feed = create_feed(config.type, feed_name, config);
            var folder = gui.addFolder(feed_name);
            folder.close();
            folder.add(feed.config, 'url').onFinishChange(function (value) {
                var feed_name = get_feed_name(this.domElement);
                feeds[feed_name].reset();
            });
            if (config.type != 'stream') {
                folder.add(feed.config, 'view', Feed.views).onFinishChange(function (value) {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].update_view();
                    if (value == 'hide') {
                        gui.setProperty('auto', false, feed_name);
                    }
                });
                folder.add(feed.config.refresh, 'auto').name("auto refresh").onFinishChange(function (value) {
                    var feed_name = get_feed_name(this.domElement);
                    var feed = feeds[feed_name];
                    if (value && feed.config.view == 'hide') {
                        gui.setProperty('view', 'show', feed_name);
                    }
                    feed.reset();
                });
                folder.add(feed.config.refresh, 'interval', 0, 90).name("refresh interval").step(1).onFinishChange(function (value) {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].reset();
                });
                folder.add(feed.config, 'alert').name('feed alert').onFinishChange(function (value) {
                    var feed_name = get_feed_name(this.domElement);
                    if (!value) {
                        feeds[feed_name].alert(false);
                    }
                });
                if (config.type == 'text') {
                    folder.add(feed.config.text, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].draw();
                    });
                }
                if (config.type == 'csv' || config.type == 'csv-table') {
                    folder.add(feed.config.csv, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].draw();
                    });
                    folder.add(feed.config.csv, 'fields').onChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].init_fields();
                    });
                    folder.add(feed.config.csv, 'min').onFinishChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].init_ranges();
                    });
                    folder.add(feed.config.csv, 'max').onFinishChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].init_ranges();
                    });
                    folder.add(feed.config.csv, 'threshold_alert').name('threshold alert');
                }
                if (config.type == 'graph') {
                    folder.add(feed.config.graph, 'max').onFinishChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].set_labels();
                    });
                    folder.add(feed.config.graph, 'min').onFinishChange(function (value) {
                        var feed_name = get_feed_name(this.domElement);
                        feeds[feed_name].set_labels();
                    });
                }
                if (config.type == 'track') {
                    add_gui_track_options(folder, feed.config.track);
                }
                if (config.type == 'map') {
                    add_gui_map_options(folder, feed.config.map);
                }
            } else {
                folder.add(feed.config, 'show').onFinishChange(function (value) {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].toggle_show();
                    if (!value) {
                        gui.setProperty('auto', false, feed_name);
                    }
                });
            }
            if (grid_types.indexOf(config.type) >= 0) {
                add_gui_grid_options(folder, feed.config.grid);
            }
            feeds[feed_name] = feed;
        }

        load(current_config_file);

        for (var id in feeds) {
            var feed = feeds[id];
            if (!feed.el.is(':visible')) {
                continue;
            }
            if (feed.config.type == 'stream') {
                feed.start();
            } else {
                feed.refresh();
            }
        }

        $('#container').sortable({
            items: 'li.panel',
            opacity: 0.8,
            placeholder: 'panel panel-placeholder',
            cursor: 'move',
            start: function (event, ui) {
                ui.placeholder.width(ui.item.width());
                ui.placeholder.height(ui.item.height());
            }
        }).resizable({
            handles: 'e'
        });

        $('.map').hover(function() { $('#container').sortable('disable') }, function() { $('#container').sortable('enable') });
        //$('.panel').on('mouseover', function () {
        //    if (!ismobile) {
        //        $(this).find('.hideable').removeClass('transparent');
        //        $(this).find('div button').addClass('button-shadow');
        //    }
        //});
        //$('.panel').on('mouseout', function () {
        //    if (!ismobile) {
        //        $(this).find('.hideable').addClass('transparent');
        //        $(this).find('div button').removeClass('button-shadow');
        //    }
        //});

        // Global navmenu panel
        $("#main_menu_list").enhanceWithin();
        //$("#main_menu_list").trigger('create');
        $("#main_menu_list").listview();


        $('.panel button').tooltip({html: true});
        $('.panel-refresh').on('click', function (event) {
            var id = $(this).closest('li').attr('id');
            if (event.shiftKey) {
                gui.toggleProperty('auto', id);
            } else {
                feeds[id].refresh();
            }
        });
        $('.panel-stream-control').on('click', function (event) {
            var id = $(this).closest('li').attr('id');
            if (event.shiftKey) {
                feeds[id].toggle();
            } else {
                feeds[id].refresh();
            }
        });
        $('.panel-settings').on('click', function (event) {
            var id = $(this).closest('li').attr('id');
            $.each(gui.__folders, function (index, folder) {
                folder.close();
            });
            gui.__folders[id].open();
            gui.open();
        });
        $('.panel-close').on('click', function (event) {
            var id = $(this).closest('li').attr('id');
            var feed = feeds[id];
            if (feed.config.type == 'stream') {
                gui.setProperty('show', false, feed.feed_name);
            } else {
                gui.setProperty('view', 'hide', feed.feed_name);
            }
        });
        $('.panel-compact').on('click', function (event) {
            var id = $(this).closest('li').attr('id');
            var feed = feeds[id];
            if (feed.config.type != 'stream') {
                gui.setProperty('view', feed.config.view == 'compact' ? 'show' : 'compact', feed.feed_name);
            }
        });
        $('.timeago').on('mouseenter', function (e) {
            var id = $(this).closest('li').attr('id');
            var feed = feeds[id];
            feed.timeago.hide();
            feed.timestring.show();
        });
        $('.timestring').on('mouseleave', function (e) {
            var id = $(this).closest('li').attr('id');
            var feed = feeds[id];
            feed.timestring.hide();
            feed.timeago.show();
        });


        $(document).on('keydown', function (event) {
            if (event.ctrlKey) {
                toggle_sortable(false);
            }
        });
        $(document).on('keyup', function (event) {
            toggle_sortable(true);
        });
        $(window).on('focusout', function (event) {
            toggle_sortable(true);
        });
        $(window).on('beforeunload', function (e) {
            save_last_config_file(current_config_file);
        });
    }

    //function toggle_sortable(enable) {
    //    if (enable) {
    //        try {
    //            $('#container').sortable('enable');
    //        } catch (e) {
    //        }
    //        $('.panel').css('cursor', 'move');
    //    } else {
    //        try {
    //            $('#container').sortable('disable');
    //        } catch (e) {
    //        }
    //        $('.panel').css('cursor', 'auto');
    //    }
    //}

    //function save(config_file) {
    //    if (!config_file) {
    //        return;
    //    }
    //    save_layout(config_file);
    //    save_gui_config(config_file);
    //    save_last_config_file(config_file);
    //}
    //
    //function load(config_file) {
    //    if (!config_file) {
    //        return;
    //    }
    //    if (!load_gui_config(config_file)) {
    //        return;
    //    }
    //    load_layout(config_file);
    //}

    //function reset(config_file) {
    //    var keys = [];
    //    for (var i = 0; i < localStorage.length; ++i) {
    //        keys.push(localStorage.key(i));
    //    }
    //    for (var i in keys) {
    //        var key = keys[i];
    //        var re = new RegExp('^feeds[.].*\\[' + config_file + '\\]');
    //        if (key.match(re)) {
    //            localStorage.removeItem(key);
    //        }
    //    }
    //}

    //function save_last_config_file(config_file) {
    //    var key = 'feeds.config_file';
    //    localStorage.setItem(key, config_file);
    //}
    //
    //function load_last_config_file() {
    //    var key = 'feeds.config_file';
    //    return localStorage.getItem(key);
    //}

    //function save_layout(config_file) {
    //    var key = 'feeds.layout[' + config_file + ']';
    //    var layout = $('#container > li').map(function () {
    //        var id = this.id;
    //        var feed = feeds[id];
    //        var layout = {id: id};
    //        if (feed.target.is('img') && feed.target.attr('src') && feed.target.is(':visible')) {
    //            layout.width = feed.target.width();
    //            layout.height = feed.target.height();
    //        }
    //        return layout;
    //    }).get();
    //    layout.push({
    //        id: 'container'
    //    });
    //    localStorage.setItem(key, JSON.stringify(layout));
    //}

    //function load_layout(config_file) {
    //    var key = 'feeds.layout[' + config_file + ']';
    //    var layout = JSON.parse(localStorage.getItem(key));
    //    if (!layout) {
    //        return;
    //    }
    //    layout.reverse();
    //    layout.forEach(function (value, index) {
    //
    //        var container = $('#container');
    //        //if (!ismobile) {
    //        //    if (value.id === 'container') {
    //        //        container.width(value.width);
    //        //        return;
    //        //    }
    //        //}
    //        var panel = $('#' + value.id);
    //        if (!panel.length) {
    //            return;
    //        }
    //        container.prepend(panel);
    //        var feed = feeds[value.id];
    //        if (feed != undefined) {
    //            if ('width' in value) {
    //                feed.target.width(value.width);
    //            }
    //            if ('height' in value) {
    //                feed.target.height(value.height);
    //            }
    //        }
    //    });
    //}

    //function save_gui_config(config_file) {
    //    var key = 'feeds.gui.config[' + config_file + ']';
    //    var config = {
    //        globals: globals
    //    };
    //    for (var id in feeds) {
    //        config[id] = feeds[id].config;
    //    }
    //    localStorage.setItem(key, JSON.stringify(config));
    //}

    //function set_properties(config, folder_name) {
    //    for (var id in config) {
    //        var value = config[id];
    //        if (typeof value === 'object') {
    //            set_properties(value, folder_name);
    //        } else {
    //            gui.setProperty(id, value, folder_name);
    //        }
    //    }
    //}

    //function load_gui_config(config_file) {
    //    var key = 'feeds.gui.config[' + config_file + ']';
    //    var config = JSON.parse(localStorage.getItem(key));
    //    if (!config) {
    //        return false;
    //    }
    //    gui.setProperty('alert_beep', config['globals'].alert_beep, 'globals');
    //    for (var id in feeds) {
    //        if (!(id in config)) {
    //            return;
    //        }
    //    }
    //    for (var id in feeds) {
    //        set_properties(config[id], id);
    //        feeds[id].init();
    //    }
    //    return true;
    //}

    //function parse_query_string() {
    //    var query = location.search.substring(1);
    //    var pairs = query.split('&');
    //    var result = {}
    //    for (var i in pairs) {
    //        var pair = pairs[i].split('=');
    //        var key = decodeURIComponent(pair[0]);
    //        var value = decodeURIComponent(pair[1]);
    //        result[key] = value;
    //    }
    //    return result;
    //}

    function load_config(file) {
        $.ajax({
            url: file
        }).done(function (data, textStatus, jqXHR) {
            load_mobile_menu(data);
            initialize(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            $('#container').append('<p>error reading <a href="' + file + '">' + file + '</a>: ' + textStatus + ' ' + errorThrown + '</p>');
            $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
            $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
        });
    }

    var query_string = parse_query_string();

    var load_mobile_menu = function (config_data) {
        //var menu_items = [];
        for (var feed_name in config_data.feeds) {
            //menu_items.push(feed_name);
            $("ul#settings").append('<li class="ui-first-child" >' +
                    //'<input  class="custom" type="checkbox" name="checkbox-1">' +
                '<a  data="' + feed_name + '" class="">' + feed_name + '</a></li>');
        }

        // Global navmenu panel

        //$(".navmenu-panel ul").listview();


        $("#navmenu-link").on("click", function () {

            $(".main-page").find(".navmenu-panel").panel("open");
        });

        $("#settings a").on('click', function () {
            //alert("clicked" + this.data);
            var elem_id = $(this).attr('data');
            var list_container = $("ul#container");
            //list_container.find('li').addClass("hide");
            //list_container.find('#' + elem_id).removeClass("hide");
        });


    };

    //var add_poll_body = function (feed_name, element, popup_div) {
    //    var id = '#' + feed_name;
    //    $(id).append(
    //        '<h3>' + feed_name +
    //        '  <button class="panel-refresh hideable " title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: auto refresh"><span class="status text-muted glyphicon glyphicon-refresh"></span></button>' +
    //        '  <button class="panel-settings hideable " title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button>' +
    //        '  <button class="panel-compact hideable  " title="compact"><span class="text-muted glyphicon glyphicon-resize-small"></span></button>' +
    //        '</h3>' +
    //        '<div class="panel-body">' +
    //        '  <time class="timestring time small">&nbsp;</time>' +
    //        '  <time class="timeago time small">&nbsp;</time>' +
    //        element +
    //        '</div>'
    //    );
    //    $(id).append(popup_div);
    //    $("#popup" + feed_name).popup();
    //
    //};

    //var add_stream_body = function (feed_name, element) {
    //    var id = '#' + feed_name;
    //    $(id).append(
    //        '<h3>' + feed_name +
    //        '  <button class="panel-stream-control" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: start/stop"><span class="status text-muted glyphicon glyphicon-stop"></span></button>' +
    //        '  <button class="panel-settings hideable transparent" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button></h3>' +
    //        '<div class="panel-body">' +
    //        element +
    //        '</div>'
    //    );
    //};

    var create_feed = function (type, feed_name, config) {
        add_panel(feed_name);
        if (type == 'image') {
            var element_str = '<a href="#popup' + feed_name + '" data-rel="popup" data-position-to="window" data-transition="fade"><div class="target"/></a>';
            var popup_div = '<div data-role="popup" id="popup' + feed_name + '" data-overlay-theme="b" data-theme="b" data-corners="false">' +
                '<a href="#" data-rel="back" class="ui-btn ui-corner-all ui-shadow ui-btn-a ui-icon-delete ui-btn-icon-notext ui-btn-right">Close</a><img class="popphoto" style="max-height:512px;" >' +
                '</div>';
            add_poll_body(feed_name, element_str, popup_div);
            return new ImageFeed(feed_name, config);
        } else if (type == 'text' || type == 'csv' || type == 'csv-table') {
            add_poll_body(feed_name, '<table class="target"><thead></thead></table>');
            return type == 'text' ? new TextFeed(feed_name, config) : new CsvFeed(feed_name, config);
        } else if (type == 'graph') {
            add_poll_body(feed_name, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-y-labels"></div><div class="graph-bars"></div></div>');
            return new GraphFeed(feed_name, config);
        } else if (type == 'stream') {
            add_stream_body(feed_name, '<div class="target stream"/>');
            return new ImageStreamFeed(feed_name, config);
        } else if (type == 'track') {
            add_poll_body(feed_name, '<div class="target"/>');
            return new TrackFeed(feed_name, config);
        } else if (type == 'map') {
            add_poll_body(feed_name, '<div class="target map"></div>');
            return new MapFeed(feed_name, config);
        }
        throw 'unrecognised feed type: ' + type;
    };

});

