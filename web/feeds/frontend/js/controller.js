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
//    config_file: 'config/web.frontend.json',
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
//var audio_context = audioContextCheck();
require(['jquery', "jquery_ui",
    "jquery_timeago",
    "bootstrap",
    "dat_gui", "Feed", "CsvFeed",
    'TextFeed', 'ImageFeed',
    'GraphFeed', 'ImageStreamFeed', 'TrackFeed', 'utils'], function ($) {

    var Feed = require('Feed');
    var ImageFeed = require('ImageFeed');
    var TextFeed = require('TextFeed');
    var GraphFeed = require('GraphFeed');
    var ImageStreamFeed = require('ImageStreamFeed');
    var CsvFeed = require('CsvFeed');
    var TrackFeed = require('TrackFeed');


    var current_config_file;
    var container_width;


    $(function () {
        container_width = $('#container').width();
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

    function hex2rgb(hex) {
        var r = parseInt(hex.substr(1,2), 16);
        var g = parseInt(hex.substr(3,2), 16);
        var b = parseInt(hex.substr(5,2), 16);
        return [r,g,b];
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
            width: 500
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

        for (var feed_name in frontend_config.feeds) {
            var config = frontend_config.feeds[feed_name];
            if (!('type' in config)) {
                config.type = 'image';
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
                if (!('track' in config)) {
                    config.track = {};
                }
                if (!('background_url' in config.track)) {
                    config.track.background_url = '';
                }
                if (!('draw_interval' in config.track)) {
                    config.track.draw_interval = 100
                }
                if (!('alpha_step' in config.track)) {
                    config.track.alpha_step = 0.05;
                }
                if (!('radius' in config.track)) {
                    config.track.radius = 3;
                }
                if (!('fill' in config.track)) {
                    config.track.fill = [57, 220, 31];
                }
                if (!('stroke' in config.track)) {
                    config.track.stroke = [23, 140, 45];
                }
                if (!('strokeWidth' in config.track)) {
                    config.track.strokeWidth = 1;
                }
                if (config.track.fill.constructor !== Array) {
                    config.track.fill = hex2rgb(config.track.fill);
                }
                if (config.track.stroke.constructor !== Array) {
                    config.track.stroke = hex2rgb(config.track.stroke);
                }
            }
            if (!('alert' in config)) {
                config.alert = true;
            }
            var feed = create_feed(config.type, feed_name, config);
            var folder = gui.addFolder(feed_name);
            folder.close();
            folder.add(feed.config, 'url').onFinishChange(function (value) {
                var feed_name = $(this.__gui.__ul).find('li.title').text();
                feeds[feed_name].reset();
            });
            if (config.type != 'stream') {
                folder.add(feed.config, 'view', Feed.views).onFinishChange(function (value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].update_view();
                    if (value == 'hide') {
                        gui.setProperty('auto', false, feed_name);
                    }
                });
                folder.add(feed.config.refresh, 'auto').name("auto refresh").onFinishChange(function (value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    var feed = feeds[feed_name];
                    if (value && feed.config.view == 'hide') {
                        gui.setProperty('view', 'show', feed_name);
                    }
                    feed.reset();
                });
                folder.add(feed.config.refresh, 'interval', 0, 90).name("refresh interval").step(1).onFinishChange(function (value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].reset();
                });
                if (config.type == 'text') {
                    folder.add(feed.config.text, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].draw();
                    });
                }
                if (config.type == 'csv' || config.type == 'csv-table') {
                    folder.add(feed.config.csv, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].draw();
                    });
                    folder.add(feed.config.csv, 'fields').onChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].init_fields();
                    });
                    folder.add(feed.config.csv, 'min').onFinishChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].init_ranges();
                    });
                    folder.add(feed.config.csv, 'max').onFinishChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].init_ranges();
                    });
                    folder.add(feed.config.csv, 'threshold_alert').name('threshold alert');
                }
                if (config.type == 'track') {
                    folder.add(feed.config.track, 'background_url').name('background url').onFinishChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].set_background();
                    });
                    folder.add(feed.config.track, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].reset_draw_interval();
                    });
                    folder.add(feed.config.track, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
                    folder.add(feed.config.track, 'radius', 0.5, 10).step(0.25);
                    folder.addColor(feed.config.track, 'fill').onChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].config.track.fill = value.constructor === Array ? value.map(Math.round) : hex2rgb(value);
                    });
                    folder.addColor(feed.config.track, 'stroke').onChange(function (value) {
                        var feed_name = $(this.__gui.__ul).find('li.title').text();
                        feeds[feed_name].config.track.stroke = value.constructor === Array ? value.map(Math.round) : hex2rgb(value);
                    });
                    folder.add(feed.config.track, 'strokeWidth', 0, 5).name('stroke width').step(0.5);
                }
                folder.add(feed.config, 'alert').name('feed alert').onFinishChange(function (value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    if (!value) {
                        feeds[feed_name].alert(false);
                    }
                });
            } else {
                folder.add(feed.config, 'show').onFinishChange(function (value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].toggle_show();
                    if (!value) {
                        gui.setProperty('auto', false, feed_name);
                    }
                });
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

        $('.panel').on('mouseover', function () {
            $(this).find('.hideable').removeClass('transparent');
            $(this).find('div button').addClass('button-shadow');
        });
        $('.panel').on('mouseout', function () {
            $(this).find('.hideable').addClass('transparent');
            $(this).find('div button').removeClass('button-shadow');
        });

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
    //
    //function save_last_config_file(config_file) {
    //    var key = 'feeds.config_file';
    //    localStorage.setItem(key, config_file);
    //}
    //
    //function load_last_config_file() {
    //    var key = 'feeds.config_file';
    //    return localStorage.getItem(key);
    //}
    //
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
    //        id: 'container',
    //        width: $('#container').width()
    //    });
    //    localStorage.setItem(key, JSON.stringify(layout));
    //};
    //
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
    //        if (value.id === 'container') {
    //            container.width(value.width);
    //            return;
    //        }
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
    //
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
            initialize(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            $('#container').append('<p>error reading <a href="' + file + '">' + file + '</a>: ' + textStatus + ' ' + errorThrown + '</p>');
            $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
            $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
        });
    }

    var query_string = parse_query_string();


    dat.GUI.prototype.setProperty = function (property, value, opt_folder_name) {
        if (opt_folder_name && opt_folder_name in this.__folders) {
            return this.__folders[opt_folder_name].setProperty(property, value);
        }
        for (var i in this.__controllers) {
            var controller = this.__controllers[i];
            if (controller.property == property) {
                controller.setValue(value);
                break;
            }
        }
    };

    dat.GUI.prototype.toggleProperty = function (property, opt_folder_name) {
        if (opt_folder_name && opt_folder_name in this.__folders) {
            return this.__folders[opt_folder_name].toggleProperty(property);
        }
        for (var i in this.__controllers) {
            var controller = this.__controllers[i];
            if (controller.property == property) {
                controller.setValue(!controller.getValue());
                break;
            }
        }
    };

    var add_panel = function (feed_name) {
        var class_str = " transparent  ";
        $('#container').append(
            '<li id="' + feed_name + '" class="panel ">' +
            '  <button type="button" class="panel-close hideable text-muted  ' + class_str + 'pull-right" title="close"><span>&times;</span></button>' +
            '</li>'
        );
    };

    //var add_poll_body = function (feed_name, element, popup_div) {
    //    var id = '#' + feed_name;
    //    $(id).append(
    //        '<h3>' + feed_name +
    //        '  <button class="panel-refresh hideable transparent" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: auto refresh"><span class="status text-muted glyphicon glyphicon-refresh"></span></button>' +
    //        '  <button class="panel-settings hideable  transparent" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button>' +
    //        '  <button class="panel-compact hideable  transparent" title="compact"><span class="text-muted glyphicon glyphicon-resize-small"></span></button>' +
    //        '</h3>' +
    //        '<div class="panel-body">' +
    //        '  <time class="timestring time small">&nbsp;</time>' +
    //        '  <time class="timeago time small">&nbsp;</time>' +
    //        element +
    //        '</div>'
    //    );
    //    $(id).append(popup_div);
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
            var element_str = '<img class="target"/>';
            add_poll_body(feed_name, element_str);
            return new ImageFeed(feed_name, config);
        } else if (type == 'text' || type == 'csv' || type == 'csv-table') {
            add_poll_body(feed_name, '<table class="target"><thead></thead></table>');
            return type == 'text' ? new TextFeed(feed_name, config) : new CsvFeed(feed_name, config);
        } else if (type == 'graph') {
            add_poll_body(feed_name, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-y-labels"></div><div class="graph-bars"></div></div>');
            return new GraphFeed(feed_name, config);
        } else if (type == 'stream') {
            add_stream_body(feed_name, '<img class="target"/>');
            return new ImageStreamFeed(feed_name, config);
        } else if (type == 'track') {
            add_poll_body(feed_name, '<canvas class="target" resize></canvas>');
            return new TrackFeed(feed_name, config);
        }
        throw 'unrecognised feed type: ' + type;
    };

});

