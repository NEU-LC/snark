/**
 * Created by vrushali on 28/10/15.
 */

var globals = {
    refresh: function () {
        $.each(feeds, function (index, feed) {
            feed.refresh();
        });
    },
    start_auto_refresh: function () {
        $.each(feeds, function (index, feed) {
            if (feed.show) {
                gui.setProperty('auto', true, feed.feed_name);
            }
        });
    },
    stop_auto_refresh: function () {
        $.each(feeds, function (index, feed) {
            gui.setProperty('auto', false, feed.feed_name);
        });
    },
    start_streams: function () {
        $.each(feeds, function (index, feed) {
            if (feed.config.type === 'stream') {
                feed.reset();
            }
        });
    },
    stop_streams: function () {
        $.each(feeds, function (index, feed) {
            if (feed.config.type === 'stream') {
                feed.stop();
            }
        });
    },
    start: function () {
        this.start_auto_refresh();
        this.start_streams();
    },
    stop: function () {
        this.stop_auto_refresh();
        this.stop_streams();
    },
    show: function () {
        $.each(feeds, function (index, feed) {
            if (feed.config.type == 'stream') {
                gui.setProperty('show', true, feed.feed_name);
            } else {
                gui.setProperty('view', 'show', feed.feed_name);
            }
        });
    },
    compact: function () {
        $.each(feeds, function (index, feed) {
            if (feed.config.type != 'stream') {
                gui.setProperty('view', 'compact', feed.feed_name);
            }
        });
    },
    hide: function () {
        $.each(feeds, function (index, feed) {
            if (feed.config.type == 'stream') {
                gui.setProperty('show', false, feed.feed_name);
            } else {
                gui.setProperty('view', 'hide', feed.feed_name);
            }
        });
    },
    save: function () {
        save(globals.config_file);
    },
    reset: function () {
        reset(globals.config_file);
        location.reload();
    },
    enable_alerting: function () {
        $.each(feeds, function (index, feed) {
            gui.setProperty('alert', true, feed.feed_name);
            gui.setProperty('threshold_alert', true, feed.feed_name);
        });
    },
    disable_alerting: function () {
        $.each(feeds, function (index, feed) {
            gui.setProperty('alert', false, feed.feed_name);
            gui.setProperty('threshold_alert', false, feed.feed_name);
        });
    },
    clear_alerts: function () {
        $.each(feeds, function (index, feed) {
            feed.alert(false);
        });
    },
    config_dir: 'config',
    config_file: 'config/web.frontend.json',
    alert_beep: false,
    beep: function () {
        if (audio_context != undefined) {
            var oscillator = audio_context.createOscillator();
            oscillator.connect(audio_context.destination);
            oscillator.start(0);
            oscillator.stop(audio_context.currentTime + 0.2);
        }
    },
    isMobile: false
};
var feed_path_to_id = function (feed_path) {
    return feed_path.replace(/\[/g, "_").replace(/\]/g, "_").replace(/\//g, "_");
};


function parse_query_string() {
    var query = location.search.substring(1);
    var parameters = query.split('&');
    var object = {};
    for (var i in parameters) {
        var key_value = parameters[i].match(/(.*?)=(.*)/);
        if (key_value) {
            var key = decodeURIComponent(key_value[1]);
            var value = decodeURIComponent(key_value[2]);
            if (value.match(/^["'].*["']$/)) {
                value = eval(value); // TODO better unquote
            }
            if (key in object) {
                if (!object[key] || object[key].constructor !== Array) {
                    object[key] = [object[key]];
                }
                object[key].push(value);
            } else {
                object[key] = value;
            }
        } else {
            object[parameters[i]] = undefined;
        }
    }
    return object;
}

function set_properties(config, folder_name) {
    for (var id in config) {
        var value = config[id];
        if (typeof value === 'object' && value.constructor !== Array) {
            set_properties(value, folder_name);
        } else {
            gui.setProperty(id, value, folder_name);
        }
    }
}

function save_gui_config(config_file) {
    var key = 'feeds.gui.config[' + config_file + ']';
    var config = {
        globals: globals
    };
    for (var id in feeds) {
        config[id] = feeds[id].config;
    }
    localStorage.setItem(key, JSON.stringify(config));
}
var audio_context = audioContextCheck();
function audioContextCheck() {
    if (typeof AudioContext !== "undefined") {
        return new AudioContext();
    } else if (typeof webkitAudioContext !== "undefined") {
        return new webkitAudioContext();
    } else if (typeof mozAudioContext !== "undefined") {
        return new mozAudioContext();
    } else {

        // Do stuff with soundmanager or something else if Web Audio API is not supported
    }
}

// function get_feed_name(element) {
//     return feed_path_to_id($(element).closest('ul').find('li.title').first().text());
// }
//
//
// function init_load_config(isMobile, callback) {
//     $.ajax({
//         url: globals.config_dir
//     }).done(function (data, textStatus, jqXHR) {
//         $(data).find('a').each(function (index) {
//             var href = $(this).attr('href');
//             if (href.match(/[.]json$/)) {
//                 config_files.push(globals.config_dir + '/' + href);
//             }
//         });
//         if ((config_files.length > 0) && config_files.indexOf(globals.config_file) < 0) {
//             globals.config_file = config_files[0];
//         }
//     }).fail(function (jqXHR, textStatus, errorThrown) {
//     }).always(function (data_jqXHR, textStatus, errorThrown) {
//         var file = query_string['config'];
//         if (file) {
//             globals.config_file = file;
//         } else {
//             var config_file = load_last_config_file();
//             if (config_file && config_files.indexOf(config_file) != -1) {
//                 globals.config_file = config_file;
//             }
//         }
//         if (config_files.indexOf(globals.config_file) == -1) {
//             config_files.push(globals.config_file);
//         }
//         callback(globals.config_file);
//         // load_config(globals.config_file, isMobile);
//     });
// }

function save_last_config_file(config_file) {
    var key = 'feeds.config_file';
    localStorage.setItem(key, config_file);
}

function load_last_config_file() {
    var key = 'feeds.config_file';
    return localStorage.getItem(key);
}
function save(config_file) {
    if (!config_file) {
        return;
    }
    save_layout(config_file);
    save_gui_config(config_file);
}

// function load(config_file) {
//     if (!config_file) {
//         return;
//     }
//     load_gui_config(config_file);
//     load_layout(config_file);
// }

function toggle_sortable(enable) {
    if (enable) {
        try {
            $('#container').sortable('enable');
        } catch (e) {
        }
        $('.panel').css('cursor', 'move');
    } else {
        try {
            $('#container').sortable('disable');
        } catch (e) {
        }
        $('.panel').css('cursor', 'auto');
    }
}
function reset(config_file) {
    var keys = [];
    for (var i = 0; i < localStorage.length; ++i) {
        keys.push(localStorage.key(i));
    }
    for (var i in keys) {
        var key = keys[i];
        var re = new RegExp('^feeds[.].*\\[' + config_file + '\\]');
        if (key.match(re)) {
            localStorage.removeItem(key);
        }
    }
}

function save_layout(config_file) {
    var key = 'feeds.layout[' + config_file + ']';
    var layout = $('#container > li').map(function () {
        var id = this.id;
        var feed = feeds[id];
        var layout = {id: id};
        if (feed.target.is('img') && feed.target.attr('src') && feed.target.is(':visible')) {
            layout.width = feed.target.width();
            layout.height = feed.target.height();
        }
        return layout;
    }).get();
    layout.push({
        id: 'container'
    });
    localStorage.setItem(key, JSON.stringify(layout));
}


function hex2rgb(hex, a) {
    var r = parseInt(hex.substr(1, 2), 16);
    var g = parseInt(hex.substr(3, 2), 16);
    var b = parseInt(hex.substr(5, 2), 16);
    if (a) {
        return "rgba(" + r + "," + g + "," + b + "," + a + ")";
    }
    return "rgb(" + r + "," + g + "," + b + ")";
}

function world_to_extent(world_array, width, height) {
    return [
        world_array[4],                             // min x
        world_array[5] + world_array[3] * height,   // min y
        world_array[4] + world_array[0] * width,    // max x
        world_array[5],                             // max y
    ];
}

// function add_gui_track_options(folder, config) {
//     var track_folder = folder.addFolder('track options');
//     $(track_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
//     track_folder.add(config, 'image').onFinishChange(function (value) {
//         var feed_name = get_feed_name(track_folder.domElement);
//         feeds[feed_name].set_background();
//     });
//     track_folder.add(config, 'extent').onFinishChange(function (value) {
//         var feed_name = get_feed_name(track_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.set_extent();
//     });
//     track_folder.add(config, 'scale', 1, 300).name('scale (%)').step(0.01).onChange(function (value) {
//         var feed_name = get_feed_name(track_folder.domElement);
//         if (value < feeds[feed_name].min_scale()) {
//             return;
//         }
//         feeds[feed_name].resize();
//     });
//     track_folder.add(config, 'trail').onChange(function (value) {
//         var feed_name = get_feed_name(track_folder.domElement);
//         if (!value) {
//             feeds[feed_name].remove_trail();
//         }
//     });
//     track_folder.add(config, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value) {
//         var feed_name = get_feed_name(track_folder.domElement);
//         feeds[feed_name].reset_draw_interval();
//     });
//     track_folder.add(config, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
//     track_folder.add(config, 'radius', 0.5, 20).step(0.1);
//     track_folder.addColor(config, 'fill');
//     track_folder.addColor(config, 'stroke');
//     track_folder.add(config, 'stroke_width', 0, 5).name('stroke width').step(0.5);
// }
//
// function add_gui_map_options(folder, config) {
//     var map_folder = folder.addFolder('map options');
//     $(map_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
//     map_folder.add(config, 'imagery_set', ['', 'Aerial', 'AerialWithLabels', 'Road']).onFinishChange(function (value) {
//         var feed_name = get_feed_name(map_folder.domElement);
//         var feed = feeds[feed_name];
//         if (value) {
//             feed.set_base_tile();
//         } else {
//             feed.set_base_layer();
//         }
//     });
//     map_folder.add(config, 'bing_maps_key').name('bing maps key').onFinishChange(function (value) {
//         var feed_name = get_feed_name(map_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.set_base_tile();
//     });
//     map_folder.add(config, 'image').onFinishChange(function (value) {
//         var feed_name = get_feed_name(map_folder.domElement);
//         feeds[feed_name].set_base_layer();
//     });
//     map_folder.add(config, 'extent').onFinishChange(function (value) {
//         var feed_name = get_feed_name(map_folder.domElement);
//         feeds[feed_name].set_base_layer();
//     });
//     map_folder.add(config, 'follow');
//     map_folder.add(config, 'trail');
//     map_folder.add(config, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value) {
//         var feed_name = get_feed_name(map_folder.domElement);
//         feeds[feed_name].reset_draw_interval();
//     });
//     map_folder.add(config, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
//     map_folder.add(config, 'radius', 0.5, 20).step(0.1);
//     map_folder.addColor(config, 'fill');
//     map_folder.addColor(config, 'stroke');
//     map_folder.add(config, 'stroke_width', 0, 5).name('stroke width').step(0.5);
// }
//
// function add_gui_grid_options(folder, config) {
//     var grid_folder = folder.addFolder('grid options');
//     grid_folder.add(config, 'show').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         value ? feed.add_grid() : feed.remove_grid();
//     });
//     grid_folder.add(config, 'axis_width', 0.5, 20).step(0.5).name('axis width').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     grid_folder.add(config, 'step_length', 0.5, 20).step(0.5).name('step length').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     grid_folder.add(config, 'x_offset', -100, 100).step(1).name('x offset (pixels)').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     grid_folder.add(config, 'y_offset', -100, 100).step(1).name('y offset (pixels)').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     var x_folder = grid_folder.addFolder('x-axis options');
//     x_folder.add(config.x, 'min', -100, 100).step(1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     x_folder.add(config.x, 'max', -100, 100).step(1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     x_folder.add(config.x, 'step', 0, 100).step(1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     x_folder.addColor(config.x, 'color').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     var y_folder = grid_folder.addFolder('y-axis options');
//     y_folder.add(config.y, 'min', -100, 100).step(1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     y_folder.add(config.y, 'max', -100, 100).step(1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     y_folder.add(config.y, 'step', 0, 100).step(1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     y_folder.addColor(config.y, 'color').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     var grid_lines_folder = grid_folder.addFolder('grid line options');
//     grid_lines_folder.add(config.grid_lines, 'show').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     grid_lines_folder.addColor(config.grid_lines, 'color').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     grid_lines_folder.add(config.grid_lines, 'width', 0.5, 10).step(0.1).onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     var labels_folder = grid_folder.addFolder('label options');
//     labels_folder.add(config.labels, 'show').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     labels_folder.addColor(config.labels, 'color').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     labels_folder.add(config.labels, 'font').onChange(function (value) {
//         var feed_name = get_feed_name(grid_folder.domElement);
//         var feed = feeds[feed_name];
//         feed.draw_grid();
//     });
//     $(grid_folder.domElement).closest('li.folder').find('li.title').addClass('subfolder');
// }
//
// function load_feed_items(frontend_config, feeds, path) {
//     if (path.length != 0)
//         path = path + "/";
//     for (var feed_name in feeds) {
//         var config = feeds[feed_name];
//         if (Array.isArray(config))
//             load_feed_array(frontend_config, config, path + feed_name);
//         else
//             add_new_feed(frontend_config, config, path + feed_name);
//     }
// }
//
// function load_feed_array(frontend_config, feeds, path) {
//     for (var i = 0; i < feeds.length; i++) {
//         load_feed_items(frontend_config, feeds[i], path + "[" + i + "]");
//     }
// }
//
// function add_new_feed(frontend_config, config, feed_path) {
//     var grid_types = ['image', 'stream', 'track'];
// //         var feed_name=feed_path.replace(/\[/g,"\\[").replace(/\]/g,"\\]").replace(/\//g,"\\/");
//     var feed_name = feed_path_to_id(feed_path);
//     if (!('type' in config)) {
//         config.type = 'image';
//     }
//     if (grid_types.indexOf(config.type) >= 0) {
//         config.grid = new GridOptions(config.grid);
//     }
//     if (config.type == 'stream') {
//         config.show = true;
//         if (!('stream' in config)) {
//             config.stream = {autoplay: false};
//         }
//         if (!('url' in config)) {
//             var xpath = config.xpath || feed_path;
//             config.url = frontend_config.websocket + '?xpath=' + xpath + '&data_uri=true';
//         }
//     } else {
//         config.view = Feed.views[0];
//         if (!('refresh' in config)) {
//             config.refresh = {};
//         }
//         if (!('auto' in config.refresh)) {
//             config.refresh.auto = 'interval' in config.refresh && config.refresh.interval > 0;
//         }
//         if (!('interval' in config.refresh)) {
//             config.refresh.interval = 2;
//         }
//         if (!('url' in config)) {
//             config.url = frontend_config.host + '/' + feed_path;
//         }
//     }
//     if (config.type == 'text') {
//         if (!('text' in config)) {
//             config.text = {};
//         }
//         if (!('show_items' in config.text)) {
//             config.text.show_items = 1;
//         }
//     } else if (config.type == 'csv' || config.type == 'csv-table') {
//         if (!('csv' in config)) {
//             config.csv = {};
//         }
//         if (!('show_items' in config.csv)) {
//             config.csv.show_items = 1;
//         }
//         if (!('fields' in config.csv)) {
//             config.csv.fields = '';
//         }
//         if (!('min' in config.csv)) {
//             config.csv.min = '';
//         }
//         if (!('max' in config.csv)) {
//             config.csv.max = '';
//         }
//         if (!('min_color' in config.csv) || !config.csv.min_color) {
//             config.csv.min_color = 'orange';
//         }
//         if (!('max_color' in config.csv) || !config.csv.max_color) {
//             config.csv.max_color = 'red';
//         }
//         if (!('threshold_alert' in config.csv)) {
//             config.csv.threshold_alert = false;
//         }
//     } else if (config.type == 'graph') {
//         if (!('graph' in config)) {
//             config.graph = {};
//         }
//         if (!('min' in config.graph)) {
//             config.graph.min = 0;
//         }
//         if (!('max' in config.graph)) {
//             config.graph.max = 100;
//         }
//         if (!('thresholds' in config.graph)) {
//             config.graph.thresholds = [];
//         }
//         config.graph.thresholds.sort(function (a, b) {
//             return a.value - b.value;
//         });
//     } else if (config.type == 'track') {
//         config.track = new TrackOptions(config.track);
//     } else if (config.type == 'map') {
//         config.map = new MapOptions(config.map, frontend_config);
//     }
//     if (!('alert' in config)) {
//         config.alert = true;
//     }
//     var feed = create_feed(config.type, feed_name, feed_path, config);
//     var folder = gui.addFolder(feed_path);
//     folder.close();
//     folder.add(feed.config, 'url').onFinishChange(function (value) {
//         var feed_name = get_feed_name(this.domElement);
//         feeds[feed_name].reset();
//     });
//     if (config.type != 'stream') {
//         folder.add(feed.config, 'view', Feed.views).onFinishChange(function (value) {
//             var feed_name = get_feed_name(this.domElement);
//             feeds[feed_name].update_view();
//             if (value == 'hide') {
//                 gui.setProperty('auto', false, feed_name);
//             }
//         });
//         folder.add(feed.config.refresh, 'auto').name("auto refresh").onFinishChange(function (value) {
//             var feed_name = get_feed_name(this.domElement);
//             var feed = feeds[feed_name];
//             if (value && feed.config.view == 'hide') {
//                 gui.setProperty('view', 'show', feed_name);
//             }
//             feed.reset();
//         });
//         folder.add(feed.config.refresh, 'interval', 0, 90).name("refresh interval").step(1).onFinishChange(function (value) {
//             var feed_name = get_feed_name(this.domElement);
//             feeds[feed_name].reset();
//         });
//         folder.add(feed.config, 'alert').name('feed alert').onFinishChange(function (value) {
//             var feed_name = get_feed_name(this.domElement);
//             if (!value) {
//                 feeds[feed_name].alert(false);
//             }
//         });
//         if (config.type == 'text') {
//             folder.add(feed.config.text, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].draw();
//             });
//         }
//         if (config.type == 'csv' || config.type == 'csv-table') {
//             folder.add(feed.config.csv, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].draw();
//             });
//             folder.add(feed.config.csv, 'fields').onChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].init_fields();
//             });
//             folder.add(feed.config.csv, 'min').onFinishChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].init_ranges();
//             });
//             folder.add(feed.config.csv, 'max').onFinishChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].init_ranges();
//             });
//             folder.add(feed.config.csv, 'threshold_alert').name('threshold alert');
//         }
//         if (config.type == 'graph') {
//             folder.add(feed.config.graph, 'max').onFinishChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].set_labels();
//             });
//             folder.add(feed.config.graph, 'min').onFinishChange(function (value) {
//                 var feed_name = get_feed_name(this.domElement);
//                 feeds[feed_name].set_labels();
//             });
//         }
//         if (config.type == 'track') {
//             add_gui_track_options(folder, feed.config.track);
//         }
//         if (config.type == 'map') {
//             add_gui_map_options(folder, feed.config.map);
//         }
//     } else {
//         folder.add(feed.config, 'show').onFinishChange(function (value) {
//             var feed_name = get_feed_name(this.domElement);
//             feeds[feed_name].toggle_show();
//             if (!value) {
//                 gui.setProperty('auto', false, feed_name);
//             }
//         });
//     }
//     if (grid_types.indexOf(config.type) >= 0) {
//         add_gui_grid_options(folder, feed.config.grid);
//     }
//     feeds[feed_name] = feed;
// }
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
//

