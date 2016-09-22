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
    save: function() {
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

var feed_path_to_id = function(feed_path) {
    return feed_path.replace(/\[/g,"_").replace(/\]/g,"_").replace(/\//g,"_");
}

var add_poll_body = function (feed_name, element, popup_div) {
    var id = '#' + feed_path_to_id(feed_name);
    var class_str = globals.isMobile ? '' : ' transparent';
    $(id).append(
        '<h3>' + feed_name +
        '  <button class="panel-refresh hideable' + class_str + '" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: auto refresh"><span class="status text-muted glyphicon glyphicon-refresh"></span></button>' +
        '  <button class="panel-settings hideable' + class_str + '" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button>' +
        '  <button class="panel-compact hideable' + class_str + '" title="compact"><span class="text-muted glyphicon glyphicon-resize-small"></span></button>' +
        '</h3>' +
        '<div class="panel-body">' +
        '  <time class="timestring time small">&nbsp;</time>' +
        '  <time class="timeago time small">&nbsp;</time>' +
        element +
        '</div>'
    );
    $(id).append(popup_div);
    if (globals.isMobile) {
        $("#popup" + feed_name).popup();
    }

};
var add_stream_body = function (feed_name, element) {
    var id = '#' + feed_path_to_id(feed_name);
    var class_str = globals.isMobile ? '' : ' transparent';
    $(id).append(
        '<h3>' + feed_name +
        '  <button class="panel-stream-control hideable' + class_str + '" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: start/stop"><span class="status text-muted glyphicon glyphicon-stop"></span></button>' +
        '  <button class="panel-settings hideable ' + class_str + '" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button></h3>' +
        '<div class="panel-body">' +
        element +
        '</div>'
    );
};


var add_panel = function (feed_name, feed_path) {
    var class_str = globals.isMobile ? '' : ' transparent';
    $('#container').append(
        '<li id="' + feed_name + '" class="panel" data-name="'+feed_path+'" >' +
        '  <button type="button" class="panel-close hideable text-muted pull-right' + class_str + '" title="close"><span>&times;</span></button>' +
        '</li>'
    );
};


function load_gui_config(config_file) {
    var key = 'feeds.gui.config[' + config_file + ']';
    var config = JSON.parse(localStorage.getItem(key));
    if (!config) {
        return false;
    }
    gui.setProperty('alert_beep', config['globals'].alert_beep, 'globals');
}

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

function load(config_file) {
    if (!config_file) {
        return;
    }
    load_gui_config(config_file);
    load_layout(config_file);
}

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


function load_layout(config_file) {
    var key = 'feeds.layout[' + config_file + ']';
    var layout = JSON.parse(localStorage.getItem(key));
    if (!layout) {
        return;
    }
    layout.reverse();
    layout.forEach(function (value, index) {

        var container = $('#container');
        if (value.id === 'container') {
            container.width(value.width);
            return;
        }
        var panel = $('#' + value.id);
        if (!panel.length) {
            return;
        }
        container.prepend(panel);
        var feed = feeds[value.id];
        if (feed != undefined) {
            if ('width' in value) {
                feed.target.width(value.width);
            }
            if ('height' in value) {
                feed.target.height(value.height);
            }
        }
    });
}

function hex2rgb(hex, a) {
    var r = parseInt(hex.substr(1,2), 16);
    var g = parseInt(hex.substr(3,2), 16);
    var b = parseInt(hex.substr(5,2), 16);
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

