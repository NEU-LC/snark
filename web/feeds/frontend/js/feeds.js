var Feed = function(feed_name, config) {
    this.feed_name = feed_name;
    this.config = config;
    this.id = '#' + this.feed_name;
    this.el = $(this.id);
    this.compact_icon = $(this.id + ' .panel-compact span');
    this.body = $(this.id + ' .panel-body');
    this.status = $(this.id + ' .status');
    this.timestring = $(this.id + ' .timestring').hide();
    this.timeago = $(this.id + ' .timeago').timeago();
    this.target = $(this.id + ' .target');
    this.interval = null;
    this.refresh_time = null;
    this.show = true;
};
Feed.views = ['show', 'compact', 'hide'];
Feed.prototype.init = function() {
}
Feed.prototype.reset = function() {
    if (this.config.refresh.auto) {
        this.refresh();
    } else {
        this.clear_interval();
    }
}
Feed.prototype.set_interval = function() {
    this.interval = setInterval(this.preload.bind(this), this.config.refresh.interval * 1000);
    this.status.removeClass('text-muted').addClass('text-success');
}
Feed.prototype.clear_interval = function() {
    clearInterval(this.interval);
    delete pending[this.feed_name];
    this.status.removeClass('text-success').addClass('text-muted');
}
Feed.prototype.refresh = function() {
    this.clear_interval();
    this.preload();
    if (this.config.refresh.auto) {
        this.set_interval();
    }
}
Feed.prototype.update_view = function() {
    if (this.config.view == 'show') {
        this.el.show();
        this.body.show();
        this.compact_icon.removeClass('glyphicon-resize-full').addClass('glyphicon-resize-small');
    } else if (this.config.view == 'compact') {
        this.el.show();
        this.body.hide();
        this.compact_icon.removeClass('glyphicon-resize-small').addClass('glyphicon-resize-full');
    } else if (this.config.view == 'hide') {
        this.el.hide();
    }
}
Feed.prototype.preload = function() {
    if (pending[this.feed_name]) {
        return;
    }
    pending[this.feed_name] = true;
    this.refresh_time = new Date();
    this.status.fadeTo(1000, 0.4);
    this.load();
}
Feed.prototype.update_time = function() {
    var timestring = this.refresh_time.toString();
    var timezone = timestring.match(/\((.*)\)$/);
    if (timezone.length == 2 && timezone[1].match(/ /)) {
        var short_timezone = timezone[1].split(' ').map(function(word, index) { return word[0]; }).join('');
        timestring = timestring.replace(/\(.*\)$/, '(' + short_timezone + ')');
    }
    this.timestring.text(timestring);
    this.timeago.attr('datetime', this.refresh_time.toISOString()).timeago('updateFromDOM');
}
Feed.prototype.onload = function(data) {
    this.update_time();
    this.status.finish().fadeTo(0, 1);
    if (this.config.alert) {
        this.alert(!data || !data.length);
    }
    this.onload_(data);
    delete pending[this.feed_name];
}
Feed.prototype.onerror = function() {
    this.update_time();
    this.status.finish().fadeTo(0, 1);
    this.onload_();
    delete pending[this.feed_name];
    if (this.config.alert) {
        this.alert(true);
    }
}
Feed.prototype.get_url = function() {
    var url = this.config.url;
    return url + (url.indexOf('?') < 0 ? '?q=' : '&q=') + Math.random();
}
Feed.prototype.alert = function(on) {
    var gui_folder = $(gui.__folders[this.feed_name].__ul);
    if (on) {
        this.el.addClass('panel-alert');
        gui_folder.find('.title').addClass('panel-alert');
        if (globals.alert_beep) {
            globals.beep();
        }
    } else {
        this.el.removeClass('panel-alert');
        gui_folder.find('.title').removeClass('panel-alert');
    }
}

var ImageFeed = function(feed_name, config) {
    this.base = Feed;
    this.base(feed_name, config);
    this.loader = new Image();
    this.loader.feed = this;
    this.loader.onload = function() { this.feed.onload(this.src); }
    this.loader.onerror = function() { this.feed.onerror(); }
}
ImageFeed.prototype = new Feed;
ImageFeed.prototype.load = function() {
    this.loader.src = this.get_url();
}
ImageFeed.prototype.onload_ = function(data) {
    this.target.attr('src', data);
    if (!this.is_resizable) {
        this.target.resizable({
            aspectRatio: true,
            autoHide: true,
            minWidth: 250,
            minHeight: 250,
        });
        this.is_resizable = true;
    }
}

var ImageStreamFeed = function(feed_name, config) {
    this.feed_name = feed_name;
    this.config = config;
    this.id = '#' + this.feed_name;
    this.el = $(this.id);
    this.target = $(this.id + ' .target');
    this.control = $(this.id + ' .panel-stream-control span');
    this.target.on('load', function() {
        var id = $(this).closest('li').attr('id');
        var feed = feeds[id];
        if (!feed.is_resizable) {
            feed.target.resizable({
                aspectRatio: true,
                autoHide: true,
                minWidth: 250,
                minHeight: 250,
            });
            feed.is_resizable = true;
        }
    });
}
ImageStreamFeed.prototype.onload = function(data) {
    if (data.substr(0, 5) === 'data:') {
        this.target.attr('src', data);
    } else {
        console.log(data);
    }
}
ImageStreamFeed.prototype.reset = function(options) {
    if (this.is_open()) {
        this.ws.onclose = null;
        this.ws.close();
    }
    var url = this.config.url;
    for (var key in options) {
        url += url.indexOf('?') < 0 ? '?' : '&';
        url += key + '=' + options[key];
    }
    this.ws = new WebSocket(url);
    this.ws.feed = this;
    this.ws.onopen = function() {
        console.log('ws open: ' + this.feed.feed_name + ' ' + this.url);
        this.feed.started();

    }
    this.ws.onclose = function() {
        console.log('ws close: ' + this.feed.feed_name);
        this.feed.stopped();
    }
    this.ws.onmessage = function(event) {
        this.feed.onload(event.data);
    }
}
ImageStreamFeed.prototype.is_open = function() {
    return this.ws && this.ws.readyState === WebSocket.OPEN;
}
ImageStreamFeed.prototype.toggle = function() {
    if (this.is_open()) {
        this.stop();
    } else {
        this.reset();
    }
}
ImageStreamFeed.prototype.toggle_show = function() {
    if (this.config.show) {
        this.el.show();
    } else {
        this.el.hide();
        this.stop();
    }
}
ImageStreamFeed.prototype.stop = function() {
    if (this.ws) {
        this.ws.close();
    }
}
ImageStreamFeed.prototype.started = function() {
    this.control.removeClass('text-muted').addClass('text-success');
}

ImageStreamFeed.prototype.stopped = function() {
    this.control.removeClass('text-success').addClass('text-muted');
}
ImageStreamFeed.prototype.refresh = function() {
    if (!this.is_open()) {
        this.reset({ count: 1 });
    }
}
ImageStreamFeed.prototype.start = function() {
    this.reset(this.config.stream.autoplay ? {} : { count: 1 });
}

var TextFeed = function(feed_name, config) {
    this.base = Feed;
    this.base(feed_name, config);
}
TextFeed.prototype = new Feed;
TextFeed.prototype.load = function() {
    $.ajax({
        context: this,
        url: this.get_url()
    }).done(function(data, textStatus, jqXHR) {
        this.onload(data);
    }).fail(function(jqXHR, textStatus, errorThrown) {
        this.onerror();
    });
}
TextFeed.prototype.onload_ = function(data) {
    if (data && data.length && data[data.length - 1] == '\n') {
        data = data.substring(0, data.length - 1);
    }
    data = data ? data : '&nbsp;';
    this.target.append('<tr><td><pre>' + data + '</pre></td></tr>');
    this.draw();
}
TextFeed.prototype.draw = function() {
    while (this.target.find('tbody tr').length > this.config.text.show_items ) {
        this.target.find('tbody tr').first().remove();
    }
}

var CsvFeed = function(feed_name, config) {
    this.base = Feed;
    this.base(feed_name, config);
    this.init();
}
CsvFeed.prototype = new TextFeed;
CsvFeed.prototype.init = function() {
    this.init_fields();
    this.init_styles();
}
CsvFeed.prototype.init_fields = function() {
    this.target.find('thead').empty();
    if (this.config.csv.fields) {
        var tr = $('<tr>');
        if (this.config.type == 'csv-table') {
            var fields = this.config.csv.fields.split(',');
            for (var i in fields) {
                $('<th class="text-center">')
                    .text(fields[i])
                    .appendTo(tr);
            }
        } else {
            $('<th>')
                .text(this.config.csv.fields)
                .appendTo(tr);
        }
        this.target.find('thead').append(tr);
    }
}
CsvFeed.prototype.init_styles = function() {
    this.el.remove('style');
    var css = '';
    this.min = {};
    var min = this.config.csv.min.split(',');
    for (var i in min) {
        if (min[i] && !isNaN(min[i])) {
            this.min[i] = Number(min[i]);
            css += ' .' + this.class_id(i, '-min') + ' { color: ' + this.config.csv.min_color + ' }\n'
        }
    }
    this.max = {};
    var max = this.config.csv.max.split(',');
    for (var i in max) {
        if (max[i] && !isNaN(max[i])) {
            this.max[i] = Number(max[i]);
            css += ' .' + this.class_id(i, '-max') + ' { color: ' + this.config.csv.max_color + ' }\n'
        }
    }
    $('<style>')
        .attr('type', 'text/css')
        .html(css)
        .prependTo(this.el);
}
CsvFeed.prototype.class_id = function(index, suffix) {
    return 'style-' + this.feed_name + '-' + index + suffix;
}
CsvFeed.prototype.onload_ = function(data) {
    if (data) {
        data = data.split('\n').map(function(value, index) { return value.split(','); });
        var out_of_range = false;
        for (var i in data) {
            for (var j in data[i]) {
                var value = data[i][j];
                if (value && !isNaN(value)) {
                    var class_id = '';
                    if (j in this.min && Number(value) < this.min[j]) {
                        out_of_range = true;
                        class_id = this.class_id(j, '-min');
                    } else if (j in this.max && Number(value) > this.max[j]) {
                        out_of_range = true;
                        class_id = this.class_id(j, '-max');
                    }
                    if (class_id) {
                        data[i][j] = '<span class="' + class_id + '">' + value + '</span>';
                    }
                }
            }
        }
        this.alert(this.config.csv.threshold_alert && out_of_range);
    } else {
        data = '&nbsp;'
    }
    if (this.config.type == 'csv-table') {
        if (typeof data === 'object') {
            for (var i in data) {
                if (data[i].length == 1 && !data[i][0]) { continue; }
                var tr = $('<tr class="text-right">');
                for (var j in data[i]) {
                    tr.append('<td><pre>' + data[i][j] + '</pre></td>');
                }
                this.target.append(tr);
            }
        } else {
            this.target.append('<tr><td colspan="' + this.target.find('thead th').length + '"><pre>' + data + '</pre></td></tr>');
        }
    } else {
        if (typeof data === 'object') { data = data.map(function(value, index) { return value.join(','); }).join('\n'); }
        if (data && data.length && data[data.length - 1] == '\n') {
            data = data.substring(0, data.length - 1);
        }
        this.target.append('<tr><td><pre>' + data + '</pre></td></tr>');
    }
    this.draw();
}
CsvFeed.prototype.draw = function() {
    while (this.target.find('tbody tr').length > this.config.csv.show_items ) {
        this.target.find('tbody tr').first().remove();
    }
}

var GraphFeed = function(feed_name, config) {
    this.base = Feed;
    this.base(feed_name, config);
    this.default_threshold = { value: this.config.graph.max, color: '#5cb85c' }
    this.default_exceeded_threshold = { value: Number.MAX_VALUE, color: '#d9534f', alert: true }
    this.text = $(this.id + ' .graph-text');
    this.bars = $(this.id + ' .graph-bars');
    this.bars.append('<div class="graph-bar-col"><span class="graph-bar"></span></div>');
    this.bars_width = Number($('.graph-bars').css('width').replace('px', ''));
    this.bar_width = Number($('.graph-bar-col').css('width').replace('px', ''));
    this.bar_height = Number($('.graph-bar-col').css('height').replace('px', ''));
    this.bar_count = Math.floor(this.bars_width / this.bar_width);
    if (!isFinite(this.bar_count)) {
        throw 'invalid graph bar styles; calculated bar count: ' + this.bar_count;
    }
    for (var i = 1; i < this.bar_count; ++i ) {
        this.bars.append('<div class="graph-bar-col"><span class="graph-bar"></span></div>');
    }
    var _this = this;
    this.bars.find('.graph-bar-col').each(function(i, e) {
        _this.config.graph.thresholds.forEach(function(threshold, index) {
            if (threshold.value > _this.config.graph.max) {
                return;
            }
            var span = $('<span class="graph-threshold"></span>');
            span.css('height', _this.get_bar_height(threshold.value) + 1 + 'px');
            span.css('border-bottom-color', threshold.color);
            span.appendTo(e);
        });
    });
    $('.graph-bar-col').tooltip();
}
GraphFeed.prototype = new Feed;
GraphFeed.prototype.load = function() {
    $.ajax({
        context: this,
        url: this.get_url()
    }).done(function(data, textStatus, jqXHR) {
        this.onload(data);
    }).fail(function(jqXHR, textStatus, errorThrown) {
        this.onerror();
    });
}
GraphFeed.prototype.onload_ = function(data) {
    var text = data ? data.replace("\n", '') : 'N/A';
    if (this.config.graph.units) {
        text += ' ' + this.config.graph.units;
    }
    this.text.html(text);
    var bar = this.bars.children().first();
    this.bars.append(bar);
    var value = isNaN(data) ? this.config.graph.min : Number(data);
    var height = this.get_bar_height(value);
    var threshold = this.get_threshold(value);
    bar.find('.graph-bar').css('height', height);
    bar.css('background', threshold.color);
    if (this.config.alert) {
        this.alert(threshold.alert);
    }
    bar.data('bs.tooltip').options.title = text + ' @ ' + this.refresh_time;
    bar.find('.graph-threshold').each(function(index, value) {
        var e = $(value);
        if (Number(e.css('height').replace('px', '')) >= height) {
            e.hide();
        } else {
            e.show();
        }
    });
}
GraphFeed.prototype.get_bar_height = function(value) {
    var scale = (value - this.config.graph.min) / (this.config.graph.max - this.config.graph.min);
    if (scale > 1) {
        scale = 1;
    } else if (scale < 0) {
        scale = 0;
    }
    return this.bar_height - scale * this.bar_height;
}
GraphFeed.prototype.get_threshold = function(value) {
    if (!this.config.graph.thresholds.length) {
        return this.default_threshold;
    }
    for (var i in this.config.graph.thresholds) {
        var threshold = this.config.graph.thresholds[i];
        if (value <= threshold.value) {
            return threshold
        }
    }
    return this.default_exceeded_threshold;
}

var add_panel = function(feed_name) {
    $('#container').append(
        '<li id="' + feed_name + '" class="panel">' +
        '  <button type="button" class="panel-close hideable text-muted transparent pull-right" title="close"><span>&times;</span></button>' +
        '</li>'
    );
}

var add_poll_body = function(feed_name, element) {
    var id = '#' + feed_name;
    $(id).append(
        '<h3>' + feed_name + 
        '  <button class="panel-refresh" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: auto refresh"><span class="status text-muted glyphicon glyphicon-stop"></span></button>' +
        '  <button class="panel-settings hideable transparent" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button>' +
        '  <button class="panel-compact hideable transparent" title="compact"><span class="text-muted glyphicon glyphicon-resize-small"></span></button>' +
        '</h3>' +
        '<div class="panel-body">' +
        '  <time class="timestring time small">&nbsp;</time>' +
        '  <time class="timeago time small">&nbsp;</time>' +
           element +
        '</div>'
    );
}

var add_stream_body = function(feed_name, element) {
    var id = '#' + feed_name;
    $(id).append(
        '<h3>' + feed_name + 
        '  <button class="panel-stream-control" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: start/stop"><span class="status text-muted glyphicon glyphicon-stop"></span></button>' +
        '  <button class="panel-settings hideable transparent" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button></h3>' +
        '<div class="panel-body">' +
           element +
        '</div>'
    );
}

var create_feed = function(type, feed_name, config) {
    add_panel(feed_name);
    if (type == 'image') {
        add_poll_body(feed_name, '<img class="target"/>');
        return new ImageFeed(feed_name, config);
    } else if (type == 'text' || type == 'csv' || type == 'csv-table') {
        add_poll_body(feed_name, '<table class="target"><thead></thead></table>');
        return type == 'text' ? new TextFeed(feed_name, config) : new CsvFeed(feed_name, config);
    } else if (type == 'graph') {
        add_poll_body(feed_name, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-bars"></div></div>');
        return new GraphFeed(feed_name, config);
    } else if (type == 'stream') {
        add_stream_body(feed_name, '<img class="target"/>');
        return new ImageStreamFeed(feed_name, config);
    }
    throw 'unrecognised feed type: ' + type;
}

dat.GUI.prototype.setProperty = function(property, value, opt_folder_name) {
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
}
dat.GUI.prototype.toggleProperty = function(property, opt_folder_name) {
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
}

var globals = {
    refresh: function() {
        $.each(feeds, function(index, feed) {
            feed.refresh();
        });
    },
    start_auto_refresh: function() {
        $.each(feeds, function(index, feed) {
            if (feed.show) {
                gui.setProperty('auto', true, feed.feed_name);
            }
        });
    },
    stop_auto_refresh: function() {
        $.each(feeds, function(index, feed) {
            gui.setProperty('auto', false, feed.feed_name);
        });
    },
    start_streams: function() {
        $.each(feeds, function(index, feed) {
            if (feed.config.type === 'stream') {
                feed.reset();
            }
        });
    },
    stop_streams: function() {
        $.each(feeds, function(index, feed) {
            if (feed.config.type === 'stream') {
                feed.stop();
            }
        });
    },
    start: function() {
        this.start_auto_refresh();
        this.start_streams();
    },
    stop: function() {
        this.stop_auto_refresh();
        this.stop_streams();
    },
    show: function() {
        $.each(feeds, function(index, feed) {
            if (feed.config.type == 'stream') {
                gui.setProperty('show', true, feed.feed_name);
            } else {
                gui.setProperty('view', 'show', feed.feed_name);
            }
        });
    },
    compact: function() {
        $.each(feeds, function(index, feed) {
            if (feed.config.type != 'stream') {
                gui.setProperty('view', 'compact', feed.feed_name);
            }
        });
    },
    hide: function() {
        $.each(feeds, function(index, feed) {
            if (feed.config.type == 'stream') {
                gui.setProperty('show', false, feed.feed_name);
            } else {
                gui.setProperty('view', 'hide', feed.feed_name);
            }
        });
    },
    reset: function() {
        reset(globals.config_file);
        $('#container').width(container_width);
        load_config(globals.config_file);
    },
    enable_alerting: function() {
        $.each(feeds, function(index, feed) {
            gui.setProperty('alert', true, feed.feed_name);
            gui.setProperty('threshold_alert', true, feed.feed_name);
        });
    },
    disable_alerting: function() {
        $.each(feeds, function(index, feed) {
            gui.setProperty('alert', false, feed.feed_name);
            gui.setProperty('threshold_alert', false, feed.feed_name);
        });
    },
    clear_alerts: function() {
        $.each(feeds, function(index, feed) {
            feed.alert(false);
        });
    },
    config_dir: 'config',
    config_file: 'config/web.frontend.json',
    alert_beep: true,
    beep: function() {
        var oscillator = audio_context.createOscillator();
        oscillator.connect(audio_context.destination);
        oscillator.start(0);
        oscillator.stop(audio_context.currentTime + 0.2);
    }
}

var gui;
var feeds = {};
var pending = {};
var config_files = [];
var audio_context = new AudioContext();
var current_config_file;
var container_width;

function initialize(frontend_config) {
    current_config_file = globals.config_file;
    globals.stop();
    $('#container').empty();
    try { $('#container').sortable('destroy'); } catch (e) { }
    try { $('#container').resizable('destroy'); } catch (e) { }
    feeds = {};
    pending = {};
    if (gui) {
        gui.destroy();
    }
    gui = new dat.GUI({
        width: 500,
    });
    gui.add(globals, 'config_file', config_files).name('config file').onFinishChange(function(value) {
        save(current_config_file);
        load_config(globals.config_file);
    });
    var folder = gui.addFolder('globals');
    folder.add(globals, "refresh");
    folder.add(globals, "start");
    folder.add(globals, "stop");
    folder.add(globals, "show");
    folder.add(globals, "compact");
    folder.add(globals, "hide");
    folder.add(globals, "reset");
    folder.add(globals, "enable_alerting").name("enable alerting");
    folder.add(globals, "disable_alerting").name("disable alerting");
    folder.add(globals, "clear_alerts").name("clear alerts");
    folder.add(globals, "alert_beep").name("alert beep");

    for (var feed_name in frontend_config.feeds) {
        var config = frontend_config.feeds[feed_name];
        if (!('type' in config)) { config.type = 'image'; }
        if (config.type == 'stream') {
            config.show = true;
            if (!('stream' in config)) { config.stream = { autoplay: false }; }
            if (!('url' in config)) {
                var xpath = config.xpath || feed_name;
                config.url = frontend_config.websocket + '?xpath=' + xpath + '&data_uri=true';
            }
        } else {
            config.view = Feed.views[0];
            if (!('refresh' in config)) { config.refresh = {}; }
            if (!('auto' in config.refresh)) { config.refresh.auto = 'interval' in config.refresh && config.refresh.interval > 0; }
            if (!('interval' in config.refresh)) { config.refresh.interval = 2; }
            if (!('url' in config)) { config.url = frontend_config.host + '/' + feed_name; }
        }
        if (config.type == 'text') {
            if (!('text' in config)) { config.text = {}; }
            if (!('show_items' in config.text)) { config.text.show_items = 1; }
        } else if (config.type == 'csv' || config.type == 'csv-table') {
            if (!('csv' in config)) { config.csv = {}; }
            if (!('show_items' in config.csv)) { config.csv.show_items = 1; }
            if (!('fields' in config.csv)) { config.csv.fields = ''; }
            if (!('min' in config.csv)) { config.csv.min = ''; }
            if (!('max' in config.csv)) { config.csv.max = ''; }
            if (!('min_color' in config.csv) || !config.csv.min_color) { config.csv.min_color = 'orange'; }
            if (!('max_color' in config.csv) || !config.csv.max_color) { config.csv.max_color = 'red'; }
            if (!('threshold_alert' in config.csv)) { config.csv.threshold_alert = false; }
        } else if (config.type == 'graph') {
            if (!('graph' in config)) { config.graph = {}; }
            if (!('min' in config.graph)) { config.graph.min = 0; }
            if (!('max' in config.graph)) { config.graph.max = 100; }
            if (!('thresholds' in config.graph)) { config.graph.thresholds = []; }
            config.graph.thresholds.sort(function(a,b) { return a.value - b.value; });
        }
        if (!('alert' in config)) { config.alert = true; }
        var feed = create_feed(config.type, feed_name, config);
        var folder = gui.addFolder(feed_name);
        folder.close();
        folder.add(feed.config, 'url').onFinishChange(function(value) {
            var feed_name = $(this.__gui.__ul).find('li.title').text();
            feeds[feed_name].reset();
        });
        if (config.type != 'stream') {
            folder.add(feed.config, 'view', Feed.views).onFinishChange(function(value) {
                var feed_name = $(this.__gui.__ul).find('li.title').text();
                feeds[feed_name].update_view();
                if (value == 'hide') {
                    gui.setProperty('auto', false, feed_name);
                }
            });
            folder.add(feed.config.refresh, 'auto').name("auto refresh").onFinishChange(function(value) {
                var feed_name = $(this.__gui.__ul).find('li.title').text();
                var feed = feeds[feed_name];
                if (value && feed.config.view == 'hide') {
                    gui.setProperty('view', 'show', feed_name);
                }
                feed.reset();
            });
            folder.add(feed.config.refresh, 'interval', 0, 90).name("refresh interval").step(1).onFinishChange(function(value) {
                var feed_name = $(this.__gui.__ul).find('li.title').text();
                if (value == 0) {
                    gui.setProperty('auto', false, feed_name);
                } else {
                    feeds[feed_name].reset();
                }
            });
            if (config.type == 'text') {
                folder.add(feed.config.text, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function(value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].draw();
                });
            }
            if (config.type == 'csv' || config.type == 'csv-table') {
                folder.add(feed.config.csv, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function(value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].draw();
                });
                folder.add(feed.config.csv, 'fields').onChange(function(value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].init_fields();
                });
                folder.add(feed.config.csv, 'min').onFinishChange(function(value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].init_styles();
                });
                folder.add(feed.config.csv, 'max').onFinishChange(function(value) {
                    var feed_name = $(this.__gui.__ul).find('li.title').text();
                    feeds[feed_name].init_styles();
                });
                folder.add(feed.config.csv, 'threshold_alert').name('threshold alert');
            }
            folder.add(feed.config, 'alert').name('feed alert').onFinishChange(function(value) {
                var feed_name = $(this.__gui.__ul).find('li.title').text();
                if (!value) {
                    feeds[feed_name].alert(false);
                }
            });
        } else {
            folder.add(feed.config, 'show').onFinishChange(function(value) {
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
        start: function(event, ui) {
            ui.placeholder.width(ui.item.width());
            ui.placeholder.height(ui.item.height());
        }
    }).resizable({
        handles: 'e',
    });

    $('.panel').on('mouseover', function() {
        $(this).find('.hideable').removeClass('transparent');
        $(this).find('div button').addClass('button-shadow');
    })
    $('.panel').on('mouseout', function() {
        $(this).find('.hideable').addClass('transparent');
        $(this).find('div button').removeClass('button-shadow');
    })
    $('.panel button').tooltip({ html: true });
    $('.panel-refresh').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        if (event.shiftKey) {
            gui.toggleProperty('auto', id);
        } else {
            feeds[id].refresh();
        }
    });
    $('.panel-stream-control').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        if (event.shiftKey) {
            feeds[id].toggle();
        } else {
            feeds[id].refresh();
        }
    });
    $('.panel-settings').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        $.each(gui.__folders, function(index, folder) {
            folder.close();
        });
        gui.__folders[id].open();
        gui.open();
    });
    $('.panel-close').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        var feed = feeds[id];
        if (feed.config.type == 'stream') {
            gui.setProperty('show', false, feed.feed_name);
        } else {
            gui.setProperty('view', 'hide', feed.feed_name);
        }
    });
    $('.panel-compact').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        var feed = feeds[id];
        if (feed.config.type != 'stream') {
            gui.setProperty('view', feed.config.view == 'compact' ? 'show' : 'compact', feed.feed_name);
        }
    });
    $('.timeago').on('mouseenter', function(e) {
        var id = $(this).closest('li').attr('id');
        var feed = feeds[id];
        feed.timeago.hide();
        feed.timestring.show();
    });
    $('.timestring').on('mouseleave', function(e) {
        var id = $(this).closest('li').attr('id');
        var feed = feeds[id];
        feed.timestring.hide();
        feed.timeago.show();
    });

    $(document).on('keydown', function(event) { if (event.ctrlKey) { toggle_sortable(false); } });
    $(document).on('keyup', function(event) { toggle_sortable(true); });
    $(window).on('focusout', function(event) { toggle_sortable(true); });
    $(window).on('beforeunload', function(e) {
        save(current_config_file);
    });
}

function toggle_sortable(enable) {
    if (enable) {
        $('#container').sortable('enable');
        $('.panel').css('cursor', 'move');
    } else {
        $('#container').sortable('disable');
        $('.panel').css('cursor', 'auto');
    }
}

function save(config_file) {
    if (!config_file) {
        return;
    }
    save_layout(config_file);
    save_gui_config(config_file);
    save_last_config_file(config_file);
}

function load(config_file) {
    if (!config_file) {
        return;
    }
    if (!load_gui_config(config_file)) {
        return;
    }
    load_layout(config_file);
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

function save_last_config_file(config_file) {
    var key = 'feeds.config_file';
    localStorage.setItem(key, config_file);
}

function load_last_config_file() {
    var key = 'feeds.config_file';
    return localStorage.getItem(key);
}

function save_layout(config_file) {
    var key = 'feeds.layout[' + config_file + ']';
    var layout = $('#container > li').map(function() {
        var id = this.id;
        var feed = feeds[id];
        var layout = { id: id };
        if (feed.target.is('img') && feed.target.attr('src') && feed.target.is(':visible')) {
            layout.width = feed.target.width();
            layout.height = feed.target.height();
        }
        return layout;
    }).get();
    layout.push({
        id: 'container',
        width: $('#container').width()
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
    layout.forEach(function(value, index) {
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
        if ('width' in value) {
            feed.target.width(value.width);
        }
        if ('height' in value) {
            feed.target.height(value.height);
        }
    });
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

function set_properties(config, folder_name) {
    for (var id in config) {
        var value = config[id];
        if (typeof value === 'object') {
            set_properties(value, folder_name);
        } else {
            gui.setProperty(id, value, folder_name);
        }
    }
}

function load_gui_config(config_file) {
    var key = 'feeds.gui.config[' + config_file + ']';
    var config = JSON.parse(localStorage.getItem(key));
    if (!config) {
        return false;
    }
    gui.setProperty('alert_beep', config['globals'].alert_beep, 'globals');
    for (var id in feeds) {
        if (!(id in config)) {
            return;
        }
    }
    for (var id in feeds) {
        set_properties(config[id], id);
        feeds[id].init();
    }
    return true;
}

function parse_query_string() {
    var query = location.search.substring(1);
    var pairs = query.split('&');
    var result = {}
    for (var i in pairs) {
        var pair = pairs[i].split('=');
        var key = decodeURIComponent(pair[0]);
        var value = decodeURIComponent(pair[1]);
        result[key] = value;
    }
    return result;
}

function load_config(file) {
    $.ajax({
        url: file,
    }).done(function(data, textStatus, jqXHR) {
        initialize(data);
    }).fail(function(jqXHR, textStatus, errorThrown) {
        $('#container').append('<p>error reading <a href="' + file + '">' + file + '</a>: ' + textStatus + ' ' + errorThrown + '</p>');
        $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
        $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
    });
}

var query_string = parse_query_string();

$(function() {
    container_width = $('#container').width();
    $.ajax({
        url: globals.config_dir,
    }).done(function(data, textStatus, jqXHR) {
        $(data).find('a').each(function(index) {
            var href = $(this).attr('href');
            if (href.match(/[.]json$/)) {
                config_files.push(globals.config_dir + '/' + href);
            }
        });
        if (config_files.length && config_files.indexOf(globals.config_file) == -1) {
            globals.config_file = config_files[0];
        }
    }).fail(function(jqXHR, textStatus, errorThrown) {
    }).always(function(data_jqXHR, textStatus, errorThrown) {
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
});
