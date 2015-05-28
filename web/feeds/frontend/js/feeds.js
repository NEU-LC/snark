var Sensor = function(sensor_name, config) {
    this.sensor_name = sensor_name;
    this.config = config;
    this.el = $('#' + this.sensor_name);
    this.status = $('#' + this.sensor_name + ' .status');
    this.time = $('#' + this.sensor_name + ' .time');
    this.target = $('#' + this.sensor_name + ' .target');
    this.interval = null;
    this.refresh_time = null;
    this.show = true;
};
Sensor.prototype.reset = function() {
    if (this.config.refresh.auto) {
        this.refresh();
    } else {
        this.clear_interval();
    }
}
Sensor.prototype.set_interval = function() {
    this.interval = setInterval(this.preload.bind(this), this.config.refresh.interval * 1000);
    this.status.removeClass('text-muted text-danger').addClass('text-success');
}
Sensor.prototype.clear_interval = function() {
    clearInterval(this.interval);
    delete pending[this.sensor_name];
    this.status.removeClass('text-success text-danger').addClass('text-muted');
}
Sensor.prototype.refresh = function() {
    this.clear_interval();
    this.preload();
    if (this.config.refresh.auto) {
        this.set_interval();
    }
}
Sensor.prototype.toggle_show = function() {
    if (this.show) {
        this.el.show();
    } else {
        this.el.hide();
    }
}
Sensor.prototype.preload = function() {
    if (pending[this.sensor_name]) {
        return;
    }
    pending[this.sensor_name] = true;
    this.refresh_time = Date();
    this.status.fadeTo(1000, 0.4);
    this.load();
}
Sensor.prototype.onload = function(data) {
    this.time.text(this.refresh_time);
    if (this.config.refresh.auto) {
        this.status.removeClass('text-danger').addClass('text-success');
    }
    this.status.finish().fadeTo(0, 1);
    this.onload_(data);
    delete pending[this.sensor_name];
}
Sensor.prototype.onerror = function() {
    if (this.config.refresh.auto) {
        this.status.removeClass('text-success').addClass('text-danger');
    }
    this.status.finish().fadeTo(0, 1);
    delete pending[this.sensor_name];
}
Sensor.prototype.get_url = function() {
    var url = this.config.url;
    return url + (url.indexOf('?') < 0 ? '?q=' : '&q=') + Math.random();
}

var ImageSensor = function(sensor_name, config) {
    this.base = Sensor;
    this.base(sensor_name, config);
    this.loader = new Image();
    this.loader.sensor = this;
    this.loader.onload = function() { this.sensor.onload(this.src); }
    this.loader.onerror = function() { this.sensor.onerror(); }
}
ImageSensor.prototype = new Sensor;
ImageSensor.prototype.load = function() {
    this.loader.src = this.get_url();
}
ImageSensor.prototype.onload_ = function(data) {
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

var ImageStreamSensor = function(sensor_name, config) {
    this.sensor_name = sensor_name;
    this.config = config;
    this.el = $('#' + this.sensor_name);
    this.target = $('#' + this.sensor_name + ' .target');
    this.control = $('#' + this.sensor_name + ' .panel-stream-control span');
    this.target.on('load', function() {
        var id = $(this).closest('li').attr('id');
        var sensor = sensors[id];
        if (!sensor.is_resizable) {
            sensor.target.resizable({
                aspectRatio: true,
                autoHide: true,
                minWidth: 250,
                minHeight: 250,
            });
            sensor.is_resizable = true;
        }
    });
    this.show = true;
    this.reset(this.config.stream.autoplay ? {} : { count: 1 });
}
ImageStreamSensor.prototype.onload = function(data) {
    if (data.substr(0, 5) === 'data:') {
        this.target.attr('src', data);
    } else {
        console.log(data);
    }
}
ImageStreamSensor.prototype.reset = function(options) {
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
    this.ws.sensor = this;
    this.ws.onopen = function() {
        console.log('ws open: ' + this.sensor.sensor_name + ' ' + this.url);
        this.sensor.started();

    }
    this.ws.onclose = function() {
        console.log('ws close: ' + this.sensor.sensor_name);
        this.sensor.stopped();
    }
    this.ws.onmessage = function(event) {
        this.sensor.onload(event.data);
    }
}
ImageStreamSensor.prototype.is_open = function() {
    return this.ws && this.ws.readyState === WebSocket.OPEN;
}
ImageStreamSensor.prototype.toggle = function() {
    if (this.is_open()) {
        this.stop();
    } else {
        this.reset();
    }
}
ImageStreamSensor.prototype.toggle_show = function() {
    if (this.show) {
        this.el.show();
    } else {
        this.el.hide();
        this.stop();
    }
}
ImageStreamSensor.prototype.stop = function() {
    if (this.ws) {
        this.ws.close();
    }
}
ImageStreamSensor.prototype.started = function() {
    this.control.removeClass('text-muted').addClass('text-success');
}

ImageStreamSensor.prototype.stopped = function() {
    this.control.removeClass('text-success').addClass('text-muted');
}
ImageStreamSensor.prototype.refresh = function() {
    if (!this.is_open()) {
        this.reset({ count: 1 });
    }
}

var TextSensor = function(sensor_name, config) {
    this.base = Sensor;
    this.base(sensor_name, config);
}
TextSensor.prototype = new Sensor;
TextSensor.prototype.load = function() {
    $.ajax({
        context: this,
        url: this.get_url()
    }).done(function(data, textStatus, jqXHR) {
        this.onload(data);
    }).fail(function(jqXHR, textStatus, errorThrown) {
        this.onerror();
    });
}
TextSensor.prototype.onload_ = function(data) {
    this.target.append('<li><pre>' + data + '</pre></li>');
    this.draw();
}
TextSensor.prototype.draw = function() {
    while (this.target.find('li').length > this.config.text.show_items ) {
        this.target.find('li').first().remove();
    }
}

var GraphSensor = function(sensor_name, config) {
    this.base = Sensor;
    this.base(sensor_name, config);
    this.text = $('#' + sensor_name + ' .graph-text');
    this.bars = $('#' + sensor_name + ' .graph-bars');
    this.bars.append('<div class="graph-bar-col"><span class="graph-bar"></span></div>');
    this.bars_width = $('.graph-bars').width();
    this.bar_width = $('.graph-bar-col').width();
    this.bar_height = $('.graph-bar-col').height();
    this.bar_count = Math.floor(this.bars_width / this.bar_width);
    if (!isFinite(this.bar_count)) {
        throw 'invalid graph bar styles; calculated bar count: ' + this.bar_count;
    }
    for (var i = 1; i < this.bar_count; ++i ) {
        this.bars.append('<div class="graph-bar-col"><span class="graph-bar"></span></div>');
    }
    $('.graph-bar-col').tooltip();
}
GraphSensor.prototype = new Sensor;
GraphSensor.prototype.load = function() {
    $.ajax({
        context: this,
        url: this.get_url()
    }).done(function(data, textStatus, jqXHR) {
        this.onload(data);
    }).fail(function(jqXHR, textStatus, errorThrown) {
        this.onerror();
    });
}
GraphSensor.prototype.onload_ = function(data) {
    var text = data.replace("\n", '');
    if (this.config.graph.units) {
        text += ' ' + this.config.graph.units;
    }
    this.text.html(text);
    var bar = this.bars.children().first();
    this.bars.append(bar);
    var height = this.bar_height;
    if (!isNaN(data)) {
        var value = Number(data);
        var scale = (value - this.config.graph.min) / (this.config.graph.max - this.config.graph.min);
        if (scale > 1) {
            scale = 1;
        } else if (scale < 0) {
            scale = 0;
        }
        height = this.bar_height - scale * this.bar_height;
    }
    bar.find('.graph-bar').css('height', height);
    bar.data('bs.tooltip').options.title = text + ' @ ' + this.refresh_time;
}

var add_panel = function(sensor_name) {
    $('#container').append(
        '<li id="' + sensor_name + '" class="panel">' +
        '  <button type="button" class="panel-close text-muted transparent pull-right" title="close"><span>&times;</span></button>' +
        '  <div class="panel-body">' +
        '  </div>' +
        '</li>'
    );
}

var add_poll_body = function(sensor_name, element) {
    var id = '#' + sensor_name;
    $(id + ' .panel-body').append(
        '<h3>' + sensor_name + ' <button class="panel-refresh" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: auto refresh"><span class="status text-muted glyphicon glyphicon-stop"></span></button><button class="panel-settings transparent" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button></h3>' +
        '<p class="time small">&nbsp;</p>' +
        element
    );
}

var add_stream_body = function(sensor_name, element) {
    var id = '#' + sensor_name;
    $(id + ' .panel-body').append(
        '<h3>' + sensor_name + ' <button class="panel-stream-control" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: start/stop"><span class="status text-muted glyphicon glyphicon-stop"></span></button><button class="panel-settings transparent" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button></h3>' +
        element
    );
}

var create_sensor = function(type, sensor_name, config) {
    add_panel(sensor_name);
    if (type == 'image') {
        add_poll_body(sensor_name, '<img class="target"/>');
        return new ImageSensor(sensor_name, config);
    } else if (type == 'text') {
        add_poll_body(sensor_name, '<ul class="target list-unstyled"></ul>');
        return new TextSensor(sensor_name, config);
    } else if (type == 'graph') {
        add_poll_body(sensor_name, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-bars"></div></div>');
        return new GraphSensor(sensor_name, config);
    } else if (type == 'stream') {
        add_stream_body(sensor_name, '<img class="target"/>');
        return new ImageStreamSensor(sensor_name, config);
    }
    throw 'unrecognised sensor type: ' + type;
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

var Globals = function(options) {
    this.refresh = function() {
        $.each(sensors, function(index, sensor) {
            sensor.refresh();
        });
    }
    this.start_auto_refresh = function() {
        $.each(sensors, function(index, sensor) {
            if (sensor.show) {
                gui.setProperty('auto', true, sensor.sensor_name);
            }
        });
    }
    this.stop_auto_refresh = function() {
        $.each(sensors, function(index, sensor) {
            gui.setProperty('auto', false, sensor.sensor_name);
        });
    }
    this.start_streams = function() {
        $.each(sensors, function(index, sensor) {
            if (sensor.config.type === 'stream') {
                sensor.reset();
            }
        });
    }
    this.stop_streams = function() {
        $.each(sensors, function(index, sensor) {
            if (sensor.config.type === 'stream') {
                sensor.stop();
            }
        });
    }
    this.start_all = function() {
        this.start_auto_refresh();
        this.start_streams();
    }
    this.stop_all = function() {
        this.stop_auto_refresh();
        this.stop_streams();
    }
    this.show_all = function() {
        $.each(sensors, function(index, sensor) {
            gui.setProperty('show', true, sensor.sensor_name);
        });
    }
    this.hide_all = function() {
        $.each(sensors, function(index, sensor) {
            gui.setProperty('show', false, sensor.sensor_name);
        });
    }
    this.config_file = options.config_file
}

var gui;
var sensors = {};
var pending = {};
var config_dir = 'config';
var config_files = [];
var globals = new Globals({
    config_file: config_dir + '/web.frontend.json',
});

function initialize(frontend_config) {
    globals.stop_all();
    $('#container').empty();
    sensors = {};
    pending = {};

    if (gui) {
        gui.destroy();
    }
    gui = new dat.GUI({
        width: 500,
    });
    gui.add(globals, 'config_file', config_files).name('config file').onFinishChange(function(value) {
        load_config(globals.config_file);
    });
    var folder = gui.addFolder('global actions');
    folder.add(globals, "refresh");
    folder.add(globals, "start_auto_refresh").name("start auto refresh");
    folder.add(globals, "stop_auto_refresh").name("stop auto refresh");
    folder.add(globals, "start_streams").name("start streams");
    folder.add(globals, "stop_streams").name("stop streams");
    folder.add(globals, "start_all").name("start all");
    folder.add(globals, "stop_all").name("stop all");
    folder.add(globals, "show_all").name("show all");
    folder.add(globals, "hide_all").name("hide all");

    for (var sensor_name in frontend_config.sensors) {
        var config = frontend_config.sensors[sensor_name];
        config.sensor_name = sensor_name;
        if (!('type' in config)) {
            config.type = 'image';
        }
        if (config.type == 'stream') {
            if (!('stream' in config)) {
                config.stream = {
                    autoplay: false
                };
            }
            config.stream.sensor_name = sensor_name;
            if (!('url' in config)) {
                var xpath = config.xpath || sensor_name;
                config.url = frontend_config.websocket + '?xpath=' + xpath + '&data_uri=true';
            }
        } else {
            if (!('refresh' in config)) {
                config.refresh = {};
            }
            config.refresh.sensor_name = sensor_name;
            if (!('auto' in config.refresh)) {
                config.refresh.auto = 'interval' in config.refresh && config.refresh.interval > 0;
            }
            if (!('interval' in config.refresh)) {
                config.refresh.interval = 2;
            }
            if (!('url' in config)) {
                config.url = frontend_config.host + '/' + sensor_name;
            }
        }
        if (config.type == 'text') {
            if (!('text' in config)) {
                config.text = {};
            }
            config.text.sensor_name = sensor_name;
            if (!('show_items' in config.text)) {
                config.text.show_items = 1;
            }
        } else if (config.type == 'graph') {
            if (!('graph' in config)) {
                config.graph = {};
            }
            config.graph.sensor_name = sensor_name;
            if (!('min' in config.graph)) {
                config.graph.min = 0;
            }
            if (!('max' in config.graph)) {
                config.graph.max = 100;
            }
        }
        var sensor = create_sensor(config.type, sensor_name, config);
        var folder = gui.addFolder(sensor_name);
        folder.add(sensor.config, 'url').onFinishChange(function(value) {
            sensors[this.object.sensor_name].reset();
        });
        folder.add(sensor, 'show').onFinishChange(function(value) {
            this.object.toggle_show();
            if (!value) {
                gui.setProperty('auto', false, this.object.sensor_name);
            }
        });
        if (config.type != 'stream') {
            folder.add(sensor.config.refresh, 'auto').name("auto refresh").onFinishChange(function(value) {
                sensors[this.object.sensor_name].reset();
            });
            folder.add(sensor.config.refresh, 'interval', 0, 90).name("refresh interval").step(1).onFinishChange(function(value) {
                if (value == 0) {
                    gui.setProperty('auto', false, this.object.sensor_name);
                } else {
                    sensors[this.object.sensor_name].reset();
                }
            });
            if (config.type == 'text') {
                folder.add(sensor.config.text, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function(value) {
                    sensors[this.object.sensor_name].draw();
                });
            }
            sensor.refresh();
        }
        sensors[sensor_name] = sensor;
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
        start: function() {
            $('#container').addClass('container-resize');
        },
        stop: function() {
            $('#container').removeClass('container-resize');
        }
    });

    $('.panel').on('mouseover', function() {
        $(this).find('button').addClass('thin-border');
        $(this).find('.panel-settings, .panel-close').removeClass('transparent');
        $(this).find('div button').addClass('button-shadow');
    })
    $('.panel').on('mouseout', function() {
        $(this).find('button').removeClass('thin-border');
        $(this).find('.panel-settings, .panel-close').addClass('transparent');
        $(this).find('div button').removeClass('button-shadow');
    })
    $('.panel button').tooltip({ html: true });
    $('.panel-refresh').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        if (event.shiftKey) {
            gui.toggleProperty('auto', id);
        } else {
            sensors[id].refresh();
        }
    });
    $('.panel-stream-control').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        if (event.shiftKey) {
            sensors[id].toggle();
        } else {
            sensors[id].refresh();
        }
    });
    $('.panel-settings').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        if (!gui.closed && !gui.__folders[id].closed) {
            gui.close();
            return;
        }
        $.each(gui.__folders, function(index, folder) {
            folder.close();
        });
        gui.__folders[id].open();
        gui.open();
    });
    $('.panel-close').on('click', function(event) {
        var id = $(this).closest('li').attr('id');
        gui.setProperty('show', false, id);
    });

    $(document).on('keydown', function(event) {
        if (event.ctrlKey) {
            sort_enable();
        }
    });
    $(document).on('keyup', function(event) { sort_disable(); });
    $(window).on('focusout', function(event) { sort_disable(); });
}

function sort_enable() {
    $('#container').sortable('disable');
    $('.panel').css('cursor', 'auto');
}

function sort_disable() {
    $('#container').sortable('enable');
    $('.panel').css('cursor', 'move');
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
function parse_query_string() {
    var result = {};
    var parameters = location.search.replace(/^\?/, '').split('&');
    for (var i in parameters) {
        var key_value = parameters[i].split('=');
        result[key_value[0]] = key_value[1];
    }
    return result;
}

$(function() {
    $.ajax({
        url: config_dir,
    }).done(function(data, textStatus, jqXHR) {
        $(data).find('a').each(function(index) {
            var href = $(this).attr('href');
            if (href.match(/[.]json$/)) {
                config_files.push(config_dir + '/' + href);
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
        }
        if (config_files.indexOf(globals.config_file) == -1) {
            config_files.push(globals.config_file);
        }
        load_config(globals.config_file);
    });
});
