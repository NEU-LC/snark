define(['handlebars', 'CsvLayer', 'ImageLayer', 'jquery', 'jquery_ui', 'dat_gui', 'ol', 'utils'], function (Handlebars, CsvLayer, ImageLayer) {
    function MapApp(id) {
        this.id = id;
        this.layers = {};
        this.layers_added = 0;
        this.query_object = parse_query_string();
        this.load_config();
    };
    function Options(options) {
        Object.assign(this, options);
        this.imagery_set = this.imagery_set || 'Road';
        this.bing_maps_key = this.bing_maps_key || '';
    }
    function CsvOptions(options) {
        Object.assign(this, options);
        this.filename = this.filename || '';
        this.binary = this.binary || '';
        this.format = this.format || '';
        this.guess = 'guess' in this ? this.guess : 'true';
        this.fields = this.fields || '';
        this.geometry = this.geometry || this.shape || 'line';
        this.fill = this.fill || 'rgba(255, 255, 255, 0.4)';
        this.stroke = this.stroke || '#3399CC';
        this.stroke_width = Number(this.stroke_width) || 1.25;
        this.radius = Number(this.radius) || 5;
    }
    Object.defineProperty(CsvOptions.prototype, 'name', { get: function() { return this.filename; } });
    function ImageOptions(options) {
        Object.assign(this, options);
        this.image = this.image || '';
        this.extent = this.extent || (this.image ? this.image + 'w' : '');
    }
    Object.defineProperty(ImageOptions.prototype, 'name', { get: function() { return this.image; } });
    MapApp.prototype.load_config = function () {
        var config = this.query_object['config'] != undefined ? this.query_object['config'] : 'config/map.json';
        var _this = this;
        $.get(config, function(data) {
            _this.init(data);
        }).fail(function(jqXHR, textStatus, errorThrown) {
            _this.init();
        });
    }
    MapApp.prototype.init = function (config) {
        this.config = config || {};
        this.options = new Options(config);
        this.init_from_query_string();
        this.init_map();
        this.init_gui();
        this.init_layer_list();
        this.add_layers_from_config();
        this.add_layers_from_query_string();
    };
    MapApp.prototype.init_from_query_string = function () {
        var keys = Object.keys(new Options());
        for (var i in keys) {
            var key = keys[i];
            if (key in this.query_object) {
                this.options[key] = this.query_object[key];
            }
        }
    }
    MapApp.prototype.init_map = function () {
        this.map = new ol.Map({
            target: this.id,
            loadTilesWhileInteracting: true,
            view: new ol.View({
                center: ol.proj.transform([151.193698, -33.890146], 'EPSG:4326', 'EPSG:3857'),
                projection: ol.proj.get('EPSG:3857'),
                minZoom: 1,
                maxZoom: 40,
                zoom: 30,
                zoomFactor: 1.5
            })
        });
        if (this.options.bing_maps_key) {
            this.map.addLayer(new ol.layer.Tile({
                source: new ol.source.BingMaps({
                    imagerySet: this.options.imagery_set,
                    key: this.options.bing_maps_key
                })
            }));
        }
    };
    MapApp.prototype.init_gui = function () {
        this.gui = new dat.GUI({
            width: 500
        });
        this.init_gui_map_options();
        this.init_gui_add_csv_layer();
        this.init_gui_add_image_layer();
    }
    MapApp.prototype.init_gui_map_options = function () {
        var _this = this;
        this.map_options_folder = this.gui.addFolder('Map Options');
        this.map_options_folder.add(this.options, 'imagery_set', ['Aerial', 'AerialWithLabels', 'Road']).name('imagery set').onFinishChange(function (value) {
            _this.set_base_layer();
        });
        this.map_options_folder.add(this.options, 'bing_maps_key').name('bing maps key').onFinishChange(function (value) {
            _this.set_base_layer();
        });
    }
    MapApp.prototype.set_base_layer = function () {
        this.map.getLayers().setAt(0, new ol.layer.Tile({
            source: new ol.source.BingMaps({
                imagerySet: this.options.imagery_set,
                key: this.options.bing_maps_key
            })
        }));
    }
    MapApp.prototype.init_gui_add_csv_layer = function () {
        var _this = this;
        this.add_csv_layer_folder = this.gui.addFolder('Add CSV Layer');
        this.add_csv_layer_options = new CsvOptions();
        Object.defineProperty(this.add_csv_layer_options, 'add', {
            value: function() { _this.add_layer_from_options(this); }
        });
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'filename');
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'fields');
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'binary');
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'geometry', ['', 'line', 'point']);
        this.add_csv_layer_folder.addColor(this.add_csv_layer_options, 'fill');
        this.add_csv_layer_folder.addColor(this.add_csv_layer_options, 'stroke');
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'stroke_width', 0, 10).step(0.5).name('stroke width');
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'radius', 1, 20).step(1);
        this.add_csv_layer_folder.add(this.add_csv_layer_options, 'add');
    };
    MapApp.prototype.init_gui_add_image_layer = function () {
        var _this = this;
        this.add_image_layer_folder = this.gui.addFolder('Add Image Layer');
        this.add_image_layer_options = new ImageOptions();
        Object.defineProperty(this.add_image_layer_options, 'add', {
            value: function() { _this.add_layer_from_options(this); }
        });
        this.add_image_layer_folder.add(this.add_image_layer_options, 'image');
        this.add_image_layer_folder.add(this.add_image_layer_options, 'extent');
        this.add_image_layer_folder.add(this.add_image_layer_options, 'add');
    };
    MapApp.prototype.init_layer_list = function () {
        this.layer_list_item_template = Handlebars.compile(
            '<li class="layer-list-item" id="{{id}}"><span class="center-icon" title="center">&#10753;</span> <label title="view"><input type="checkbox" checked/><span class="view-icon">&#x1f441;</span> {{label}}</label><span class="remove-icon" title="remove">&#x00d7;</span></li>'
        );
        this.layer_list = $('<ul/>', { id: 'layer-list' }).appendTo('body');
        var _this = this;
        this.layer_list.sortable({
            cursor: 'move',
            opacity: 0.8,
            placeholder: 'layer-list-item layer-list-item-placeholder',
            start: function(event, ui) {
                ui.placeholder.width(ui.item.width());
                ui.placeholder.height(ui.item.height());
                ui.item.width(ui.item.width() + 1); // to prevent wrapping as fractional widths are truncated
            },
            stop: function(event, ui) {
                var layer_id = $(ui.item).closest('li').attr('id');
                _this.reorder_layer(_this.layers[layer_id]);
            }
        });
    }
    MapApp.prototype.get_next_layer_id = function() {
        return 'layer' + ++this.layers_added;
    }
    MapApp.prototype.add_layer = function(Layer, options) {
        var layer = new Layer(this.get_next_layer_id(), options);
        var _this = this;
        layer.onload = function () {
            _this.reorder_layer(this);
            var center = this.get_center();
            if (center) {
                _this.map.getView().setCenter(center);
            }
        }
        layer.load();
        this.layers[layer.id] = layer;
        var list_item = this.layer_list_item_template({
            id: layer.id,
            label: layer.id.replace('layer', '') + ': ' + options.name
        });
        this.layer_list.prepend(list_item);
        //this.layer_list.append(list_item);
        var li = $('li#' + layer.id);
        li.find('input').on('click', this.toggle_layer.bind(this));
        var remove_icon = li.find('.remove-icon');
        remove_icon.on('click', this.remove_layer.bind(this));
        li.find('.center-icon').on('click', this.center_on_layer.bind(this));
    }
    MapApp.prototype.toggle_layer = function(event) {
        var input = event.target;
        var li = $(input).closest('li');
        var layer_id = li.attr('id');
        this.layers[layer_id].layer.setVisible(input.checked);
        var view_icon = li.find('.view-icon');
        if (input.checked) {
            view_icon.removeClass('hidden');
        } else {
            view_icon.addClass('hidden');
        }
    }
    MapApp.prototype.remove_layer = function(event) {
        var input = event.target;
        var li = $(input).closest('li');
        var layer_id = li.attr('id');
        this.map.removeLayer(this.layers[layer_id].layer);
        li.remove();
        delete this.layers[layer_id];
    }
    MapApp.prototype.center_on_layer = function(event) {
        var input = event.target;
        var li = $(input).closest('li');
        var layer_id = li.attr('id');
        var center = this.layers[layer_id].get_center();
        if (center) {
            this.map.getView().setCenter(center);
        }
    }
    MapApp.prototype.get_layer_order = function() {
        return this.layer_list.find('li').map(function() { return this.id }).get().reverse();
    }
    MapApp.prototype.reorder_layer = function(layer) {
        var layers = this.map.getLayers();
        var order = this.get_layer_order();
        layers.remove(layer.layer);
        layers.insertAt(order.indexOf(layer.id) + 1, layer.layer);
    }
    function parse_layer_options(layer_options) {
        var pairs = layer_options.split(';');
        var options = {};
        for (var i in pairs) {
            var parts = pairs[i].split('=');
            options[parts[0]] = parts[1];
        }
        return options;
    }
    MapApp.prototype.add_layers_from_config = function() {
        var layers = this.config.layers;
        for (var i in layers) {
            this.add_layer_from_options(layers[i]);
        }
    }
    MapApp.prototype.add_layers_from_query_string = function() {
        var layers = this.query_object['layer'];
        if (layers) {
            if (layers.constructor === String) {
                this.add_layer_from_options(parse_layer_options(layers));
            } else {
                for (var i in layers) {
                    var layer_options = layers[i];
                    this.add_layer_from_options(parse_layer_options(layer_options));
                }
            }
        }
        if ('filename' in this.query_object || 'image' in this.query_object) {
            this.add_layer_from_options(this.query_object);
        }
    }
    MapApp.prototype.add_layer_from_options = function(options) {
        if ('filename' in options) {
            this.add_layer(CsvLayer, new CsvOptions(options));
        } else if ('image' in options) {
            this.add_layer(ImageLayer, new ImageOptions(options));
        } else {
            console.error('required fields missing');
        }
    }
    return MapApp;
});
