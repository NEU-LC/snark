define(['jquery', 'ol', 'Feed', 'utils'], function ($, ol, Feed) {

    var MapFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
        this.first_point = true;
        this.init_map();
        this.set_base_layer();
        this.reset_draw_interval();
    };

    MapFeed.prototype = Object.create(Feed.prototype);
    MapFeed.prototype.init_map = function () {
        this.feature_source = new ol.source.Vector();
        this.feature_layer = new ol.layer.Vector({
            source: this.feature_source
        });
        this.map = new ol.Map({
            layers: [this.feature_layer],
            target: this.target[0],
            loadTilesWhileInteracting: true
        });
        var feed = this;
        this.target.resizable({
            aspectRatio: false,
            autoHide: true,
            minHeight: 269,
            minWidth: 269,
            resize: function (event, ui) {
                feed.map.updateSize();
            }
        });
    }
    MapFeed.prototype.set_base_layer = function () {
        this.map.removeLayer(this.base_layer);
        var image = this.config.map.image;
        if (this.config.map.image) {
            var extent = this.config.map.extent;
            if (extent && extent.indexOf(',') >= 0) {
                extent = extent.trim().split(',').map(Number);
                return this.set_base_image(image, extent, new ol.proj.Projection({
                    code: 'pixels',
                    units: 'pixels',
                    extent: extent
                }));
            }
            return this.set_base_image_load(image, extent)
        }
        this.set_base_tile();
    }
    MapFeed.prototype.set_base_tile = function () {
        if (!this.config.map.bing_maps_key) {
            console.error('require "bing_maps_key" in config');
            return;
        }
        this.map.removeLayer(this.base_layer);
        this.projection = ol.proj.get('EPSG:3857');
        this.base_layer = new ol.layer.Tile({
            source: new ol.source.BingMaps({
                imagerySet: this.config.map.imagery_set,
                key: this.config.map.bing_maps_key
            })
        });
        this.view = new ol.View({
            center: this.view ? this.view.getCenter() : ol.proj.transform([151.193698, -33.890146], 'EPSG:4326', 'EPSG:3857'),
            projection: this.projection,
            minZoom: 1,
            maxZoom: 30,
            zoom: this.view ? this.view.getZoom() : 17
        });
        this.map.getLayers().insertAt(0, this.base_layer);
        this.map.setView(this.view);
    }
    MapFeed.prototype.set_base_image = function (source, extent, projection) {
        this.projection = projection || ol.proj.get('EPSG:3857');
        this.base_layer = new ol.layer.Image({
            source: new ol.source.ImageStatic({
                url: source,
                projection: this.projection,
                imageExtent: extent
            })
        });
        this.view = new ol.View({
            center: ol.extent.getCenter(extent),
            projection: this.projection,
            minZoom: 1,
            maxZoom: 40,
            zoom: this.projection.getCode() == 'EPSG:3857' ? 30 : 1,
            zoomFactor: 1.5
        });
        this.map.getLayers().insertAt(0, this.base_layer);
        this.map.setView(this.view);
    }
    MapFeed.prototype.set_base_image_load = function (source, extent) {
        var image = new Image();
        var feed = this;
        image.onload = function () {
            if (extent) {
                $.get(extent, function (data) {
                    var e;
                    if (data.indexOf(',') >= 0) {
                        e = data.trim().split(',').map(Number);
                    } else {
                        data = data.trim().split('\n').map(Number);
                        e = world_to_extent(data, image.width, image.height);
                    }
                    feed.set_base_image(source, ol.proj.transformExtent(e, 'EPSG:4326', 'EPSG:3857'));
                });
            } else {
                var e = [0, 0, image.width, image.height];
                feed.set_base_image(source, e, new ol.proj.Projection({
                    code: source,
                    units: 'pixels',
                    extent: e
                }));
            }
        };
        image.src = source;
    }
    MapFeed.prototype.reset_draw_interval = function () {
        if (this.draw_interval) {
            clearInterval(this.draw_interval);
            this.draw_interval = null;
        }
        this.draw_interval = setInterval(this.draw.bind(this), this.config.map.draw_interval);
    }
    MapFeed.prototype.draw = function () {
        var feed = this;
        var features = this.feature_source.getFeatures().filter(function (feature) {
            feature.alpha -= feed.config.map.alpha_step;
            return feature.alpha > 0;
        });
        this.feature_source.clear();
        this.feature_source.addFeatures(features);
    }
    MapFeed.prototype.load = function () {
        $.ajax({
            context: this,
            crossDomain: true,
            url: this.get_url(),
            timeout: globals.timeout
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
    };
    MapFeed.prototype.onload_ = function (data) {
        if (!data) {
            return;
        }
        if (!this.base_layer) {
            return;
        }
        var point = data.split(',').map(Number);
        var coordinates = this.projection.getCode() == 'EPSG:3857' ? ol.proj.transform(point, 'EPSG:4326', 'EPSG:3857') : point;
        var feature = new ol.Feature(new ol.geom.Point(coordinates));
        feature.alpha = 1;
        var feed = this;
        feature.setStyle(function (resolution) {
            return new ol.style.Style({
                image: new ol.style.Circle({
                    radius: feed.config.map.radius,
                    fill: new ol.style.Fill({
                        color: hex2rgb(feed.config.map.fill, this.alpha)
                    }),
                    stroke: new ol.style.Stroke({
                        color: hex2rgb(feed.config.map.stroke, this.alpha),
                        width: feed.config.map.stroke_width
                    })
                })
            });
        });
        if (!this.config.map.trail) {
            this.feature_source.clear();
        }
        this.feature_source.addFeature(feature);
        if (this.view) {
            if (this.first_point) {
                this.view.setCenter(coordinates);
                this.first_point = false;
            } else if (this.config.map.follow) {
                if (this.config.refresh.interval > 0.1) {
                    var pan = ol.animation.pan({
                        source: this.view.getCenter(),
                        duration: 100, // change depending on data rate
                    });
                    this.map.beforeRender(pan);
                }
                this.view.setCenter(coordinates);
            }
        }
    };

    return MapFeed;
});
