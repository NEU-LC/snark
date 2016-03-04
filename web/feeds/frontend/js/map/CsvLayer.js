define(['Csv', 'jquery', 'ol', 'utils'], function (Csv) {
    function CsvLayer(id, options) {
        this.id = id;
        this.options = options;
        this.source = new ol.source.Vector();
        this.layer = new ol.layer.Vector({
            source: this.source
        });
    };
    CsvLayer.prototype.load = function () {
        var _this = this;
        if (this.options.binary) {
            var xhr = new XMLHttpRequest();
            xhr.open('GET', this.options.filename, true);
            xhr.responseType = 'arraybuffer';
            xhr.onload = function (event) {
                if (this.status != 200) {
                    console.error(event);
                    return;
                }
                _this.load_data(xhr.response);
            }
            xhr.send();
        } else {
            $.get(this.options.filename, function(data) {
                _this.load_data(data);
            });
        }
    };
    CsvLayer.prototype.load_data = function (data) {
        this.csv = new Csv(this.options, data);
        var coordinates = [];
        while (true) {
            var o = this.csv.readObject();
            if (!o || !o.longitude || !o.latitude) {
                break;
            }
            coordinates.push([o.longitude, o.latitude]);
        }
        if (!coordinates.length) {
            console.warn(this.options.filename + ' has no points');
            return;
        }
        var feature = new ol.Feature();
        var stroke = new ol.style.Stroke({
            color: this.options.stroke,
            width: this.options.stroke_width
        });
        if (this.options.geometry == 'point') {
            feature.setGeometry(new ol.geom.MultiPoint(coordinates));
            feature.setStyle(new ol.style.Style({
                image: new ol.style.Circle({
                    fill: new ol.style.Fill({
                        color: this.options.fill
                    }),
                    stroke: stroke,
                    radius: this.options.radius
                })
            }));
        } else {
            feature.setGeometry(new ol.geom.LineString(coordinates));
            feature.setStyle(new ol.style.Style({
                stroke: stroke
            }));
        }
        feature.getGeometry().transform('EPSG:4326', 'EPSG:3857');
        this.source.addFeature(feature);
        if (this.onload) {
            this.onload();
        }
    };
    CsvLayer.prototype.get_center = function () {
        var feature = this.source.getFeatures()[0];
        if (!feature) {
            return;
        }
        return ol.extent.getCenter(feature.getGeometry().getExtent());
    }
    return CsvLayer;
});
