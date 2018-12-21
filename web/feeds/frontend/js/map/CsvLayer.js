// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
        var feature = new ol.Feature();
        var stroke = new ol.style.Stroke({
            color: this.options.stroke,
            width: this.options.stroke_width
        });
        if (this.options.geometry == 'point') {
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
            var geometry = new ol.geom.MultiLineString();
            var line = new ol.geom.LineString();
            var prev;
            var cross_antimeridian = false;
            while (true) {
                var o = this.csv.readObject();
                if (!o || !o.longitude || !o.latitude) {
                    break;
                }
                if (prev && Math.abs(o.longitude - prev.longitude) >= 180) {
                    cross_antimeridian = true;
                    geometry.appendLineString(line);
                    line = new ol.geom.LineString();
                }
                line.appendCoordinate([o.longitude, o.latitude]);
                prev = o;
            }
            geometry.appendLineString(line);
            feature.setGeometry(geometry);
            feature.setStyle(new ol.style.Style({
                stroke: stroke
            }));
            if (cross_antimeridian) {
                var center = ol.extent.getCenter(geometry.getExtent());
                center[0] += (center[0] >= 0 ? 1 : -1) * 180;
                this.center = ol.proj.transform(center, 'EPSG:4326', 'EPSG:3857');
            }
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
        if (this.center) {
            return this.center;
        }
        return ol.extent.getCenter(feature.getGeometry().getExtent());
    }
    CsvLayer.prototype.get_extent = function () {
        var feature = this.source.getFeatures()[0];
        if (!feature) {
            return;
        }
        return feature.getGeometry().getExtent();
    }
    return CsvLayer;
});
