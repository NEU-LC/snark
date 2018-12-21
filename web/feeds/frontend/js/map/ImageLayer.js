// This file is part of comma, a generic and flexible library
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

define(['jquery', 'ol', 'utils'], function () {
    function ImageLayer(id, options) {
        this.id = id;
        this.options = options;
    };
    ImageLayer.prototype.load = function () {
        var _this = this;
        var image = new Image();
        image.onload = function () {
            $.get(_this.options.extent, function (data) {
                var extent;
                if (data.indexOf(',') >= 0) {
                    extent = data.trim().split(',').map(Number);
                } else {
                    data = data.trim().split('\n').map(Number);
                    extent = world_to_extent(data, image.width, image.height);
                }
                extent = ol.proj.transformExtent(extent, 'EPSG:4326', 'EPSG:3857');
                _this.load_image(image, extent);
            });
        };
        image.src = this.options.image;
    };
    ImageLayer.prototype.load_image = function (image, extent) {
        this.image = image;
        this.extent = extent;
        this.layer = new ol.layer.Image({
            source: new ol.source.ImageStatic({
                url: image.src,
                imageExtent: extent
            })
        });
        if (this.onload) {
            this.onload();
        }
    };
    ImageLayer.prototype.get_center = function () {
        return ol.extent.getCenter(this.extent);
    }
    ImageLayer.prototype.get_extent = function () {
        return this.extent;
    }
    return ImageLayer;
});
