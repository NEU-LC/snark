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
                if (_this.options.extent.endsWith('.csv')) {
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
    return ImageLayer;
});
