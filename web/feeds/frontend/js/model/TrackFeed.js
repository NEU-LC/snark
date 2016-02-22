define('TrackFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');

    var TrackPoint = function(x, y, alpha) {
        this.x = x;
        this.y = y;
        this.alpha = alpha || TrackPoint.DEFAULT_ALPHA;
    }
    TrackPoint.DEFAULT_ALPHA = 1;

    var TrackFeed = function (feed_name, config) {
        this.base = Feed;
        this.base(feed_name, config);
        this.ctx = this.target[0].getContext('2d');
        this.points = [];

        this.image_loader = new Image();
        var _this = this;
        this.image_loader.onload = function () {
            _this.original_width = this.width;
            _this.original_height = this.height;
            _this.target.width(this.width);
            _this.target.height(this.height);
            _this.target.css('background', 'url(' + this.src + ')');
            _this.target.css('background-size', 'cover');
            _this.ctx.canvas.width = this.width;
            _this.ctx.canvas.height = this.height;
            _this.resize();
            _this.set_extent();
        };

        this.set_background();
        this.reset_draw_interval();
    };

    TrackFeed.prototype = Object.create(Feed.prototype);
    TrackFeed.prototype.reset_draw_interval = function() {
        if (this.draw_interval) {
            clearInterval(this.draw_interval);
            this.draw_interval = null;
        }
        this.draw_interval = setInterval(this.draw.bind(this), this.config.track.draw_interval);
    }
    TrackFeed.prototype.set_background = function() {
        if (!this.config.track.background_url) {
            this.target.css('background', '');
            return;
        }
        this.image_loader.src = this.config.track.background_url;
    }
    TrackFeed.prototype.set_extent = function() {
        var extent = this.config.track.extent;
        if (extent) {
            switch (extent.constructor) {
                case Array:
                    this.extent = extent;
                    break;
                case String:
                    var _this = this;
                    var width = this.original_width;
                    var height = this.original_height;
                    $.get(extent, function (data) {
                        data = data.trim().split('\n').map(Number);
                        _this.extent = [
                            data[4],                       // min x
                            data[5] + data[3] * height,    // min y
                            data[4] + data[0] * width,     // max x
                            data[5],                       // max y
                        ]
                    });
                    break;
            }
        } else {
            this.extent = extent;
        }
    }

    TrackFeed.prototype.resize = function() {
        if (this.target.resizable('instance')) {
            this.target.resizable('destroy');
        }
        this.target.css('margin-top', '5px');
        var scale = Math.max(this.config.track.scale, this.min_scale()) / 100;
        this.target.width(this.original_width * scale);
        this.target.height(this.original_height * scale);
        var feed = this;
        this.target.resizable({
            aspectRatio: true,
            autoHide: true,
            minHeight: 269,
            resize: function (event, ui) {
                feed.config.track.scale = ui.size.width / feed.original_width * 100;
                gui.updateDisplay('scale', feed.feed_name);
            }
        });
    }
    TrackFeed.prototype.min_scale = function() {
        return 269 / this.original_height * 100;
    }
    TrackFeed.prototype.draw = function() {
        this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);
        for (var i in this.points) {
            var point = this.points[i];
            this.ctx.beginPath();
            this.ctx.arc(point.x, point.y, this.config.track.radius, 0, 2 * Math.PI);
            this.ctx.fillStyle = "rgba(" + this.config.track.fill.concat(point.alpha).join(',') + ")";
            this.ctx.fill();
            if (this.config.track.stroke_width) {
                this.ctx.lineWidth = this.config.track.stroke_width;
                this.ctx.strokeStyle = "rgba(" + this.config.track.stroke.concat(point.alpha).join(',') + ")";
                this.ctx.stroke();
            }
            point.alpha -= this.config.track.alpha_step;
        }
        var length = this.points.length;
        for (var i = length; i-- && this.points.length >= length; ) {
            var point = this.points[i];
            if (point.alpha <= 0) {
                this.points.splice(i, 1);
            }
        }
    }
    TrackFeed.prototype.remove_trail = function() {
        if (this.points.length > 1) {
            this.points = [this.points.pop()];
        }
    }
    TrackFeed.prototype.load = function () {
        $.ajax({
            context: this,
            url: this.get_url()
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
    };
    TrackFeed.prototype.onload_ = function (data) {
        if (!data) { return; }
        var p = data.split(',').map(Number);
        var point = new TrackPoint(p[0], p[1]);
        if (this.extent) {
            point.x = this.ctx.canvas.width * (point.x - this.extent[0]) / (this.extent[2] - this.extent[0]) ;
            point.y = this.ctx.canvas.height * (1 - (point.y - this.extent[1]) / (this.extent[3] - this.extent[1]));
        }
        if (this.config.track.trail) {
            this.points.push(point);
        } else {
            this.points = [point];
        }
    };

    return TrackFeed;
});
