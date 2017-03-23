define(['jquery', 'Feed', 'Grid', 'utils'], function ($, Feed, Grid) {

    var TrackPoint = function (x, y, alpha) {
        this.x = x;
        this.y = y;
        this.alpha = alpha || TrackPoint.DEFAULT_ALPHA;
    }
    TrackPoint.DEFAULT_ALPHA = 1;

    var TrackFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
        this.canvas = $('<canvas>', {class: 'track'}).appendTo(this.target);
        this.ctx = this.canvas[0].getContext('2d');
        this.points = [];

        this.image_loader = new Image();
        var _this = this;
        this.image_loader.onload = function () {
            _this.original_width = this.width;
            _this.original_height = this.height;
            _this.canvas.css('background', 'url(' + this.src + ')');
            _this.canvas.css('background-size', 'cover');
            _this.ctx.canvas.width = this.width;
            _this.ctx.canvas.height = this.height;
            _this.resize();
            _this.set_extent();
            _this.check_grid();
        };

        this.set_background();
        this.reset_draw_interval();
    };

    TrackFeed.prototype = Object.create(Feed.prototype);
    TrackFeed.prototype.reset_draw_interval = function () {
        if (this.draw_interval) {
            clearInterval(this.draw_interval);
            this.draw_interval = null;
        }
        this.draw_interval = setInterval(this.draw.bind(this), this.config.track.draw_interval);
    }
    TrackFeed.prototype.set_background = function () {
        if (!this.config.track.image) {
            this.target.css('background', '');
            return;
        }
        this.image_loader.src = this.config.track.image;
    }
    TrackFeed.prototype.set_extent = function () {
        var extent = this.config.track.extent;
        this.extent = undefined;
        if (extent) {
            if (extent.indexOf(',') >= 0) {
                this.extent = extent.trim().split(',').map(Number);
            } else {
                var _this = this;
                $.get(extent, function (data) {
                    if (data.indexOf(',') >= 0) {
                        _this.extent = data.trim().split(',').map(Number);
                    } else {
                        data = data.trim().split('\n').map(Number);
                        _this.extent = world_to_extent(data, _this.original_width, _this.original_height);
                    }
                });
            }
        }
    }

    TrackFeed.prototype.resize = function () {
        if (this.target.resizable('instance')) {
            this.target.resizable('destroy');
        }
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
    TrackFeed.prototype.min_scale = function () {
        return 269 / this.original_height * 100;
    }
    TrackFeed.prototype.draw = function () {
        this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);
        var _this = this;
        this.points = this.points.filter(function (point) {
            _this.ctx.beginPath();
            _this.ctx.arc(point.x, point.y, _this.config.track.radius, 0, 2 * Math.PI);
            _this.ctx.fillStyle = hex2rgb(_this.config.track.fill, point.alpha);
            _this.ctx.fill();
            if (_this.config.track.stroke_width) {
                _this.ctx.lineWidth = _this.config.track.stroke_width;
                _this.ctx.strokeStyle = hex2rgb(_this.config.track.stroke, point.alpha);
                _this.ctx.stroke();
            }
            point.alpha -= _this.config.track.alpha_step;
            return point.alpha > 0;
        });
    }
    TrackFeed.prototype.remove_trail = function () {
        if (this.points.length > 1) {
            this.points = [this.points.pop()];
        }
    }
    TrackFeed.prototype.load = function () {
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
    TrackFeed.prototype.onload_ = function (data) {
        if (!data) {
            return;
        }
        var p = data.split(',').map(Number);
        var point = new TrackPoint(p[0], p[1]);
        if (this.extent) {
            point.x = this.ctx.canvas.width * (point.x - this.extent[0]) / (this.extent[2] - this.extent[0]);
            point.y = this.ctx.canvas.height * (1 - (point.y - this.extent[1]) / (this.extent[3] - this.extent[1]));
        }
        if (this.config.track.trail) {
            this.points.push(point);
        } else {
            this.points = [point];
        }
    };
    TrackFeed.prototype.check_grid = function () {
        if (this.config.grid && this.config.grid.show) {
            this.remove_grid();
            this.add_grid();
        }
    }
    TrackFeed.prototype.add_grid = function () {
        this.grid = new Grid(this.target, this.config.grid, this.original_width, this.original_height);
    }
    TrackFeed.prototype.remove_grid = function () {
        if (this.grid) {
            this.grid.remove();
            delete this.grid;
        }
    }
    TrackFeed.prototype.draw_grid = function () {
        if (this.grid) {
            this.grid.draw();
        }
    }

    return TrackFeed;
});
