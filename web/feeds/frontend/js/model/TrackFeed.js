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
        var img = new Image();
        var feed = this;
        img.onload = function () {
            feed.target.width(this.width);
            feed.target.height(this.height);
            feed.target.css('background', 'url(' + this.src + ')');
            feed.target.css('bacgkround-repeat', 'no-repeat');
            feed.ctx.canvas.width = this.width;
            feed.ctx.canvas.height = this.height;
        };
        img.src = this.config.track.background_url;
    }
    TrackFeed.prototype.draw = function() {
        this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);
        for (var i = this.points.length; i--; ) {
            var point = this.points[i];
            point.alpha -= this.config.track.alpha_step;
            if (point.alpha <= 0) {
                this.points.splice(i, 1);
            }
            this.ctx.beginPath();
            this.ctx.arc(point.x, point.y, this.config.track.radius, 0, 2 * Math.PI);
            this.ctx.fillStyle = "rgba(" + this.config.track.fill.concat(point.alpha).join(',') + ")";
            this.ctx.fill();
            if (this.config.track.strokeWidth) {
                this.ctx.lineWidth = this.config.track.strokeWidth;
                this.ctx.strokeStyle = "rgba(" + this.config.track.stroke.concat(point.alpha).join(',') + ")";
                this.ctx.stroke();
            }
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
        var point = data.split(',').map(Number);
        this.points.push(new TrackPoint(point[0], point[1]));
    };

    return TrackFeed;
});
