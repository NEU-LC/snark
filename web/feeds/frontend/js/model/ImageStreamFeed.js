/**
 * Created by vrushali on 15/10/15.
 */
define('ImageStreamFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');

    var ImageStreamFeed = function (feed_name, config) {
        this.feed_name = feed_name;
        this.config = config;
        this.id = '#' + this.feed_name;
        this.el = $(this.id);
        this.target = $(this.id + ' .target');
        this.control = $(this.id + ' .panel-stream-control span');
        this.target.on('load', function () {
            var id = $(this).closest('li').attr('id');
            var feed = feeds[id];
            if (!feed.is_resizable) {
                feed.target.resizable({
                    aspectRatio: true,
                    autoHide: true,
                    minWidth: 269,
                    minHeight: 269
                });
                feed.is_resizable = true;
            }
        });
    };
    ImageStreamFeed.prototype.init = function () {
    };
    ImageStreamFeed.prototype.onload = function (data) {
        if (data.substr(0, 5) === 'data:') {
            this.target.attr('src', data);
        } else {
            console.log(data);
        }
    };
    ImageStreamFeed.prototype.reset = function (options) {
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
        this.ws.feed = this;
        this.ws.onopen = function () {
            console.log('ws open: ' + this.feed.feed_name + ' ' + this.url);
            this.feed.started();

        };
        this.ws.onclose = function () {
            console.log('ws close: ' + this.feed.feed_name);
            this.feed.stopped();
        };
        this.ws.onmessage = function (event) {
            this.feed.onload(event.data);
        }
    };
    ImageStreamFeed.prototype.is_open = function () {
        return this.ws && this.ws.readyState === WebSocket.OPEN;
    };
    ImageStreamFeed.prototype.toggle = function () {
        if (this.is_open()) {
            this.stop();
        } else {
            this.reset();
        }
    };
    ImageStreamFeed.prototype.toggle_show = function () {
        if (this.config.show) {
            this.el.show();
        } else {
            this.el.hide();
            this.stop();
        }
    };
    ImageStreamFeed.prototype.stop = function () {
        if (this.ws) {
            this.ws.close();
        }
    };
    ImageStreamFeed.prototype.started = function () {
        this.control.removeClass('text-muted').addClass('text-success');
    };

    ImageStreamFeed.prototype.stopped = function () {
        this.control.removeClass('text-success').addClass('text-muted');
    };
    ImageStreamFeed.prototype.refresh = function () {
        if (!this.is_open()) {
            this.reset({count: 1});
        }
    };
    ImageStreamFeed.prototype.start = function () {
        this.reset(this.config.stream.autoplay ? {} : {count: 1});
    };
    return ImageStreamFeed;
});