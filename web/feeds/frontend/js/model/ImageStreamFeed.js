define(['jquery', 'Grid'], function ($, Grid) {

    var ImageStreamFeed = function (feed_name, config) {
        this.feed_name = feed_name;
        this.config = config;
        this.id = '#' + this.feed_name;
        this.el = $(this.id);
        this.target = $(this.id + ' .target');
        this.control = $(this.id + ' .panel-stream-control span');
        this.init();
    };
    ImageStreamFeed.prototype.init = function () {
        this.target.resizable({
            aspectRatio: true,
            autoHide: true,
            minWidth: 269,
            minHeight: 269
        });
        this.img = $('<img>').appendTo(this.target);
        var _this = this;
        this.img.one('load', function (event) {
            _this.check_grid();
        });
    };
    ImageStreamFeed.prototype.onload = function (data) {
        if (data.substr(0, 5) === 'data:') {
            this.img.attr('src', data);
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
        this.control.removeClass('text-muted glyphicon-stop').addClass('text-success glyphicon-refresh');
        this.el.removeClass('panel-disabled').addClass('panel-enabled');
        var gui_folder = $(gui.__folders[this.feed_name].__ul);
        if (globals.isMobile) {
            gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').first().removeClass('panel-disabled').addClass('panel-enabled');
        } else {
            gui_folder.find('.title').first().removeClass('panel-disabled').addClass('panel-enabled');
        }
    };

    ImageStreamFeed.prototype.stopped = function () {
        this.control.removeClass('text-success glyphicon-refresh').addClass('text-muted glyphicon-stop');
        this.el.removeClass('panel-enabled').addClass('panel-disabled');
        var gui_folder = $(gui.__folders[this.feed_name].__ul);
        if (globals.isMobile) {
            gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').removeClass('panel-enabled').addClass('panel-disabled');
        } else {
            gui_folder.find('.title').removeClass('panel-enabled').addClass('panel-disabled');
        }
    };
    ImageStreamFeed.prototype.refresh = function () {
        if (!this.is_open()) {
            this.reset({count: 1});
        }
    };
    ImageStreamFeed.prototype.start = function () {
        this.reset(this.config.stream.autoplay ? {} : {count: 1});
    };
    ImageStreamFeed.prototype.check_grid = function () {
        if (this.config.grid && this.config.grid.show) {
            this.remove_grid();
            this.add_grid();
        }
    }
    ImageStreamFeed.prototype.add_grid = function () {
        this.grid = new Grid(this.target, this.config.grid, this.img.width(), this.img.height());
    }
    ImageStreamFeed.prototype.remove_grid = function () {
        if (this.grid) {
            this.grid.remove();
            delete this.grid;
        }
    }
    ImageStreamFeed.prototype.draw_grid = function () {
        if (this.grid) {
            this.grid.draw();
        }
    }
    return ImageStreamFeed;
});
