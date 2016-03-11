define(['jquery', 'Feed', 'Grid'], function ($, Feed, Grid) {

    var ImageFeed = function (feed_name, config) {
        this.base = Feed;
        this.base(feed_name, config);
        this.popup_img = $("#popup" + this.feed_name + " img");
        this.init();
    };

    ImageFeed.prototype = Object.create(Feed.prototype);
    ImageFeed.prototype.init = function () {
        this.img = $('<img>').appendTo(this.target);
        this.loader = new Image();
        var _this = this;
        this.loader.onload = function () {
            _this.onload(this.src);
        };
        this.loader.onerror = function () {
            _this.onerror();
        }
    }
    ImageFeed.prototype.load = function () {
        this.loader.width_prev = this.loader.width;
        this.loader.height_prev = this.loader.height;
        this.loader.src = this.get_url();
    };
    ImageFeed.prototype.onload_ = function (data) {
        this.img.attr('src', data);
        this.popup_img.attr('src', data);
        if (this.loader.width != this.loader.width_prev || this.loader.height != this.loader.height_prev) {
            if (this.target.resizable('instance')) {
                this.target.resizable('destroy');
            }
            this.target.width(this.loader.width);
            this.target.height(this.loader.height);
            this.target.resizable({
                aspectRatio: true,
                autoHide: true,
                minWidth: 269,
                minHeight: 269
            });
            this.remove_grid();
        }
        if (this.config.grid && this.config.grid.show && !this.grid) {
            this.add_grid();
        }
    };
    ImageFeed.prototype.add_grid = function () {
        this.grid = new Grid(this.target, this.config.grid, this.loader.width, this.loader.height);
    }
    ImageFeed.prototype.remove_grid = function () {
        if (this.grid) {
            this.grid.remove();
            delete this.grid;
        }
    }
    ImageFeed.prototype.draw_grid = function () {
        if (this.grid) {
            this.grid.draw();
        }
    }

    return ImageFeed;
});
