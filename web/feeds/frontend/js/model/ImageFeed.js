/**
 * Created by vrushali on 15/10/15.
 */
define('ImageFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');

    var ImageFeed = function (feed_name, config) {
        this.base = Feed;
        this.base(feed_name, config);

        this.popup_img = $("#popup" + this.feed_name + " img");
        this.loader = new Image();
        this.loader.feed = this;
        this.loader.onload = function () {
            this.feed.onload(this.src);
        };
        this.loader.onerror = function () {
            this.feed.onerror();
        }
    };

    ImageFeed.prototype = Object.create(Feed.prototype);
    ImageFeed.prototype.load = function () {
        this.loader.src = this.get_url();
    };
    ImageFeed.prototype.onload_ = function (data) {
        this.target.attr('src', data);
        this.popup_img.attr('src', data);
        if (!this.is_resizable) {
            this.target.resizable({
                aspectRatio: true,
                autoHide: true,
                minWidth: 250,
                minHeight: 250
            });
            this.is_resizable = true;
        }
    };


    return ImageFeed;
});