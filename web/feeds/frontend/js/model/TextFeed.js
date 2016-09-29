/**
 * Created by vrushali on 15/10/15.
 */
define('TextFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');
    var TextFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
    };
    TextFeed.prototype = Object.create(Feed.prototype);
    TextFeed.prototype.load = function () {
        $.ajax({
            context: this,
            url: this.get_url()
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
    };
    TextFeed.prototype.onload_ = function (data) {
        if (data && data.length && data[data.length - 1] == '\n') {
            data = data.substring(0, data.length - 1);
        }
        data = data ? data : '&nbsp;';
        this.target.append('<tr><td><pre>' + data + '</pre></td></tr>');
        this.draw();
    };
    TextFeed.prototype.draw = function () {
        while (this.target.find('tbody tr').length > this.config.text.show_items) {
            this.target.find('tbody tr').first().remove();
        }
    };
    return TextFeed;
});
