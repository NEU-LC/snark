/**
 * Created by vrushali on 15/10/15.
 */
define('GraphFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');

    var GraphFeed = function (feed_name, config) {
        this.base = Feed;
        this.base(feed_name, config);
        this.default_threshold = {value: this.config.graph.max, color: '#5cb85c'};
        this.default_exceeded_threshold = {value: Number.MAX_VALUE, color: '#d9534f', alert: true};
        this.text = $(this.id + ' .graph-text');
        this.bars = $(this.id + ' .graph-bars');
        this.bars.append('<div class="graph-bar-col"><span class="graph-bar"></span></div>');
        this.bars_width = Number($('.graph-bars').css('width').replace('px', ''));
        this.bar_width = Number($('.graph-bar-col').css('width').replace('px', ''));
        this.bar_height = Number($('.graph-bar-col').css('height').replace('px', ''));
        this.bar_count = Math.floor(this.bars_width / this.bar_width);
        if (!isFinite(this.bar_count)) {
            throw 'invalid graph bar styles; calculated bar count: ' + this.bar_count;
        }
        for (var i = 1; i < this.bar_count; ++i) {
            this.bars.append('<div class="graph-bar-col"><span class="graph-bar"></span></div>');
        }
        var _this = this;
        this.bars.find('.graph-bar-col').each(function (i, e) {
            _this.config.graph.thresholds.forEach(function (threshold, index) {
                if (threshold.value > _this.config.graph.max) {
                    return;
                }
                var span = $('<span class="graph-threshold"></span>');
                span.css('height', _this.get_bar_height(threshold.value) + 1 + 'px');
                span.css('border-bottom-color', threshold.color);
                span.appendTo(e);
            });
        });
        $('.graph-bar-col').tooltip();
    };
    GraphFeed.prototype = Object.create(Feed.prototype);
    GraphFeed.prototype.load = function () {
        $.ajax({
            context: this,
            url: this.get_url()
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
    };
    GraphFeed.prototype.onload_ = function (data) {
        var text = data ? data.replace("\n", '') : 'N/A';
        if (this.config.graph.units) {
            text += ' ' + this.config.graph.units;
        }
        this.text.html(text);
        var bar = this.bars.children().first();
        this.bars.append(bar);
        var value = isNaN(data) ? this.config.graph.min : Number(data);
        var height = this.get_bar_height(value);
        var threshold = this.get_threshold(value);
        bar.find('.graph-bar').css('height', height);
        bar.css('background', threshold.color);
        if (this.config.alert) {
            this.alert(threshold.alert);
        }
        bar.data('bs.tooltip').options.title = text + ' @ ' + this.refresh_time;
        bar.find('.graph-threshold').each(function (index, value) {
            var e = $(value);
            if (Number(e.css('height').replace('px', '')) >= height) {
                e.hide();
            } else {
                e.show();
            }
        });
    };
    GraphFeed.prototype.get_bar_height = function (value) {
        var scale = (value - this.config.graph.min) / (this.config.graph.max - this.config.graph.min);
        if (scale > 1) {
            scale = 1;
        } else if (scale < 0) {
            scale = 0;
        }
        return this.bar_height - scale * this.bar_height;
    };
    GraphFeed.prototype.get_threshold = function (value) {
        if (!this.config.graph.thresholds.length) {
            return this.default_threshold;
        }
        for (var i in this.config.graph.thresholds) {
            var threshold = this.config.graph.thresholds[i];
            if (value <= threshold.value) {
                return threshold
            }
        }
        return this.default_exceeded_threshold;
    };
    return GraphFeed;
});
