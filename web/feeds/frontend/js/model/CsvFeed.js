/**
 * Created by vrushali on 15/10/15.
 */

define('CsvFeed', ["jquery", 'TextFeed'], function ($, Feed, TextFeed) {
    //var $ = require('jquery');
    //require("jquery_ui");
    var Feed = require('Feed');
    var TextFeed = require("TextFeed");

    var CsvFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
        this.init();
    };
    CsvFeed.prototype = Object.create(TextFeed.prototype);
    CsvFeed.prototype.init = function () {
        this.init_fields();
        this.init_ranges();
    };
    CsvFeed.prototype.init_fields = function () {
        this.target.find('thead').empty();
        if (this.config.csv.fields) {
            var tr = $('<tr>');
            if (this.config.type == 'csv-table') {
                var fields = this.config.csv.fields.split(',');
                for (var i in fields) {
                    $('<th class="text-center">')
                        .text(fields[i])
                        .appendTo(tr);
                }
            } else {
                $('<th>')
                    .text(this.config.csv.fields)
                    .appendTo(tr);
            }
            this.target.find('thead').append(tr);
        }
    };
    CsvFeed.prototype.init_ranges = function () {
        this.min = {};
        var min = this.config.csv.min.split(',');
        for (var i in min) {
            if (min[i] && !isNaN(min[i])) {
                this.min[i] = Number(min[i]);
            }
        }
        this.max = {};
        var max = this.config.csv.max.split(',');
        for (var i in max) {
            if (max[i] && !isNaN(max[i])) {
                this.max[i] = Number(max[i]);
            }
        }
    };
    CsvFeed.prototype.onload_ = function (data) {
        var data_color;
        var column_has_space;
        var tbody = $('<tbody>');
        if (data) {
            data = data.split('\n').map(function (value, index) {
                return value.split(',');
            });
            var out_of_range = false;
            data_color = new Array(data.length);
            var columns = 0;
            for (var i in data) {
                if (columns < data[i].length) {
                    columns = data[i].length;
                }
            }
            var column_has_space = new Array(columns);
            for (var i in data) {
                for (var j = 0; j < data[i].length; ++j) {
                    if (data[i][j].match(/ /)) {
                        column_has_space[j] = true;
                    }
                }
            }
            for (var i in data) {
                data_color[i] = new Array(data[i].length);
                for (var j in data[i]) {
                    var value = data[i][j];
                    if (value && !isNaN(value)) {
                        if (j in this.min && Number(value) < this.min[j]) {
                            out_of_range = true;
                            data_color[i][j] = this.config.csv.min_color;
                        } else if (j in this.max && Number(value) > this.max[j]) {
                            out_of_range = true;
                            data_color[i][j] = this.config.csv.max_color;
                        }
                        if (this.config.type == 'csv' && data_color[i][j]) {
                            data[i][j] = '<span style="color:' + data_color[i][j] + '">' + data[i][j] + '</span>';
                        }
                    }
                }
            }
            this.alert(this.config.csv.threshold_alert && out_of_range);
        } else {
            data = '&nbsp;'
        }
        if (this.config.type == 'csv-table') {
            if (typeof data === 'object') {
                for (var i in data) {
                    if (data[i].length == 1 && !data[i][0]) {
                        continue;
                    }
                    var tr = $('<tr>');
                    for (var j in data[i]) {
                        var td = $('<td>');
                        if (!column_has_space[j]) {
                            td.addClass('text-right');
                        }
                        var pre = $('<pre>');
                        if (data_color[i][j]) {
                            pre.css('color', data_color[i][j]);
                        }
                        tr.append(td.append(pre.text(data[i][j])));
                    }
                    tbody.append(tr);
                }
            } else {
                tbody.append('<tr><td colspan="' + this.target.find('thead th').length + '"><pre>' + data + '</pre></td></tr>');
            }
        } else {
            if (typeof data === 'object') {
                data = data.map(function (value, index) {
                    return value.join(',');
                }).join('\n');
            }
            if (data && data.length && data[data.length - 1] == '\n') {
                data = data.substring(0, data.length - 1);
            }
            tbody.append('<tr><td><pre>' + data + '</pre></td></tr>');
        }
        this.target.append(tbody);
        this.draw();
    };
    CsvFeed.prototype.draw = function () {
        while (this.target.find('tbody').length > this.config.csv.show_items) {
            this.target.find('tbody').first().remove();
        }
    };
    return CsvFeed;
});
