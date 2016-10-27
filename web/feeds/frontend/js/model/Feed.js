/**
 * Created by vrushali on 15/10/15.
 */
define('Feed', ["jquery", "jquery_timeago", "utils"], function ($) {
    var Feed = function (feed_name, feed_path, config) {
        this.feed_name = feed_name;
        this.feed_path = feed_path;
        this.config = config;
        this.id = '#' + feed_path_to_id(this.feed_name);
        this.el = $(this.id);
        this.compact_icon = $(this.id + ' .panel-compact span');
        this.body = $(this.id + ' .panel-body');
        this.panel_refresh = $(this.id + ' .panel-refresh');
        this.status = $(this.id + ' .status');
        this.timestring = $(this.id + ' .timestring').hide();
        this.timeago = $(this.id + ' .timeago').timeago();
        this.target = $(this.id + ' .target');
        this.interval = null;
        this.refresh_time = null;
        this.show = true;
        this.isMobile = false;
    };

    Feed.prototype.reset = function () {
        if (this.config.refresh.auto) {
            this.refresh();
        } else {
            this.clear_interval();
        }
    };
    Feed.prototype.set_interval = function () {
        this.interval = setInterval(this.preload.bind(this), this.config.refresh.interval * 1000);
        this.status.removeClass('text-muted').addClass('text-success');
        this.el.addClass('panel-enabled');
        var gui_folder = $(gui.__folders[this.feed_name].__ul);
        if (globals.isMobile) {
            gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').first().addClass('panel-enabled');
        } else {
            gui_folder.find('.title').first().addClass('panel-enabled');
        }
    };


    Feed.prototype.clear_interval = function () {
        clearInterval(this.interval);
        delete pending[this.feed_name];
        this.status.removeClass('text-success').addClass('text-muted');
        this.el.removeClass('panel-enabled');
        var gui_folder = $(gui.__folders[this.feed_name].__ul);
        if (globals.isMobile) {
            gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').removeClass('panel-enabled');
        } else {
            gui_folder.find('.title').removeClass('panel-enabled');
        }
    };
    Feed.prototype.refresh = function () {
        this.clear_interval();
        this.preload();
        if (this.config.refresh.auto) {
            this.set_interval();
        }
    };
    Feed.prototype.update_view = function () {
        if (this.config.view == 'show') {
            this.el.show();
            this.body.show();
            this.compact_icon.removeClass('glyphicon-resize-full').addClass('glyphicon-resize-small');
        } else if (this.config.view == 'compact') {
            this.el.show();
            this.body.hide();
            this.compact_icon.removeClass('glyphicon-resize-small').addClass('glyphicon-resize-full');
        } else if (this.config.view == 'hide') {
            this.el.hide();
        }
    };
    Feed.prototype.preload = function () {
        if (pending[this.feed_name]) {
            return;
        }
        pending[this.feed_name] = true;
        this.refresh_time = new Date();
        this.status.fadeTo(1000, 0.4);
        this.load();
    };
    Feed.prototype.update_time = function () {
        var timestring = this.refresh_time.toString();
        //alert(timestring);
        var timezone = timestring.match(/\((.*)\)$/);
        if (timezone != undefined && timezone.length == 2 && timezone[1].match(/ /)) {
            var short_timezone = timezone[1].split(' ').map(function (word, index) {
                return word[0];
            }).join('');
            timestring = timestring.replace(/\(.*\)$/, '(' + short_timezone + ')');
        }
        this.timestring.text(timestring);
        this.timeago.attr('datetime', this.refresh_time.toISOString()).timeago('updateFromDOM');
    };
    Feed.prototype.onload = function (data) {
        this.update_time();
        this.status.finish().fadeTo(0, 1);
        if (this.config.alert) {
            this.alert(!data || !data.length);
        }
        this.onload_(data);
        delete pending[this.feed_name];
    };
    Feed.prototype.onerror = function () {
        this.update_time();
        this.status.finish().fadeTo(0, 1);
        this.onload_();
        delete pending[this.feed_name];
        if (this.config.alert) {
            this.alert(true);
        }
    };
    Feed.prototype.get_url = function () {
        var url = this.config.url;
        return url + (url.indexOf('?') < 0 ? '?q=' : '&q=') + Math.random();
    };
    Feed.prototype.alert = function (on) {
        var gui_element = gui.__folders[this.feed_path];
        if (gui_element != undefined) {
            var gui_folder = $(gui_element.__ul);
            if (on) {
                this.el.addClass('panel-alert');
                if (globals.isMobile) {
                    gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').first().addClass('panel-alert');
                }
                else {
                    gui_folder.find('.title').first().addClass('panel-alert');
                }

                if (globals.alert_beep) {
                    globals.beep();
                }
            } else {
                if (globals.isMobile) {

                    gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').removeClass('panel-alert');
                }
                else {
                    gui_folder.find('.title').removeClass('panel-alert');
                }
                this.el.removeClass('panel-alert');
            }
        }
    };
    Feed.views = ['show', 'compact', 'hide'];

    return Feed;

});
