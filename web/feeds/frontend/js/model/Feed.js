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
        this.input_container = $(this.id + ' .inputs-group');
        this.hint = this.config.hint != undefined ? this.config.hint : "";
        this.interval = null;
        this.refresh_time = null;
        this.show = true;
        this.isMobile = false;
        this.fields = [];
        if (config.form != undefined) {
            this.extract_fields(config.form);
        }


    };

    Feed.prototype.reset = function () {
        if (this.config.refresh.auto) {
            $(this.input_container).find("input[type=text]").attr("readonly", "readonly");
            $(this.input_container).find("input[type=text]").attr("title", "readonly");
            this.refresh();
        } else {
            this.clear_interval();
            $(this.input_container).find("input[type=text]").removeAttr("readonly");
            $(this.input_container).find("input[type=text]").removeAttr("title");
        }
    };

    Feed.prototype.extract_fields = function (form_elements) {
        if (form_elements != undefined) {

            if (form_elements['buttons'] != undefined) {
                if (form_elements['input'] != undefined) {
                    this.populate_path_values(form_elements['input'], "");
                }
                // this.buttons = form_elements['buttons'];
            }
            else {
                this.populate_path_values(form_elements, "");
            }
        }
    };

    Feed.prototype.populate_path_values = function (form_elements, prefix) {
        for (var element in form_elements) {
            var value = form_elements[element];
            var type = typeof value;
            if (type == "object") {
                var p = get_prefix(element, prefix);
                this.populate_path_values(value, p);
            }
            else if (type = "string") {
                var p = get_prefix(element, prefix);
                this.fields[p] = value;
            }
            // console.log(element + " typeof " + (typeof element) + "   type= " + type);
        }
    };

    var get_prefix = function (key, prefix) {
        var p;
        if (prefix.length > 0) {
            var type = typeof prefix;
            if (isNaN(key)) {
                p = prefix + "/" + key;
            }
            else {
                p = prefix + "[" + key + "]";
            }
        }
        else {
            p = key;
        }
        return p;
    };

    Feed.prototype.set_interval = function () {
        this.interval = setInterval(this.preload.bind(this), this.config.refresh.interval * 1000);
        this.status.removeClass('text-muted glyphicon-stop').addClass('text-success glyphicon-refresh');
        this.el.removeClass('panel-disabled').addClass('panel-enabled');
        var gui_folder = $(gui.__folders[this.feed_path].__ul);
        if (globals.isMobile) {
            gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').first().removeClass('panel-disabled').addClass('panel-enabled');
        } else {
            gui_folder.find('.title').first().removeClass('panel-disabled').addClass('panel-enabled');
        }
    };


    Feed.prototype.clear_interval = function () {
        clearInterval(this.interval);
        delete pending[this.feed_name];
        if (!this.target.hasClass('form')) {
            this.status.removeClass('text-success glyphicon-refresh').addClass('text-muted glyphicon-stop');
            this.el.removeClass('panel-enabled').addClass('panel-disabled');
            var gui_folder = $(gui.__folders[this.feed_path].__ul);
            if (globals.isMobile) {
                gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').removeClass('panel-enabled').addClass('panel-disabled');
            } else {
                gui_folder.find('.title').removeClass('panel-enabled').addClass('panel-disabled');
            }
        }
        else {
            // this.el.removeClass('panel-enabled').addClass('panel-disabled');
        }
    };
    Feed.prototype.refresh = function () {
        this.clear_interval();
        this.preload();
        if (this.config.refresh.auto) {
            this.set_interval();
        }
        if (this.hint != "") {
            $(this.target).attr("rel", "tooltip");
            $(this.target).attr("data-placement", "top");
            $(this.target).attr("data-html", "true");
            $(this.target).attr("data-toggle", "tooltip");
            $(this.target).attr("data-original-title", this.hint);
            $(this.target).tooltip();
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


    Feed.prototype.load_inputs = function (container) {
        var is_disabled = false;
        if (this.config.refresh.auto) {
            is_disabled = true;
        }
        for (var field in this.fields) {
            var row = $('<div>', {
                class: "form"
            });
            var label = $('<label>',
                {
                    text: field,
                    class: "col-sm-4 "
                });
            var each = $('<div>', {
                class: " col-sm-8"
            });
            var input = $('<input>',
                {
                    type: 'text',
                    class: "form-control ",
                    name: field,
                    value: this.fields[field]
                });
            if (is_disabled) {
                $(input).prop('readonly', "readonly");
                $(input).prop('title', "readonly");
            }
            each.append(input);
            row.append(label).append(each);
            container.append(row).append($('<div>', {class: "clear"}));
            // this.target.append(row);
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

    Feed.prototype.is_feed_inputs = function () {
        var size = Object.keys(this.fields).length;
        return size > 0 && this.config.type != "form";
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
        $.ajaxSetup({cache: false});
        if (this.is_feed_inputs()) {
            var params = $(this.input_container).find("form").serialize();
            url = url + (url.indexOf('?') < 0 ? '?' : '&') + params;
        }
        return url;//+ (url.indexOf('?') < 0 ? '?q=' : '&q=') + Math.random();

        // return url;+ (url.indexOf('?') < 0 ? '?q=' : '&q=') + Math.random();
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
