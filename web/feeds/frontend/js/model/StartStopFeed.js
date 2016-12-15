/**
 * Created by vrushali on 15/10/15.
 */
define('StartStopFeed', ["jquery", "FormFeed"], function ($) {
    var FormFeed = require('FormFeed');
    var StartStopFeed = function (feed_name, feed_path, config) {
        this.base = FormFeed;
        this.base(feed_name, feed_path, config);
        this.start_btn = undefined;
        this.stop_btn = undefined;
        // this.buttons = [];
    };

    StartStopFeed.prototype = Object.create(FormFeed.prototype);

    StartStopFeed.prototype.add_buttons = function (container) {
        this.start_btn = add_button(this, "Start", "btn-primary col-sm-4", "start");
        this.stop_btn = add_button(this, "Stop", "btn-danger col-sm-3", "stop");
        container.append(this.start_btn).append($('<label/>',
            {class: "col-sm-1"})).append(this.stop_btn).append($('<label/>',
            {class: "col-sm-1"}))
    };

    var add_button = function (this_, name, style, command) {

        var btn = $('<button/>',
            {
                value: name,
                name: command,
                text: name,
                class: "btn " + style,
                click: function (event) {
                    event.preventDefault();
                    var url = this_.get_url();
                    if (url != undefined) {
                        $.ajax({
                            context: this,
                            data: $(this).closest("form").serialize() + "&" + $.param({'command': this.name}),
                            url: url
                        }).done(function (data, textStatus, jqXHR) {
                            var json = $.parseJSON(data);
                            this_.update_output(json);
                            if (json != undefined && json.status != undefined && json.status.code != undefined && json.status.code == 0) {
                                $(this).attr("disabled", "disabled");
                                var btn_name = $(this).attr("name");
                                if (btn_name == "stop") {
                                    $(this_.start_btn).removeAttr("disabled");
                                }
                                else {
                                    $(this_.stop_btn).removeAttr("disabled");
                                }
                            }
                        }).fail(function (jqXHR, textStatus, errorThrown) {
                            StartStopFeed.prototype.onerror();
                        });
                    }
                }
            });
        return btn;
    };


    StartStopFeed.prototype.update_ui = function (data) {
        if (data.running != undefined && data.running.status != undefined) {
            if (data.running.status == "1") {
                $(this.form).find("input[type=text]").attr("readonly", "readonly");
                $(this.form).find("button[name=start]").attr('disabled', "disabled");
                $(this.form).find("button[name=stop]").removeAttr('disabled');
            }
            else {
                $(this.form).find("button[name=stop]").attr('disabled', "disabled");
                $(this.form).find("button[name=start]").removeAttr('disabled');
                $(this.form).find("input[type=text]").removeAttr("readonly");
            }
        }
    };
    return StartStopFeed;
});
