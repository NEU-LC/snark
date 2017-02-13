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

        StartStopFeed.prototype.addListeners = function () {
            var this_ = this;
            $($(this.form).find("input[type=text]")).on("input", function () {
                $(this_.form).find("button[name=start]").removeAttr('disabled');
                $(this_.form).find("button[name=clear]").removeAttr('disabled');
            });
            // var isValidForm = true;
            //
            // var keys = Object.keys(this.fields);
            // for (index in keys) {
            //     var key = keys[index];
            //     if (this.fields[key].length == 0) {
            //         isValidForm = false;
            //     }
            // }
            // if (!isValidForm) {
            //     update_ui_status(0, this.form);
            // }
            // $.ajax({
            //     context: this,
            //     url: this.get_url()
            // }).done(function (data, textStatus, jqXHR) {
            //     this.onload(data);
            // }).fail(function (jqXHR, textStatus, errorThrown) {
            //     this.onerror();
            // });
        };

        // StartStopFeed.prototype.load = function () {
        //     FormFeed.prototype.load.call(this);
        // };

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
                                        $(this_.form).find("button[name=start]").removeAttr("disabled");
                                        $(this_.form).find("button[name=clear]").removeAttr("disabled");
                                    }
                                    else {
                                        $(this_.form).find("button[name=stop]").removeAttr("disabled");
                                        $(this_.form).find("button[name=clear]").attr("disabled", "disabled");
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

        var update_ui_status = function (status, form) {
            if (status == "1") {
                $(form).find("input[type=text]").attr("readonly", "readonly"); //[value!='']
                $(form).find("button[name=start]").attr('disabled', "disabled");
                $(form).find("button[name=clear]").attr('disabled', "disabled");
                $(form).find("button[name=stop]").removeAttr('disabled');
            }
            else {
                $(form).find("button[name=stop]").attr('disabled', "disabled");
                $(form).find("button[name=clear]").removeAttr('disabled');
                $(form).find("button[name=start]").removeAttr('disabled');
                $(form).find("input[type=text]").removeAttr("readonly"); //[value!='']
            }
        };

        StartStopFeed.prototype.update_ui = function (data) {
            if (data.running != undefined && data.running.status != undefined) {
                update_ui_status(data.running.status, this.form);
                if (data.running.status == "1") {
                    if (data.fields != undefined) {
                        var keys = Object.keys(data.fields);
                        for (index in keys) {
                            var key = keys[index];
                            var input = $(this.form).find("input[name=" + key + "]");
                            input.val(data.fields[key]);
                            // if (input.val().length == 0) {
                            // }
                        }
                    }
                }
            }
        };
        return StartStopFeed;
    }
);
