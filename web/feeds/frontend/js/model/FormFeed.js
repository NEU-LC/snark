/**
 * Created by vrushali on 15/10/15.
 */
define('FormFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');
    var space = $('<label/>',
        {class: "col-sm-2"});
    var FormFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        config.alert = false;
        this.base(feed_name, feed_path, config);
        this.fields = [];
        // this.buttons = [];
        this.form = $('<form>');
        this.extract_fields(config.form);
        this.ok_label = this.config.ok_label != undefined ? this.config.ok_label : "Submit";
        this.init();
        this.addListeners();

    };

    FormFeed.prototype = Object.create(Feed.prototype);

    FormFeed.prototype.init = function () {
        this.target.empty();
        this.form.empty();
        this.load_inputs(this.form);

        var this_ = this;

        // var combo = $('<select>');
        //
        // $("form").submit(function (event) {
        //     alert("Handler for .submit() called.");
        //     event.preventDefault();
        // });
        var buttons = $('<div>', {class: "col-sm-12"});
        this.add_buttons(buttons);
        // var num = this.buttons.length;

        var clear = $('<button/>',
            {
                text: 'Clear',
                type: 'button',
                class: "btn btn-default col-sm-3",
                click: function () {
                    $($(this).closest("form").find("input[type=text]")).each(function () {
                        $(this).val('');
                    });
                    $($(this).closest("form").find("button")).each(function () {
                        $(this).attr('disabled', "disabled");
                    });
                    // $($(this).closest("form").find("button")).each(function () {
                    //     $(this).attr('disabled', "disabled");
                    // });
                }
            });


        // form.append(combo).append("<br>");
        // form.append(input).append("<br>");
        buttons.append(clear);
        this.form.append(buttons);
        $(this.form).append($('<div>', {class: "clear"}));
        this.target.append(this.form);

        var size = Object.keys(this.fields).length;
        if (size > 0) {
            this.target.width(400);
        }
        else {
            $(this.target).css("min-width", function () {
                return 200;
            });
        }
        $(this.target).css("min-height", function () {
            return 100;
        });

    };
    FormFeed.prototype.add_buttons = function (container) {
        var submit = $('<button/>',
            {
                value: "submit",
                text: this.ok_label,
                class: "btn btn-primary col-sm-7",
                click: function (event) {
                    event.preventDefault();
                    var url = this_.get_url();
                    if (url != undefined) {
                        $.ajax({
                            context: this,
                            data: $(this).closest("form").serialize(),
                            url: url
                        }).done(function (data, textStatus, jqXHR) {
                            var json = $.parseJSON(data);
                            // FormFeed.prototype.onload_(data);
                            // data = JSON.parse('{"output" : {"message": "Done. success", "x": { "a": 0, "b": 1 } },"status" :{"code": 0 , "message": "Success. Added successfully."}}');
                            this_.update_output(json);
                        }).fail(function (jqXHR, textStatus, errorThrown) {
                            FormFeed.prototype.onerror();
                        });
                    }
                }
            });
        container.append(submit).append(space)
    };

    FormFeed.prototype.addListeners = function () {
        $($(this.form).find("input[type=text]")).on("input", function () {
            $($(this).closest("form").find("button")).each(function () {
                $(this).removeAttr('disabled');
            });
        });
        // $.ajax({
        //     context: this,
        //     url: this.get_url()
        // }).done(function (data, textStatus, jqXHR) {
        //     this.onload(data);
        // }).fail(function (jqXHR, textStatus, errorThrown) {
        //     this.onerror();
        // });
    };

    FormFeed.prototype.load = function () {
        $.ajax({
            context: this,
            url: this.get_url(),
            data: $(this.target).find("form").serialize(),
            timeout: globals.timeout
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
        // $.ajax({
        //     context: this,
        //     url: this.get_url()
        // }).done(function (data, textStatus, jqXHR) {
        //     this.onload(data);
        // }).fail(function (jqXHR, textStatus, errorThrown) {
        //     this.onerror();
        // });
    };
    FormFeed.prototype.update_output = function (data) {
        // this.target.height(100);
        // data = data.replace(/\n/g, '<br/>');
        this.el.removeClass('panel-disabled');
        $(this.form).find(".error-message").remove();
        $(this.form).find(".success-message").remove();
        $(this.form).find(".result-panel").remove();
        if (data.output != undefined) {
            var output = data.output;
            var panel = $('<div>', {class: "panel result-panel"});
            panel.append($('<label>', {class: "", text: "Output"}));
            panel.append($('<div>', {class: "form-results", text: JSON.stringify(output, null, 4)}));
            panel.append($('<div>', {class: "clearfix"}));
            $(this.form).append(panel);
            $(this.form).append($('<div>', {class: "clear"}));
        }
        if (data.status != undefined) {
            var status = data.status;
            if (status.code != undefined) {

                if (status.code != 0) {
                    this.el.addClass('panel-disabled');
                    $(this.form).append($('<label>', {
                        class: "error-message col-sm-11 ",
                        text: (status.message ? status.message : "Error.")
                    }));
                    $(this.form).append($('<div>', {class: "clear"}));
                }
                else {
                    $(this.form).append($('<label>', {
                        class: "success-message col-sm-11",
                        text: (status.message ? status.message : "Success. ")
                    }));
                    $(this.form).append($('<div>', {class: "clear"}));
                }
            }
        }

    };
    FormFeed.prototype.update_ui = function (data) {
    };
    FormFeed.prototype.onload_ = function (data) {
        if (data != undefined) {
            var json = $.parseJSON(data);
            if (json != undefined && json.status != undefined && json.status.code != undefined && json.status.code == 0) {
                this.update_ui(json);
            }
        }
    };

    return FormFeed;
});
