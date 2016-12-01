/**
 * Created by vrushali on 15/10/15.
 */
define('FormFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');
    var FormFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        config.alert = false;
        this.base(feed_name, feed_path, config);
        this.fields = [];
        this.buttons = [];
        this.form = $('<form>');
        this.extract_fields(config.form);
        this.ok_label = this.config.ok_label != undefined ? this.config.ok_label : "Submit";

    };

    FormFeed.prototype = Object.create(Feed.prototype);

    // FormFeed.prototype.extract_fields = function (form_elements) {
    //     if (form_elements != undefined) {
    //
    //         if (form_elements['buttons'] != undefined) {
    //             if (form_elements['input'] != undefined) {
    //                 this.populate_path_values(form_elements['input'], "");
    //             }
    //             this.buttons = form_elements['buttons'];
    //         }
    //         else {
    //             this.populate_path_values(form_elements, "");
    //         }
    //     }
    //     console.log(this.fields);
    // };


    // FormFeed.prototype.populate_path_values = function (form_elements, prefix) {
    //     for (var element in form_elements) {
    //         var value = form_elements[element];
    //         var type = typeof value;
    //         if (type == "object") {
    //             var p = get_prefix(element, prefix);
    //             this.populate_path_values(value, p);
    //         }
    //         else if (type = "string") {
    //             var p = get_prefix(element, prefix);
    //             this.fields[p] = value;
    //         }
    //         // console.log(element + " typeof " + (typeof element) + "   type= " + type);
    //     }
    // };

    // var get_prefix = function (key, prefix) {
    //     var p;
    //     if (prefix.length > 0) {
    //         var type = typeof prefix;
    //         if (isNaN(key)) {
    //             p = prefix + "/" + key;
    //         }
    //         else {
    //             p = prefix + "[" + key + "]";
    //         }
    //     }
    //     else {
    //         p = key;
    //     }
    //     return p;
    // };

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

        var num = this.buttons.length;
        var submit = $('<input/>',
            {
                value: this.ok_label,
                class: "btn btn-primary col-sm-7",
                click: function (event) {
                    event.preventDefault();
                    var url = this_.get_url();
                    if (url != undefined) {
                        $.ajax({
                            // context: this,
                            data: $(this).closest("form").serialize(),
                            url: url
                        }).done(function (data, textStatus, jqXHR) {
                            // FormFeed.prototype.onload_(data);
                            // data = JSON.parse('{"output" : {"message": "Done. success", "x": { "a": 0, "b": 1 } },"status" :{"code": 0 , "message": "Success. Added successfully."}}');
                            this_.onload(data);
                        }).fail(function (jqXHR, textStatus, errorThrown) {
                            FormFeed.prototype.onerror();
                        });
                    }
                }
            });

        var clear = $('<button/>',
            {
                text: 'Clear',
                type: 'button',
                class: "btn btn-default col-sm-3",
                click: function () {
                    $($(this).closest("form").find("input[type=text]")).each(function () {
                        $(this).val('');
                    });
                }
            });
        var space = $('<label/>',
            {class: "col-sm-2"});
        // form.append(combo).append("<br>");
        // form.append(input).append("<br>");
        buttons.append(submit).append(space).append(clear);
        this.form.append(buttons);
        $(this.form).append($('<div>', {class: "clear"}));
        this.target.append(this.form);
        if (this.fields.length > 0) {
            this.target.width(500);
        }
        else {
            $(this.target).css("min-height", function () {
                return 80;
            });
            $(this.target).css("min-width", function () {
                return 200;
            });
        }
    };

    FormFeed.prototype.load = function () {
        this.init();

        // $.ajax({
        //     context: this,
        //     url: this.get_url()
        // }).done(function (data, textStatus, jqXHR) {
        //     this.onload(data);
        // }).fail(function (jqXHR, textStatus, errorThrown) {
        //     this.onerror();
        // });
    };

    FormFeed.prototype.onload_ = function (data) {
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
    return FormFeed;
});
