// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

define(['jquery'], function ($) {

    var Grid = function (container, options, width, height) {
        this.container = container;
        this.options = options;
        this.width = width;
        this.height = height;
        this.init();
        this.draw();
    };
    Grid.prototype.init = function () {
        this.canvas = $('<canvas>', { class: 'grid' }).attr({ width: this.width, height: this.height });
        this.canvas.appendTo(this.container);
    }
    function draw_line(context, style, start, end) {
        if (style.strokeStyle) { context.strokeStyle = style.strokeStyle; }
        if (style.lineWidth) { context.lineWidth = style.lineWidth; }
        if (style.lineCap) { context.lineCap = style.lineCap; }
        context.beginPath();
        context.moveTo(start[0], start[1]);
        context.lineTo(end[0], end[1]);
        context.stroke();
    }
    function draw_text(context, style, text, x, y) {
        if (style.font) { context.font = style.font; }
        if (style.fillStyle) { context.fillStyle = style.fillStyle; }
        context.fillText(text, x, y);
    }
    Grid.prototype.draw = function() {
        var context = this.canvas[0].getContext('2d');
        context.clearRect(0, 0, context.canvas.width, context.canvas.height);
        var grid_lines = this.options.grid_lines;
        var grid_style = {
            strokeStyle: this.options.grid_lines.color,
            lineWidth: this.options.grid_lines.width
        };
        var axis_style = {
            lineWidth: this.options.axis_width
        }
        var step_style = {
            lineWidth: this.options.axis_width
        }
        var label_style = {
            fillStyle: this.options.labels.color,
            font: this.options.labels.font
        }
        var step_length = this.options.step_length;
        var x_axis = this.options.x;
        var y_axis = this.options.y;
        var y_range = y_axis.min < y_axis.max ? y_axis.max - y_axis.min : undefined;
        var x_range = x_axis.min < x_axis.max ? x_axis.max - x_axis.min : undefined;
        var x0 = context.canvas.width * (0 - x_axis.min) / x_range + this.options.x_offset;
        var y0 = context.canvas.height * (1 - (0 - y_axis.min) / y_range) + this.options.y_offset;
        // draw x axis
        if (!Number.isNaN(y0)) {
            axis_style.strokeStyle = step_style.strokeStyle = this.options.x.color;
            if (x_range && x_axis.step) {
                var step_pixels = context.canvas.width * (x_axis.step / x_range);
                var xs = []
                for (var x = x_axis.step, pixel = x0 + step_pixels; pixel.toFixed(10) <= context.canvas.width; x += x_axis.step, pixel += step_pixels ) { xs.push({ value: x, pixel: pixel }); }
                for (var x = -x_axis.step, pixel = x0 - step_pixels; pixel.toFixed(10) >= 0; x -= x_axis.step, pixel -= step_pixels ) { xs.unshift({ value: x, pixel: pixel }); }
                for (var i in xs) {
                    var x = xs[i];
                    if (grid_lines.show) {
                        draw_line(context, grid_style, [x.pixel, 0], [x.pixel, context.canvas.height]);
                    }
                    draw_line(context, step_style, [x.pixel, y0 - step_length], [x.pixel, y0 + step_length]);
                    if (this.options.labels.show) {
                        draw_text(context, label_style, x.value, x.pixel + 2, y0 - 5);
                    }
                }
            }
            draw_line(context, axis_style, [0, y0], [context.canvas.width, y0]);
        }
        // draw y axis
        if (!Number.isNaN(x0)) {
            axis_style.strokeStyle = step_style.strokeStyle = this.options.y.color;
            if (y_range && y_axis.step) {
                var step_pixels = context.canvas.height * (y_axis.step / y_range);
                var ys = []
                for (var y = -y_axis.step, pixel = y0 + step_pixels; pixel.toFixed(10) <= context.canvas.height; y -= y_axis.step, pixel += step_pixels ) { ys.push({ value: y, pixel: pixel }); }
                for (var y = y_axis.step, pixel = y0 - step_pixels; pixel.toFixed(10) >= 0; y += y_axis.step, pixel -= step_pixels ) { ys.unshift({ value: y, pixel: pixel }); }
                for (var i in ys) {
                    var y = ys[i];
                    if (grid_lines.show) {
                        draw_line(context, grid_style, [0, y.pixel], [context.canvas.width, y.pixel]);
                    }
                    draw_line(context, step_style, [x0 - step_length, y.pixel], [x0 + step_length, y.pixel]);
                    if (this.options.labels.show) {
                        draw_text(context, label_style, y.value, x0 + 5, y.pixel - 5);
                    }
                }
            }
            draw_line(context, axis_style, [x0, 0], [x0, context.canvas.height]);
        }
        if (this.options.labels.show && !Number.isNaN(x0) && !Number.isNaN(y0) && (x_axis.step || y_axis.step)) {
            draw_text(context, label_style, 0, x0 + 5, y0 - 5);
        }
    }
    Grid.prototype.remove = function () {
        if (this.canvas) {
            this.canvas.remove();
        }
        delete this.canvas;
    }
    return Grid;
});
