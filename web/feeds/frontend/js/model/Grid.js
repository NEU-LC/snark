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
                for (var x = x0 - step_pixels; x >= 0; x -= step_pixels) { xs.unshift(x); }
                for (var x = x0 + step_pixels; x <= context.canvas.width; x += step_pixels) { xs.push(x); }
                for (var i in xs) {
                    var x = xs[i];
                    if (grid_lines.show) {
                        draw_line(context, grid_style, [x, 0], [x, context.canvas.height]);
                    }
                    draw_line(context, step_style, [x, y0 - step_length], [x, y0 + step_length]);
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
                for (var y = y0 - step_pixels; y >= 0; y -= step_pixels) { ys.unshift(y); }
                for (var y = y0 + step_pixels; y <= context.canvas.height; y += step_pixels) { ys.push(y); }
                for (var i in ys) {
                    var y = ys[i];
                    if (grid_lines.show) {
                        draw_line(context, grid_style, [0, y], [context.canvas.width, y]);
                    }
                    draw_line(context, step_style, [x0 - step_length, y], [x0 + step_length, y]);
                }
            }
            draw_line(context, axis_style, [x0, 0], [x0, context.canvas.height]);
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
