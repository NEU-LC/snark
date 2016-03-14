define(function () {

    function GridOptions(options) {
        function AxisOptions(options, defaults) {
            Object.assign(this, options);
            this.min = this.min || defaults.min || 0;
            this.max = this.max || defaults.max || 10;
            this.step = this.step || defaults.step || 1;
            this.color = this.color || defaults.color || 'rgba(50, 75, 150, 0.9)';
        }

        function GridLineOptions(options) {
            Object.assign(this, options);
            this.show = 'show' in this ? this.show : true;
            this.color = this.color || 'rgba(100, 100, 100, 0.4)';
            this.width = this.width || 1;
        }

        function LabelOptions(options) {
            Object.assign(this, options);
            this.show = this.show || false;
            this.color = this.color || 'rgba(0, 0, 0, 0.9)';
            this.font = this.font || '18px sans-serif';
        }

        Object.assign(this, options);
        this.show = typeof options !== 'undefined';
        this.x = new AxisOptions(this.x, { color: 'rgba(255, 0, 0, 0.5)' });
        this.y = new AxisOptions(this.y, { color: 'rgba(0, 255, 0, 0.5)' });
        this.axis_width = this.axis_width || 2;
        this.step_length = this.step_length || 5;
        this.grid_lines = new GridLineOptions(this.grid_lines);
        this.x_offset = this.x_offset || 0;
        this.y_offset = this.y_offset || 0;
        this.labels = new LabelOptions(this.labels);
    }

    return GridOptions;

});
