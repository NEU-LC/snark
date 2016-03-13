define(function () {

    function TrackOptions(options) {
        Object.assign(this, options);
        this.image = this.image || '';
        this.extent = this.extent || '';
        this.scale = this.scale || 100;
        this.trail = this.trail || false;
        this.draw_interval = this.draw_interval || 100;
        this.alpha_step = this.alpha_step || 0;
        this.radius = this.radius || 5;
        this.fill = this.fill || '#3aee23';
        this.stroke = this.stroke || '#10a81a';
        this.stroke_width = this.stroke_width || 2;
    }

    return TrackOptions;
});
