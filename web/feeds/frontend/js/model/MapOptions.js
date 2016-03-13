define(function () {

    function MapOptions(options, global_options) {
        Object.assign(this, options);
        this.bing_maps_key = this.bing_maps_key || global_options.bing_maps_key || '';
        this.imagery_set = this.imagery_set || (this.image ? '' : 'Aerial');
        this.image = this.image || '';
        this.extent = this.extent || '';
        this.follow = this.follow || false;
        this.trail = this.trail || false;
        this.draw_interval = this.draw_interval || 100;
        this.alpha_step = this.alpha_step || 0;
        this.radius = this.radius || 5;
        this.fill = this.fill || '#3aee23';
        this.stroke = this.stroke || '#10a81a';
        this.stroke_width = this.stroke_width || 2;
    }

    return MapOptions;
});
