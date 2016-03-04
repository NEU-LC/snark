require.config({
    baseUrl: '',
    paths: {
        jquery: 'lib/jquery/dist/jquery.min',
        jquery_ui: 'lib/jquery-ui/jquery-ui.min',
        handlebars: 'lib/handlebars.min-latest',
        dat_gui: 'lib/dat-gui/build/dat.gui.snark',
        ol: 'lib/OpenLayers/build/ol-debug',
        Csv: 'js/Csv',
        utils: 'js/utils',
        MapApp: 'js/map/MapApp',
        CsvLayer: 'js/map/CsvLayer',
        ImageLayer: 'js/map/ImageLayer',
    }
});

var app;

require(['MapApp'], function(MapApp) {
    app = new MapApp('map');
});
