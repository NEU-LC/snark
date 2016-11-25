place the frontend configuration files in the config/ directory.
configuration files must be in json format and end in '.json'.

web.frontend.json description:

{
    "host": "http://host:port",                 // default feed server, optional
                                                // if a feed entry in the "feeds" section does not have a url then the url is formed by <host>/<feed-name>
    "websocket": "ws://host:port/endpoint",     // default stream feed server, optional
    "bing_maps_key": <string>,                  // required if using map imagery sets, Bing Maps API key. Get yours at http://bingmapsportal.com/.
    "timeout": <milliseconds>,                  // optional, feed timeout in ms to get new data; default: 60000
    "feeds":
    {
        "<feed-name>":
        {
            "type": <type>,                     // optional, type: "image", "text", "graph", "track", "map"; default: "image"
            "url": "http://host:port/path"      // optional
            "refresh":                          // optional
            {
                "interval": <number>,           // optional, refresh interval in seconds, 0 = refresh as soon as possible, default: 2
                "auto": true|false              // optional, auto refresh, default: false
                                                // if a non-zero interval is given then auto is true unless specified
            },
            "text":                             // optional, only applicable when "type": "text"
            {
                "show_lines": <number>          // optional, number of lines to show, default: 5
            },
            "csv":                              // optional, only applicable when "type": "csv"
            {
                "min": "<csv min values>",      // optional, csv list of min values, empty allowed e.g. min value of 3rd field: ,,40
                "min_color": "css-color",       // optional, color the value if it is below min, default: orange
                "max": "<csv max values>",      // optional, csv list of max values, empty allowed, e.g. max value of 5th and 7th fields: ,,,,100,,10
                "max_color": "css-color",       // optional, color the value if it is above max, default: red
                "threshold_alert": true|false   // optional, alert if values are out of range, default: false
            },
            "graph":                            // optional, only applicable when "type": "graph"
            {
                "min": <number>,                // optional, min value, default: 0
                "max": <number>,                // optional, max value, default: 100
                "units": "unit",                // optional, value units
                "thresholds":                   // optional, array of thresholds
                [
                    {
                        "value": <number>,      // graph values less than or equal to this value will be colored by the corresponding color
                        "color": <css-color>,
                        "alert": true|false     // optional, raise panel alert if graph value is within this threshold
                    }
                ]
            },
            "stream":                           // optional, only applicable when "type": "stream"
            {
                "autoplay": true|false          // autoplay stream on load, default: false
            },
            "track":                            // optional, only applicable when "type": "track"
            {
                "image": <url>,                 // optional, url to background image, may be relative path or http://...
                "extent": <array|string>,       // optional, extent of image, default: input points are pixels
                                                             csv: min-x, min-y, max-x, max-y (positive y is up)
                                                             string: url to world file
                "scale": <number>,              // optional, scale background image in percentage, default: 100
                "trail": true|false,            // optional, show track with trail, default: false
                "draw_interval": <number>,      // optional, canvas draw interval in milliseconds, default: 100
                "alpha_step": <number>,         // optional, alpha decrement step on each draw (fade out) where alpha between 0 and 1, default: 0
                "radius": <number>,             // optional, radius of each track point, default: 5
                "fill": <color>,                // optional, point fill color in hex format '#rrggbb', default: #10a81a
                "stroke": <color>,              // optional, point stroke color in hex format '#rrggbb', default: #3aee23
                "stroke_width": <number>        // optional, point stroke width, 0 = no stroke, default: 2
            },
            "map":                              // optional, only applicable when "type": "map"
            {
                "imagery_set": <string>         // optional, map imagery set options: "Aerial, "AerialWithLabels", "Road", default: "Aerial"
                "image": <url>,                 // optional, url to background image, may be relative path or http://...
                "extent": <array|string>,       // optional, extent of image, default: '' - input points are pixels
                                                             csv: min-x, min-y, max-x, max-y (positive y is up), units: pixels
                                                             string: url to world file, units: degrees
                "follow": true|false,           // optional, follow input points, default: false
                "trail": true|false,            // optional, show track with trail, default: false
                "draw_interval": <number>,      // optional, canvas draw interval in milliseconds, default: 100
                "alpha_step": <number>,         // optional, alpha decrement step on each draw (fade out) where alpha between 0 and 1, default: 0
                "radius": <number>,             // optional, radius of each track point, default: 5
                "fill": <color>,                // optional, point fill color in hex format '#rrggbb', default: #10a81a
                "stroke": <color>,              // optional, point stroke color in hex format '#rrggbb', default: #3aee23
                "stroke_width": <number>        // optional, point stroke width, 0 = no stroke, default: 2
            },
            "grid":                             // optional, if present display grid on panel, available to types: 'image', 'stream', 'track'
            {
                "axis_width": <number>,         // optional, x-/y-axis width, default: 2
                "step_length": <number>,        // optional, length of each step on the axis, default: 5
                "x_offset": <number>,           // optional, x offset in pixels, default: 0
                "y_offset": <number>,           // optional, y offset in pixels, default: 0
                "x":                            // optional, x-axis options
                {
                    "min": <number>,            // optional, min x value, default: 0
                    "max": <number>,            // optional, max x value, default: 10
                    "step": <number>,           // optional, x value steps relative to 0, default: 1
                    "color": <color>            // optional, x axis color, default: 'rgba(255, 0, 0, 0.5)' (red)
                },
                "y":                            // optional, y-axis options
                {
                    "min": <number>,            // optional, min y value, default: 0
                    "max": <number>,            // optional, max y value, default: 10
                    "step": <number>,           // optional, y value steps relative to 0, default: 1
                    "color": <color>            // optional, y axis color, default: 'rgba(0, 255, 0, 0.5)' (green)
                },
                "grid_lines":                   // optional, grid line options
                {
                    "show": true|false,         // optional, show grid lines, default: true
                    "color": <color>,           // optional, grid line color, default: 'rgba(100, 100, 100, 0.4)'
                    "width": <number>           // optional, grid line width, default: 1
                },
                "labels":                       // optional, label options
                {
                    "show": true|false,         // optional, show labels, default: true
                    "color": <color>,           // optional, label color, default: 'rgba(0, 0, 0, 0.9)'
                    "font": <font>              // optional, label font, default: '18px sans-serif'
                }
            },
            "alert": true|false                 // optional, enable alerting, default: true
        },
        "<feed-name>":
        {
        }
        ...
    }
}

see: examples/web.frontend.json


map.html

This page can display datasets over maps. Layers and map options can be specified through the controls on the page, a config file or through the URL query

config: configuration file of map options and layer definitions (default: 'config/map.json')

    query example:
        
        http://<web-server>/map.html?config=config/map-with-layers.json

imagery_set:        specifies which Bing Maps imagery set to use - 'Aerial', 'AerialWithLabels', 'Road' (default: 'Road')

bing_maps_key:      Bing Maps API key. Get yours at http://bingmapsportal.com

    query example:
        
        http://<web-server>/map.html?imagery_set=AerialWithLabels&bing_maps_key=<Your Bing Maps API Key>

layer: defines a layer, 2 types of layers are supported: image and data layers

    data layer

        data options
            filename:       url to data file
            fields:         the fields of the data file, in particular 'latitude' and 'longitude' are required
            binary:         if the data is binary, then the binary format of the fields (similar type specifiers used in comma)

        style options
            geometry:       geometry used to display datasets, 'line' or 'point' (default: 'line')
            fill:           fill color, applicable when geometry is 'point', rgba() or hex '#rrggbb' (default: rgba(255, 255, 255, 0.4))
                            when hex is used in the URL query then the '#' must be URI encoded: (i.e. '%23')
            stroke:         stroke color (default: #3399CC)
            stroke_width:   stroke width (default: 1.25)
            radius:         point radius (default: 5)

        query example:

            http://<web-server>/map.html?layer=filename=data/lat-lon.bin;fields=,latitude,longitude;binary=t,2d;fill=rgba(123,45,67,0.5)

    image layer

        image:              url to image file
        extent:             url to image extents file, can be csv (min-x,min-y,max-x,max-y) or World file
                            if none is provided, a default World file is assumed at <url to image file>w (e.g. if the image is 'images/sydney.png', assume extent file is 'images/sydney.pngw')

        query example:

            http://<web-server>/map.html?layer=image=images/sydney.png;extent=images/sydney.png.csv
            http://<web-server>/map.html?layer=image=images/sydney.png;extent=images/sydney.png.csv&layer=filename=data/lat-lon.csv;fields=latitude,longitude

example map config file:

{
    "imagery_set": "Road",
    "bing_maps_key": "Get yours at http://bingmapsportal.com",
    "layers":
    [
        {
            "image": "images/sydney.png"
        },
        {
            "filename": "data/lat-lon.bin",
            "fields": ",latitude,longitude",
            "binary": "t,2d"
            "geometry": "point",
            "fill": "rgba(235,115,115,0.6)",
            "stroke": "#8333cc",
            "stroke_width": 2,
            "radius": 3
        }
    ]
}
