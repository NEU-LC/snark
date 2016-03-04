place the frontend configuration files in the config/ directory.
configuration files must be in json format and end in '.json'.

web.frontend.json description:

{
    "host": "http://host:port",                 // default feed server, optional
                                                // if a feed entry in the "feeds" section does not have a url then the url is formed by <host>/<feed-name>
    "websocket": "ws://host:port/endpoint",     // default stream feed server, optional
    "bing_maps_key": <string>                   // required if using map imagery sets, Bing Maps API key. Get yours at http://bingmapsportal.com/.
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
                "alert": true|false             // optional, alert if values are out of range, default: true
            },
            "graph":                            // optional, only applicable when "type": "graph"
            {
                "min": <number>,                // optional, min value, default: 0
                "max": <number>,                // optional, max value, default: 100
                "units": "unit",                // optional, value units
                "thresholds": [                 // optional, array of thresholds
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
                                                             array: [min-x, min-y, max-x, max-y] (positive y is up)
                                                             string: url to world file
                "scale": <number>,              // optional, scale background image in percentage, default: 100
                "trail": true|false,            // optional, show track with trail, default: false
                "draw_interval": <number>,      // optional, canvas draw interval in milliseconds, default: 100
                "alpha_step": <number>,         // optional, alpha decrement step on each draw (fade out) where alpha between 0 and 1, default: 0
                "radius": <number>,             // optional, radius of each track point, default: 5
                "fill": <color>,                // optional, point fill color in hex format '#rrggbb', default: #10a81a
                "stroke": <color>,              // optional, point stroke color in hex format '#rrggbb', default: #3aee23
                "strokeWidth": <number>         // optional, point stroke width, 0 = no stroke, default: 2
            },
            "map":                              // optiona, only applicable when "type": "map"
            {
                "imagery_set": <string>         // optional, map imagery set options: "Aerial, "AerialWithLabels", "Road", default: "Aerial"
                "image": <url>,                 // optional, url to background image, may be relative path or http://...
                "extent": <array|string>,       // optional, extent of image, default: '' - input points are pixels
                                                             array: [min-x, min-y, max-x, max-y] (positive y is up), units: pixels
                                                             string: url to world file, units: degrees
                "follow": true|false,           // optional, follow input points, default: false
                "trail": true|false,            // optional, show track with trail, default: false
                "draw_interval": <number>,      // optional, canvas draw interval in milliseconds, default: 100
                "alpha_step": <number>,         // optional, alpha decrement step on each draw (fade out) where alpha between 0 and 1, default: 0
                "radius": <number>,             // optional, radius of each track point, default: 5
                "fill": <color>,                // optional, point fill color in hex format '#rrggbb', default: #10a81a
                "stroke": <color>,              // optional, point stroke color in hex format '#rrggbb', default: #3aee23
                "strokeWidth": <number>         // optional, point stroke width, 0 = no stroke, default: 2
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
