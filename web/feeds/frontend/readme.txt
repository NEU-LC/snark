place the frontend configuration files in the config/ directory.
configuration files must be in json format and end in '.json'.

web.frontend.json description:

{
    "host": "http://host:port",                 // default feed server, optional
                                                // if a feed entry in the "feeds" section does not have a url
                                                // then the url is formed by <host>/<feed-name>
    "websocket": "ws://host:port/endpoint",     // default stream feed server, optional
    "feeds":
    {
        "<feed-name>":
        {
            "type": <type>,                     // optional, type: "image", "text", "graph", "track"; default: "image"
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
            "track":                            // optiona, only applicable when "type": "track"
            {
                "background_url": <url>,        // optional, url to background image, may be relative path or http://...
                "draw_interval": <number>,      // optional, canvas draw interval in milliseconds, default: 100
                "alpha_step": <number>,         // optional, alpha decrement step on each draw (fade out), default: 0.05
                "fill": <color>,                // optional, point fill color in hex format '#rrggbb' or RGB16 array [r,g,b], default: [57,220,31]
                "stroke": <color>,              // optional, point stroke color in hex format '#rrggbb' or RGB16 array [r,g,b], default: [23,140,45]
                "strokeWidth": <number>         // optional, point stroke width, 0 = no stroke, default: 1
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
