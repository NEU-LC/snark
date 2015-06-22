place the frontend configuration files in the config/ directory.
configuration files must be in json format and end in '.json'.

web.frontend.json description:

{
    "host": "http://host:port",                 // default sensor server, optional
                                                // if a sensor entry in the "sensors" section does not have a url
                                                // then the url is formed by <host>/<sensor-name>
    "websocket": "ws://host:port/endpoint",     // default stream sensor server, optional
    "sensors":
    {
        "<sensor-name>":
        {
            "type": "image"|"text"|"graph"      // optional, default: image
            "url": "http://host:port/path"      // optional
            "refresh":                          // optional
            {
                "interval": <number>            // optional, refresh interval in seconds, default: 2
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
                "units": "unit"                 // optional, value units
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
            "alert": true|false                 // optional, enable alerting, default: true
        },
        "<sensor-name>":
        {
        }
        ...
    }
}

see: examples/web.frontend.json
