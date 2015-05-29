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
                "interval": <number>            // refresh interval in seconds, optional, default: 2
                "auto": true|false              // auto refresh, optional, default: false
                                                // if a non-zero interval is given then auto is true unless specified
            },
            "text":                             // optional, only applicable when "type": "text"
            {
                "show_lines": <number>          // number of lines to show, optional, default: 5
            },
            "graph":                            // optional, only applicable when "type": "graph"
            {
                "min": <number>,                // min value, optional, default: 0
                "max": <number>,                // max value, optional, default: 100
                "units": "unit"                 // value units, optional
            },
            "stream":                           // optional, only applicable when "type": "stream"
            {
                "autoplay": true|false          // autoplay stream on load, default: false
            }
        },
        "<sensor-name>":
        {
        }
        ...
    }
}

see: examples/web.frontend.json
