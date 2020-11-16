Overview of applications supporting the Ouster lidar.

The currently supported and tested models are the OS1-16 and OS1-64.

The following applications are available. Use `<app_name>` `--help` for details.

`ouster-cat` - a simple interface to the tcp command port and udp data streams, outputs raw data

`ouster-to-csv` - takes raw data and converts to more structured csv (in binary). Works with both lidar and imu data

`ouster-align` - corrects for the small offsets in each beam to align images

`ouster-cv` - outputs a set of image fields as a single OpenCV image

`ouster-view` - display image and point-cloud data

Configuration
-------------
Many of the applications require a config file. This can be generated from the device with:
```
ouster-cat config --device <lidar host>
```
It's recommended that this is placed in an "ouster" section within a
`config.json` config file.

For simplicity the config option is not shown in the examples below.

Examples
--------
```
ouster-cat config --device os1-991832000987.local

fields=$( ouster-to-csv lidar --output-fields )
format=$( ouster-to-csv lidar --output-format )

ouster-cat lidar | ouster-to-csv lidar \
    | view-points --fields \$fields --binary \$format

ouster-cat lidar | ouster-to-csv | ouster-align | ouster-cv \
    | cv-cat "scale=60;resize=1.0,2.0;view;null"
```
There are many more examples contained in the help of each application. For instance, try:
```
ouster-to-csv --help --verbose
ouster-cv --help
```
