'use strict';

var stats = new Stats();
stats.domElement.style.position = 'absolute';
stats.domElement.style.left = '0px';
stats.domElement.style.top = '0px';

var options_point_cloud = {
    websocket_url: 'ws://' + location.hostname + ':9999',
    host: location.hostname,
    port: 0,
    size: 12,
    color_scheme: 0,
    scans: 1,
    points: 60000,
    point_size: 0.15,
    height_min: -2.89654266502,
    height_range: 5.52007555834,
    distance_max: 110,
    stats: true,
    refresh: function() {
        is_refresh = true;
        init_stream(this.url);
    },
    start: function() {
        init_stream(this.url);
    },
    stop: function() {
        if (ws) {
            ws.close();
            ws = null;
        }
    },
    toggle: function() {
        return (ws && ws.readyState === WebSocket.OPEN) ? this.stop() : this.start();
    },
    clear: function() {
        while (point_clouds.length) {
            remove_point_cloud();
        }
    }
}

var ws;
var is_refresh = false;
function init_stream(url) {
    if (ws) {
        ws.close();
        ws = null;
    }
    ws = new WebSocket(options_point_cloud.websocket_url);
    ws.binaryType = 'arraybuffer';
    ws.onopen = function() {
        console.log('WebSocket opened: ' + this.url);
        ws.send(JSON.stringify({
            host: options_point_cloud.host,
            port: Number(options_point_cloud.port),
            size: Number(options_point_cloud.size)
        }));
        ws.send(options_point_cloud.points);
    }
    ws.onclose = function(e) {
        (e.code == 1000 ? console.log : console.error).call(console, 'WebSocket closed: Code: ' + e.code + (e.reason ? ' Reason: ' + e.reason : ''));
    }
    ws.onmessage = function(e) {
        ws.send(options_point_cloud.points);
        add_point_cloud(e.data);
        //if (is_refresh) { ws.onmessage = null; ws.close(); is_refresh = false; return; }
    }
}

var gui;
function init_gui() {
    gui = new dat.GUI({
        width: 400
    });
    gui_point_cloud();
}

function gui_point_cloud() {
    var folder = gui.addFolder('point cloud');
    folder.open();
    folder.add(options_point_cloud, 'websocket_url').name('websocket server').onFinishChange(function(value) {
        init_stream(options_point_cloud.url);
    });
    folder.add(options_point_cloud, 'host');
    folder.add(options_point_cloud, 'port');
    //folder.add(options_point_cloud, 'size');
    folder.add(options_point_cloud, 'points', 1, 60000).step(100);
    folder.add(options_point_cloud, 'point_size', 0.01, 0.3).name('point size').onChange(function(value) {
        point_cloud_material.size = options_point_cloud.point_size;
    });
    folder.add(options_point_cloud, 'scans', 1, 10).step(1);
    folder.add(options_point_cloud, 'color_scheme', [0,1,2]).name('color scheme').onFinishChange(function(value) {
        options_point_cloud.color_scheme = Number(value);
    });
    //folder.add(options_point_cloud, 'height_min', -4, -2).step(0.001).name('height min');
    //folder.add(options_point_cloud, 'height_range', 1, 6).step(0.001).name('height range');
    //folder.add(options_point_cloud, 'distance_max', 1, 150).step(1).name('distance max');
    //folder.add(options_point_cloud, 'refresh');
    folder.add(options_point_cloud, 'toggle').name('start/stop <kbd>spacebar</kbd>');
    folder.add(options_point_cloud, 'clear');
    folder.add(options_point_cloud, 'stats').onFinishChange(function(value) {
        if (options_point_cloud.stats) {
            $(stats.domElement).show();
        } else {
            $(stats.domElement).hide();
        }
    });
}

var width = window.innerWidth;
var height = window.innerHeight;
var scene = new THREE.Scene();
var renderer = new THREE.WebGLRenderer();
renderer.setSize(width, height);
var camera = new THREE.PerspectiveCamera(45, width/height, 0.1, 10000);
camera.position.z = 50;
camera.position.y = 25;
var controls = new THREE.OrbitControls(camera, renderer.domElement);

window.addEventListener('resize', function() {
    width = window.innerWidth;
    height = window.innerHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}, false);

function animate() {
    renderer.render(scene, camera);
    stats.update();
    requestAnimationFrame(animate);
}

var point_clouds = [];
var point_cloud_material = new THREE.PointCloudMaterial({
    size: options_point_cloud.point_size,
    vertexColors: THREE.VertexColors
});

function add_point_cloud(data) {
    var dataview = new DataView(data);
    var length = dataview.byteLength / Float32Array.BYTES_PER_ELEMENT;
    var positions = new Float32Array(length);
    var colors = new Float32Array(length);
    var color = new THREE.Color();
    var height_min = options_point_cloud.height_min;
    var height_range = options_point_cloud.height_range;
    var distance_max = options_point_cloud.distance_max;
    for (var i = 0, i_1, i_2; i < length; ++i) {
        positions[i] = dataview.getFloat32(i * Float32Array.BYTES_PER_ELEMENT, true);
        if (i % 3 == 2) {
            i_1 = i - 1;
            i_2 = i - 2;
            var x =  positions[i_2];
            var y = -positions[i  ];
            var z =  positions[i_1];
            positions[i_1] = y;
            positions[i  ] = z;
            var distance = (new THREE.Vector2(x, z)).length();
            var h;
            switch (options_point_cloud.color_scheme) {
                case 0:
                    h = (y - height_min) / height_range * 2.5;
                    break;
                case 1:
                    h = 1 - (distance / distance_max * 4);
                    break;
                default:
                    h = (Math.atan2(z, x) + Math.PI) / (Math.PI * 2);
                    break;
            }
            var s = 1 - (distance / distance_max);
            var l = (y - height_min) / height_range * s;
            color.setHSL(h, s, l);
            colors[i_2] = color.r;
            colors[i_1] = color.g;
            colors[i  ] = color.b;
        }
    }
    var geometry = new THREE.BufferGeometry();
    geometry.addAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.addAttribute('color', new THREE.BufferAttribute(colors, 3));
    //geometry.computeBoundingSphere();
    var p = new THREE.PointCloud(geometry, point_cloud_material);
    while (point_clouds.length >= options_point_cloud.scans) {
        remove_point_cloud();
    }
    scene.add(p);
    point_clouds.push(p);
}

function remove_point_cloud() {
    scene.remove(point_clouds.shift());
}

function key_toggle(e) {
    var key = typeof e.which === 'number' ? e.which : e.keyCode;
    if (key === 32) {
        options_point_cloud.toggle();
    }
}

$(function() {
    init_gui();
    document.body.appendChild(renderer.domElement);
    document.body.appendChild(stats.domElement);
    document.addEventListener('keypress', key_toggle, false);
    $(':input').focusin(function() { document.removeEventListener('keypress', key_toggle); });
    $(':input').focusout(function() { document.addEventListener('keypress', key_toggle, false); });
    requestAnimationFrame(animate);
});
