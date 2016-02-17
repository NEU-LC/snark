'use strict';

var stats = new Stats();
stats.domElement.style.position = 'absolute';
stats.domElement.style.left = '0px';
stats.domElement.style.top = '0px';

var options_3d = {
    config_url: 'models/config.json',
    websocket_url: 'ws://' + location.hostname + ':9999',
    host: location.hostname,
    port: 10000,
    packets: 1,
    camera: 'perspective',
    stats: true,
    interactive: true,
    start: function() {
        set_interactive(false);
        init_stream();
    },
    stop: function() {
        if (ws) {
            ws.close();
            ws = null;
        }
        set_interactive(this.interactive);
    },
    toggle: function() {
        return (ws && ws.readyState === WebSocket.OPEN) ? this.stop() : this.start();
    },
    reset: function() {
        for (var i in movable_meshes) {
            var mesh = movable_meshes[i];
            mesh.rotation.set(0, 0, 0);
            mesh.rotateOnAxis(mesh.config.rotation_axis_vector, mesh.config.angle_offset);
        }
    },
    axes: function() {
        axes.visible = !axes.visible;
    }
}

var config;

var ws;
function init_stream() {
    if (ws) {
        ws.close();
        ws = null;
    }
    ws = new WebSocket(options_3d.websocket_url);
    ws.binaryType = 'arraybuffer';
    ws.onopen = function() {
        console.log('WebSocket opened: ' + this.url);
        ws.send(JSON.stringify({
            host: options_3d.host,
            port: Number(options_3d.port),
            size: Float64Array.BYTES_PER_ELEMENT * config.data.order.length
        }));
        ws.send(options_3d.packets);
    }
    ws.onclose = function(e) {
        (e.code == 1000 ? console.log : console.error).call(console, 'WebSocket closed: Code: ' + e.code + (e.reason ? ' Reason: ' + e.reason : ''));
    }
    ws.onmessage = function(e) {
        ws.send(options_3d.packets);
        set_angles(e.data);
    }
}

var gui;
function init_gui() {
    gui = new dat.GUI({
        width: 400
    });
    gui_3d();
    $('.dg.ac').hover(
        function() {
            controls.enabled = false;
            set_interactive(false);
            document.removeEventListener('keypress', key_toggle);
        },
        function() {
            document.addEventListener('keypress', key_toggle, false);
            set_interactive(options_3d.interactive);
            controls.enabled = true;
        }
    );
}

function gui_3d() {
    var folder = gui.addFolder('3d');
    folder.open();
    folder.add(options_3d, 'websocket_url').name('websocket server').onFinishChange(function(value) {
        init_stream();
    });
    folder.add(options_3d, 'host');
    folder.add(options_3d, 'port');
    folder.add(options_3d, 'toggle').name('start/stop <kbd>spacebar</kbd>');
    folder.add(options_3d, 'reset');
    folder.add(options_3d, 'axes');
    folder.add(options_3d, 'camera', ['perspective', 'orthographic']).onFinishChange(function(value) {
        switch (options_3d.camera) {
            case 'perspective':
                perspective_camera.up.copy(orthographic_camera.up);
                perspective_camera.position.copy(orthographic_camera.position);

                camera = perspective_camera;
                controls = perspective_controls;
                break;
            case 'orthographic':
                orthographic_camera.up.copy(perspective_camera.up);
                orthographic_camera.position.copy(perspective_camera.position);

                // from threejs/examples/js/cameras/CombinedCamera.js
                // Switches to the Orthographic camera estimating viewport from Perspective

                var fov = perspective_camera.fov;
                var aspect = perspective_camera.aspect;
                var near = perspective_camera.near;
                var far = perspective_camera.far;

                // The size that we set is the mid plane of the viewing frustum

                var hyperfocus = ( near + far ) / 2;

                var halfHeight = Math.tan( fov * Math.PI / 180 / 2 ) * hyperfocus;
                var planeHeight = 2 * halfHeight;
                var planeWidth = planeHeight * aspect;
                var halfWidth = planeWidth / 2;

                halfHeight /= perspective_camera.zoom;
                halfWidth /= perspective_camera.zoom;

                orthographic_camera.left = - halfWidth;
                orthographic_camera.right = halfWidth;
                orthographic_camera.top = halfHeight;
                orthographic_camera.bottom = - halfHeight;

                camera = orthographic_camera;
                controls = orthographic_controls;
                break;
            default:
                console.log('unrecognised camera type: ' + options_3d.camera);
        }
    });
    folder.add(options_3d, 'interactive').onFinishChange(function(value) {
        if (options_3d.interactive) {
            if (!ws) {
                set_interactive(options_3d.interactive);
            }
        } else {
            set_interactive(options_3d.interactive);
        }
    });
    folder.add(options_3d, 'stats').onFinishChange(function(value) {
        if (options_3d.stats) {
            $(stats.domElement).show();
        } else {
            $(stats.domElement).hide();
        }
    });
}

var width = window.innerWidth;
var height = window.innerHeight;
var scene = new THREE.Scene();
//scene.fog = new THREE.Fog( 0x72645b, 2, 100 );

// ground
//var plane = new THREE.Mesh(
//    new THREE.PlaneBufferGeometry( 100, 100 ),
//    new THREE.MeshPhongMaterial( { color: 0x999999, specular: 0x101010 } )
//);
//plane.rotation.x = -Math.PI/2;
//plane.position.y = -0.5;
//scene.add( plane );
//plane.receiveShadow = true;

// lights
scene.add( new THREE.HemisphereLight( 0x443333, 0x111122 ) );
//addShadowedLight( 0.5, 1, -1, 0xffff99, 1 );
addShadowedLight( 5, 10, 1, 0xffffff, 1 );
addShadowedLight( -5, -10, 1, 0xffffff, 0.8 );
//scene.add( new THREE.AmbientLight( 0x808080 ) );

var renderer = new THREE.WebGLRenderer();
renderer.setSize(width, height);
//renderer.setClearColor( scene.fog.color );
//renderer.setPixelRatio( window.devicePixelRatio );
//renderer.setSize( window.innerWidth, window.innerHeight );
//renderer.gammaInput = true;
//renderer.gammaOutput = true;

var perspective_camera = new THREE.PerspectiveCamera(45, width/height, 0.1, 10000);

var orthographic_camera = new THREE.OrthographicCamera(-width/2, width/2, height/2, -height/2, 0.1, 10000);

var perspective_controls = new THREE.TrackballControls(perspective_camera);
perspective_controls.rotateSpeed = 2.5;
perspective_controls.zoomSpeed = 1.0;
perspective_controls.panSpeed = 1.0;

var orthographic_controls = new THREE.OrthographicTrackballControls(orthographic_camera);
orthographic_controls.rotateSpeed = 2.5;
orthographic_controls.zoomSpeed = 1.0;
orthographic_controls.enabled = false;

var camera = perspective_camera;
camera.position.set(1000, 1000, 1000);
camera.lookAt(new THREE.Vector3(-1, -1, -1));
camera.up = new THREE.Vector3(0, 0, 1);

var controls = perspective_controls;

var axes = new THREE.AxisHelper(500);
axes.visible = true;
scene.add(axes);

var raycaster = new THREE.Raycaster();

window.addEventListener('resize', function() {
    width = window.innerWidth;
    height = window.innerHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}, false);

function animate() {
    renderer.render(scene, camera);
    controls.update();
    stats.update();
    requestAnimationFrame(animate);
}

var meshes = {};
var movable_meshes = [];

function attach_children() {
    if (Object.keys(meshes).length != Object.keys(config.models).length) {
        return;
    }
    for (var i in meshes) {
        var mesh = meshes[i];
        var parent_url = mesh.config.parent_url;
        if (!parent_url) { continue; }
        var parent_mesh = meshes[parent_url];
        parent_mesh.add(mesh);
        parent_mesh.add(mesh.plane_mesh);
    }
}

var default_material = new THREE.MeshPhongMaterial( { color: 0xAAAAAA, specular: 0x111111, shininess: 200 } );
//var select_material = new THREE.MeshPhongMaterial( { color: 0x838DBA, specular: 0x111111, shininess: 200 } );
var select_material = new THREE.MeshPhongMaterial( { color: 0x93ADBA, specular: 0x111111, shininess: 200 } );
var plane_geometry = new THREE.PlaneBufferGeometry(10000, 10000);
var plane_material = new THREE.MeshBasicMaterial({ color:0x2378a0, wireframe: true, side: THREE.DoubleSide });

function load_model(url) {

    var loader = new THREE.STLLoader();
    loader.load(url, function (geometry) {

        var material = default_material;
        if (geometry.hasColors) {
            material = new THREE.MeshPhongMaterial({ opacity: geometry.alpha, vertexColors: THREE.VertexColors });
        }

        var mesh = new THREE.Mesh(geometry, material);

        mesh.config = config.models[url];
        mesh.url = url;
        
        var position = mesh.config.parent_url ? mesh.config.relative_position : mesh.config.position;
        if (position) {
            mesh.position.copy(position);
        }

        mesh.castShadow = true;
        mesh.receiveShadow = true;
        scene.add(mesh);

        if (mesh.config.rotation_axis_vector) {
            if (mesh.config.angle_offset) {
                mesh.rotateOnAxis(mesh.config.rotation_axis_vector, mesh.config.angle_offset);
            }
            mesh.plane_mesh = new THREE.Mesh(plane_geometry, plane_material);
            mesh.plane_mesh.visible = false;
            mesh.plane_mesh.lookAt(mesh.config.rotation_axis_vector);
            if (position) {
                mesh.plane_mesh.position.copy(position);
            }
            scene.add(mesh.plane_mesh);
            movable_meshes.push(mesh);
        }

        meshes[url] = mesh;

        attach_children();
    });
}

function addShadowedLight( x, y, z, color, intensity ) {

    var directionalLight = new THREE.DirectionalLight( color, intensity );
    directionalLight.position.set( x, y, z );
    scene.add( directionalLight );

    directionalLight.castShadow = true;
    // directionalLight.shadowCameraVisible = true;

    var d = 1;
    directionalLight.shadowCameraLeft = -d;
    directionalLight.shadowCameraRight = d;
    directionalLight.shadowCameraTop = d;
    directionalLight.shadowCameraBottom = -d;

    directionalLight.shadowCameraNear = 1;
    directionalLight.shadowCameraFar = 4;

    directionalLight.shadowMapWidth = 1024;
    directionalLight.shadowMapHeight = 1024;

    directionalLight.shadowBias = -0.005;

}

function key_toggle(e) {
    var key = typeof e.which === 'number' ? e.which : e.keyCode;
    if (key === 32) {
        options_3d.toggle();
    }
}

function set_angles(data) {
    var dataview = new DataView(data);
    var length = dataview.byteLength / Float64Array.BYTES_PER_ELEMENT;
    for (var i = 0; i < length; ++i) {
        var mesh = meshes[config.data.order[i]];
        if (!mesh) { continue; }
        var angle = dataview.getFloat64(i * Float64Array.BYTES_PER_ELEMENT, true);
        var quaternion = new THREE.Quaternion();
        quaternion.setFromAxisAngle(mesh.config.rotation_axis_vector, angle + mesh.config.angle_offset);
        mesh.quaternion.copy(quaternion);
    }
}

var selection = null;
var plane_angle_prev;

function get_mouse_position(event) {
    return new THREE.Vector2(
        (event.clientX / renderer.domElement.width) * 2 - 1,
        -(event.clientY / renderer.domElement.height) * 2 + 1
    );
}

function drag_start(event) {
    if (event.button != 0) {
        return;
    }

    var move_curr = get_mouse_position(event);

    raycaster.setFromCamera( move_curr, camera );

    // Find all intersected objects
    var intersects = raycaster.intersectObjects(movable_meshes);

    if (intersects.length > 0) {
        // Disable the controls
        controls.enabled = false;

        // Set the selection - first intersected object
        selection = intersects[0].object;
        selection.material = select_material;

        // Calculate angle on plane
        var intersects = raycaster.intersectObject(selection.plane_mesh);
        var point = selection.plane_mesh.worldToLocal(intersects[0].point);
        plane_angle_prev = Math.atan2(point.y, point.x);
    }
}

function drag_move(event) {
    if (!selection) {
        return;
    }
    event.preventDefault();

    var move_curr = get_mouse_position(event);

    raycaster.setFromCamera( move_curr, camera );

    // Calculate angle on plane
    var intersects = raycaster.intersectObject(selection.plane_mesh);
    var point = selection.plane_mesh.worldToLocal(intersects[0].point);
    var plane_angle = Math.atan2(point.y, point.x);

    // Calculate change in angle
    var delta = plane_angle - plane_angle_prev;
    if (delta > Math.PI/2) { delta -= 2 * Math.PI; }
    else if (delta < -Math.PI) { delta += 2 * Math.PI; }

    selection.rotateOnAxis(selection.config.rotation_axis, delta);

    plane_angle_prev = plane_angle;
}

function drag_stop(event) {
    if (selection) {
        selection.material = default_material;
        selection = null;
    }
    controls.enabled = true;
}

function set_interactive(enabled) {
    if (enabled) {
        document.addEventListener('mousedown', drag_start, false);
        document.addEventListener('mousemove', drag_move, false);
        document.addEventListener('mouseup', drag_stop, false);
    } else {
        document.removeEventListener('mousedown', drag_start);
        document.removeEventListener('mousemove', drag_move);
        document.removeEventListener('mouseup', drag_stop);
    }
}

function set_options(keys) {
    for (var i in keys) {
        var key = keys[i];
        var value = config.data[key];
        if (value) {
            options_3d[key] = value;
        }
    }
}

function prepare_config() {
    for (var i in config.models) {
        var model = config.models[i];
        if (model.rotation_axis) {
            model.rotation_axis_vector = (new THREE.Vector3()).copy(model.rotation_axis);
        }
        if (!model.parent_url) { continue; }
        if (model.relative_position) { continue; }
        var parent_model = config.models[model.parent_url];
        model.relative_position = {
            x: model.position.x - parent_model.position.x,
            y: model.position.y - parent_model.position.y,
            z: model.position.z - parent_model.position.z,
        }
    }
}

function load_models() {
    for (var i in config.models) {
        load_model(i);
    }
}

function load_config(url) {
    $.ajax({
        url: url
    }).done(function (data, textStatus, jqXHR) {
        config = data;
        set_options(['websocket_url', 'host', 'port']);
        init_gui();
        prepare_config();
        load_models();
    }).fail(function (jqXHR, textStatus, errorThrown) {
        console.log('failed to load config "' + url + '": ' + textStatus + ' ' + errorThrown);
    });
}

$(function() {
    document.body.appendChild(renderer.domElement);
    document.body.appendChild(stats.domElement);
    document.addEventListener('keypress', key_toggle, false);
    set_interactive(options_3d.interactive);
    requestAnimationFrame(animate);
    load_config(options_3d.config_url);
});
