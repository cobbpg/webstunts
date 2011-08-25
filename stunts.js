var degToRad = Math.PI / 180;
var worldScale = 1;

var lightPosition = [0, 2000, 0];
var carModel = 4;
var wheelScale = 1.0 * worldScale;

var edgeLength = 0.3048 * 205 * worldScale; // each tile is 205-foot wide square
var edgeSize = 1024;
var hillHeight = 450;
var scaleFactor = edgeLength / edgeSize;
var carScaleFactor = scaleFactor / 20;
var carScaleY = 0.75;
var trackPosition = [edgeLength, 0, edgeLength]; // the mesh collider doesn't seem to like negative coordinates

var gl;
var jl;

var track;
var car;
var camera;

function notify(msg, tag) {
    if (!tag) {
	tag = 'p';
    }
    document.getElementById('notifications').innerHTML += '<' + tag + '>' + msg + '</' + tag + '>';
}

function vec(xyz) {
    return new Vector3D(xyz[0], xyz[1], xyz[2], 0);
}

function isElement(array, value) {
    return array.some(function(elem) { return elem == value });
}

function isGrass(material) {
    return isElement([16,101,102,103,104,105], material);
}

// Convert a Stunts model into a JTriangleMesh
function physicalModel(model) {
    var vertices = [];
    var indices = [];

    for (i = 0; i < model.vertices.length; i++) {
	vertices.push(vec(model.vertices[i]));
    }

    for (i = 0; i < model.faceIndices.length; i++) {
	var x = model.faceIndices[i];
	// Exclude road markings and corner kerbs from the physics
	if (!isElement([18,20,21,27,30,127,128], model.faceMaterials[i])) {
	    for (j = 1; j < x.length - 1; j++) {
		var t = { i0: x[0] };
		t.i1 = x[j + 1];
		t.i2 = x[j];
		indices.push(t);
	    }
	}
    }

    var skin = { vertices: vertices, indices: indices };
    var pos = vec(trackPosition);
    var rot = new jiglib.Matrix3D();
    var body = new jiglib.JTriangleMesh(skin, pos, rot, 200, 5 * worldScale);
    return body;
}

// Initialise a car given with the model number (0-10)
function initCar(carModel) {
    jl.setGravity(vec([0, -9.81 * worldScale, 0]));

    var carName = cars[carModel % 11];

    // Remove the previous car from the physical world
    if (car) {
	jl.removeBody(car.get_chassis());
    }

    // Initialise the car object
    car = new jiglib.JCar(null);
    car.name = carName;

    var wheelData = wheels[carName];

    var startInfo = track.startInfo;
    car.maxSteerAngle = 50;
    car.steerRate = 3;
    car.driveTorque = 2500 * worldScale * worldScale;

    car.setCar(car.maxSteerAngle, car.steerRate, car.driveTorque);

    // Calculate the bounding box of the car body to be used as the collision object
    var carVertices = models[carName].vertices;
    var xmin = 100000, xmax = -100000, ymin = 100000, ymax = -100000, zmin = 100000, zmax = -100000;
    for (i = 0; i < carVertices.length; i++) {
	xmin = Math.min(xmin, carVertices[i][0]);
	ymin = Math.min(ymin, carVertices[i][1]);
	zmin = Math.min(zmin, carVertices[i][2]);
	xmax = Math.max(xmax, carVertices[i][0]);
	ymax = Math.max(ymax, carVertices[i][1]);
	zmax = Math.max(zmax, carVertices[i][2]);
    }

    xmin *= carScaleFactor;
    ymin *= carScaleFactor;
    zmin *= carScaleFactor;
    xmax *= carScaleFactor;
    ymax *= carScaleFactor;
    zmax *= carScaleFactor;

    car.get_chassis().set_sideLengths(vec([xmax - xmin, (ymax - ymin) * carScaleY, zmax - zmin]));

    var startPosition = [(startInfo.x + 1) * edgeLength,
			 ymin + (ymax - ymin) * (1 - carScaleY) + wheelData[0].radius * wheelScale + 0.1,
			 (startInfo.y + 1) * edgeLength];
    if (startInfo.elevated) {
	startPosition[1] += hillHeight * scaleFactor;
    }

    car.get_chassis().moveTo(vec(startPosition));
    car.get_chassis().set_rotationY(startInfo.o * 90);
    car.get_chassis().set_mass(1000);

    // The chassis (internally a JBox) is the connection to the physics engine
    jl.addBody(car.get_chassis());

    // Set up wheels
    var travel = 0.2;
    var sideFriction = 1.2;
    var fwdFriction = 2.6;
    var restingFrac = 0.2;
    var dampingFrac = 0.8;
    var rays = 10;

    // The collision shape must be at the origin, so the positions of the wheels need to be adjusted accordingly
    car.centre = [xmin + xmax, ymin + ymax, zmin + zmax];
    vec3.add(car.centre, [0, (ymax - ymin) * (1 - carScaleY), 0]);
    vec3.scale(car.centre, -0.5);

    for (i = 0; i < wheelData.length; i++) {
	var pos = vec3.create(wheelData[i].position);
	vec3.scale(pos, worldScale);
	vec3.add(pos, car.centre);
	car.setupWheel(i, vec(pos), sideFriction, fwdFriction, travel, wheelData[i].radius * wheelScale, restingFrac, dampingFrac, rays);
    }

    // Initialise camera
    camera = {}

    camera.position = vec3.create([[0,0,100],[100,0,0],[0,0,-100],[-100,0,0]][startInfo.o]);
    vec3.add(camera.position, startPosition);

    // Initialise meshes
    var followDistance = vec3.length([xmax - xmin, ymax - ymin, zmax - zmin]) * 1.2;
    camera.distances = { near: followDistance, far: followDistance * 1.5 };

    var trans = mat4.create();
    mat4.identity(trans);
    mat4.scale(trans, [carScaleFactor, carScaleFactor, carScaleFactor]);
    car.chassisMesh = initMesh(mergeModels([{ trans: trans, model: cars[carModel] }]));
    car.wheelMesh = initMesh(models['wheel']);
}

// Initialise physics with an infinite plane
function initPhysics() {
    jl = jiglib.PhysicsSystem.getInstance();
    jl.setCollisionSystem(true, trackPosition[0], trackPosition[1], trackPosition[2], 60, 30, 60, edgeLength, edgeLength, edgeLength);
    jl.setSolverType("NORMAL");
    //jl.setSolverType("ACCUMULATED");

    //var ground = new jiglib.JPlane();
    //ground.set_y(0);
    //ground.set_rotationX(90);
    //ground.set_movable(false);
    //jl.addBody(ground);
}

// Initialise WebGL
function initGL(canvas) {
    try {
	gl = canvas.getContext("experimental-webgl");
	gl.viewportWidth = canvas.width;
	gl.viewportHeight = canvas.height;
    } catch(e) { }

    if (!gl) {
	notify("Could not initialise WebGL!");
    }

    gl.antiMoire = "#define ANTI_MOIRE_ENABLED\n";
    gl.getExtension("OES_standard_derivatives");
    var supported = gl.getSupportedExtensions();
    if (supported.indexOf("OES_standard_derivatives") < 0) {
	notify("OES_standard_derivatives not supported! Disabling Moiré reduction.");
	gl.antiMoire = "";
    }
}

// Compile the shader in the given element
function getShader(gl, id) {
    var shaderScript = document.getElementById(id);
    if (!shaderScript) {
	return null;
    }

    var str = "";
    var k = shaderScript.firstChild;
    while (k) {
	if (k.nodeType == 3) {
	    str += k.textContent;
	}
	k = k.nextSibling;
    }

    var shader;
    if (shaderScript.type == "x-shader/x-fragment") {
	shader = gl.createShader(gl.FRAGMENT_SHADER);
    } else if (shaderScript.type == "x-shader/x-vertex") {
	shader = gl.createShader(gl.VERTEX_SHADER);
    } else {
	return null;
    }

    gl.shaderSource(shader, gl.antiMoire + str);
    gl.compileShader(shader);

    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
	notify("Errors from shader <code>" + id + "</code>:");
	notify(gl.getShaderInfoLog(shader), "pre");
	return null;
    }

    return shader;
}

var shaderProgram;

// Initialise shaders
function initShaders() {
    var fragmentShader = getShader(gl, "stunts-fs");
    var vertexShader = getShader(gl, "stunts-vs");

    shaderProgram = gl.createProgram();
    gl.attachShader(shaderProgram, vertexShader);
    gl.attachShader(shaderProgram, fragmentShader);
    gl.linkProgram(shaderProgram);

    if (!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
	notify("Could not initialise shaders!");
    }

    gl.useProgram(shaderProgram);

    var attributes = ["vertexPosition", "vertexNormal", "vertexColour", "vertexMaterialInfo"];
    var uniforms = ["worldProjection", "normalProjection", "lightPosition"];

    for (i in attributes) {
	var a = attributes[i];
	shaderProgram[a] = gl.getAttribLocation(shaderProgram, a);
	gl.enableVertexAttribArray(shaderProgram[a]);
    }

    for (i in uniforms) {
	var u = uniforms[i];
	shaderProgram[u] = gl.getUniformLocation(shaderProgram, u);
    }
}

// Merge a list of models with associated transformations into a single model
function mergeModels(parts) {
    var vertices = [];
    var faceIndices = [];
    var faceMaterials = [];
    var faceBiases = [];
    var n = 0;
    var v = vec3.create();

    for (i = 0; i < parts.length; i++) {
	var trans = parts[i].trans;
	var model = models[parts[i].model];

	for (j = 0; j < model.vertices.length; j++) {
	    mat4.multiplyVec3(trans, model.vertices[j], v);
	    vertices.push([v[0],v[1],v[2]]);
	}

	for (j = 0; j < model.faceIndices.length; j++) {
	    faceIndices.push(model.faceIndices[j].map(function (x) { return x + n; }));
	    faceMaterials.push(model.faceMaterials[j]);
	    faceBiases.push(model.faceBiases[j]);
	}

	n += model.vertices.length;
    }

    return {
	vertices: vertices,
	faceIndices: faceIndices,
	faceMaterials: faceMaterials,
	faceBiases: faceBiases
    }
}

// Convert a Stunts mesh into buffers ready for rendering
function initMesh(model) {
    var vertices = model.vertices;
    var faceIndices = model.faceIndices;
    var faceMaterials = model.faceMaterials;
    var faceBiases = model.faceBiases;

    var flatNormals = [];
    var flatVertices = [];
    var flatColours = [];
    var flatMaterialInfo = [];

    for (index = 0; index < faceIndices.length; index++) {
	var ix = faceIndices[index];

	var u = vec3.create(vertices[ix[0]]);
	var v = vec3.create(vertices[ix[1]]);
	var w = vec3.create(vertices[ix[2]]);
	vec3.subtract(w, v);
	vec3.subtract(v, u);
	var normal = vec3.create();
	vec3.cross(w, v, normal);

	var material = materials[faceMaterials[index]]
	var rgb = material.rgb;
	rgb = [(rgb >> 16) / 255, ((rgb >> 8) & 0xff) / 255, (rgb & 0xff) / 255];

	var bias = isGrass(faceMaterials[index]) ? 2 : faceBiases[index];

	switch (bias) {
	case 0:
	    bias = 0;
	    break;
	case 1:
	    bias = -0.000005;
	    break;
	case 2:
	    bias = 0.00002;
	    break;
	}

	for (i = 1; i < ix.length - 1; i++) {
	    var js = [0, i, i+1];
	    for (j = 0; j < 3; j++) {
		for (k = 0; k < 3; k++) {
		    flatVertices.push(vertices[ix[js[j]]][k]);
		    flatNormals.push(normal[k]);
		    flatColours.push(rgb[k]);
		    switch (k) {
		    case 0:
			flatMaterialInfo.push(bias);
			break;
		    case 1:
			flatMaterialInfo.push(material.shininess);
			break;
		    case 2:
			var materialType = 1;
			if (material.type == 'Transparent') { materialType = 0; }
			if (material.type == 'Grate') { materialType = 0.5; }
			flatMaterialInfo.push(materialType);
			break;
		    }
		}
	    }
	}

    }

    var mesh = {};

    mesh.vertexPosition = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexPosition);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(flatVertices), gl.STATIC_DRAW);
    mesh.vertexPosition.itemSize = 3;
    mesh.vertexPosition.numItems = flatVertices.length / 3;

    mesh.vertexNormal = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexNormal);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(flatNormals), gl.STATIC_DRAW);
    mesh.vertexNormal.itemSize = 3;
    mesh.vertexNormal.numItems = flatNormals.length / 3;

    mesh.vertexColour = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexColour);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(flatColours), gl.STATIC_DRAW);
    mesh.vertexColour.itemSize = 3;
    mesh.vertexColour.numItems = flatColours.length / 3;

    mesh.vertexMaterialInfo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexMaterialInfo);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(flatMaterialInfo), gl.STATIC_DRAW);
    mesh.vertexMaterialInfo.itemSize = 3;
    mesh.vertexMaterialInfo.numItems = flatMaterialInfo.length / 3;

    return mesh;
}

var cameraProjection = mat4.create();
var worldProjection = mat4.create();
var normalProjection = mat3.create();

var projectedLight = vec3.create();
var tempPos = vec3.create();
var tempRot = mat4.create();

var t0;

// Render a mesh
function renderMesh(mesh) {
    mat4.toInverseMat3(worldProjection, normalProjection);
    mat3.transpose(normalProjection);

    gl.uniformMatrix4fv(shaderProgram.worldProjection, false, worldProjection);
    gl.uniformMatrix3fv(shaderProgram.normalProjection, false, normalProjection);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexPosition);
    gl.vertexAttribPointer(shaderProgram.vertexPosition, mesh.vertexPosition.itemSize, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexNormal);
    gl.vertexAttribPointer(shaderProgram.vertexNormal, mesh.vertexNormal.itemSize, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexColour);
    gl.vertexAttribPointer(shaderProgram.vertexColour, mesh.vertexColour.itemSize, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexMaterialInfo);
    gl.vertexAttribPointer(shaderProgram.vertexMaterialInfo, mesh.vertexMaterialInfo.itemSize, gl.FLOAT, false, 0, 0);

    gl.drawArrays(gl.TRIANGLES, 0, mesh.vertexPosition.numItems);
}

// Render the whole scene and update the world
function updateScene() {
    gl.viewport(0, 0, gl.viewportWidth, gl.viewportHeight);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // Update camera position
    var carBody = car.get_chassis();
    var carPos = carBody.get_currentState().position;
    vec3.set([carPos.x, carPos.y, carPos.z], tempPos);

    var camPos = vec3.create(camera.position);
    vec3.subtract(camPos, [0, 2 * worldScale, 0]);
    vec3.subtract(camPos, tempPos);
    vec3.normalize(camPos);
    vec3.scale(camPos, Math.max(camera.distances.near, Math.min(camera.distances.far, vec3.length(camPos))));
    vec3.add(camPos, [0, 2 * worldScale, 0]);
    vec3.add(camPos, tempPos, camera.position);
    camera.position[1] = Math.max(0.2, camera.position[1]);

    // Set camera matrix and light position
    mat4.perspective(45, gl.viewportWidth / gl.viewportHeight, 0.1, 50000, cameraProjection);
    mat4.multiply(cameraProjection, mat4.lookAt(camera.position, tempPos, [0, 1, 0]));

    mat4.multiplyVec3(cameraProjection, lightPosition, projectedLight);
    gl.uniform3fv(shaderProgram.lightPosition, projectedLight);

    // Render track
    mat4.set(cameraProjection, worldProjection);
    mat4.translate(worldProjection, trackPosition);

    renderMesh(track.mesh);

    // Render car chassis
    mat4.set(cameraProjection, worldProjection);
    vec3.set([carPos.x, carPos.y, carPos.z], tempPos);
    mat4.translate(worldProjection, tempPos);
    mat4.transpose(carBody.get_currentState().orientation.get_rawData(), tempRot);
    mat4.multiply(worldProjection, tempRot);
    mat4.set(worldProjection, cameraProjection); // Saving the transformation for the wheel children
    mat4.translate(worldProjection, car.centre);

    renderMesh(car.chassisMesh);

    // Render wheels
    var carWheels = car.get_wheels();
    var wheelData = wheels[car.name];
    for (i = 0; i < carWheels.length; i++) {
	var pos = carWheels[i].getActualPos();
	vec3.set([pos.x, pos.y, pos.z], tempPos);

	mat4.set(cameraProjection, worldProjection);
	mat4.translate(worldProjection, tempPos);
	mat4.rotateY(worldProjection, carWheels[i].getSteerAngle() * degToRad);
	mat4.rotateX(worldProjection, carWheels[i].getAxisAngle() * degToRad);
	mat4.scale(worldProjection, [wheelData[i].width * wheelScale, wheelData[i].radius * wheelScale, wheelData[i].radius * wheelScale]);

	renderMesh(car.wheelMesh);
    }

    // Update physics
    if (!t0) {
	t0 = new Date().getTime();
    } else {
	var n = 5;
	var t1 = new Date().getTime();
	var ms = Math.min(30, t1 - t0) / (750 * n);
	t0 = t1;

	var g = carBody.get_currentState().position.y >= 0 ? 1 : -0.2;
	jl.setGravity(vec([0, -9.81 * worldScale * g, 0]));

	var speed = car.get_chassis().get_currentState().linVelocity.get_length();
	car.setCar(car.maxSteerAngle, car.steerRate / Math.max(1, (speed - 10) * 0.5), car.driveTorque);
	for (i = 0; i < n; i++) {
	    jl.integrate(ms);
	}
    }
}

var scale = [scaleFactor, scaleFactor, scaleFactor];

function makeTrans(x, y, elevated, orientation) {
    var trans = mat4.create();

    mat4.identity(trans);
    mat4.scale(trans, scale);
    mat4.translate(trans, [edgeSize * x, elevated ? hillHeight : 0, edgeSize * y]);
    mat4.rotateY(trans, orientation * Math.PI * 0.5);

    return trans;
}

function buildTrack(codes) {
    var trackData = [];
    var terrainData = [];

    // Raw track and terrain data (tracks are vertically mirrored in the trk file!)
    for (y = 0; y < 30; y++) {
	var trackRow = [];
	var terrainRow = [];

	for (x = 0; x < 30; x++) {
	    trackRow.push(codes[(29 - y) * 30 + x]);
	    terrainRow.push(codes[y * 30 + x + 901]);
	}

	trackData.push(trackRow);
	terrainData.push(terrainRow);
    }

    // Replacements for roads on slopes
    var map07 = { 0x27: 0x67, 0x3B: 0x67, 0x62: 0x67, 0x04: 0xD0, 0x0E: 0xD4, 0x18: 0xD8 };
    var map08 = { 0x24: 0x68, 0x38: 0x68, 0x5F: 0x68, 0x05: 0xD1, 0x0F: 0xD5, 0x19: 0xD9 };
    var map09 = { 0x26: 0x67, 0x3A: 0x67, 0x61: 0x67, 0x04: 0xD2, 0x0E: 0xD6, 0x18: 0xDA };
    var map0A = { 0x25: 0x68, 0x39: 0x68, 0x60: 0x68, 0x05: 0xD3, 0x0F: 0xD7, 0x19: 0xDB };

    // Processed track items
    var trackItems = []
    for (y = 0; y < trackData.length; y++) {
	for (x = 0; x < trackData[y].length; x++) {
	    // Is it an elevated field (top of the hill)?
	    var e = terrainData[y][x] == 0x06;
	    var c = trackData[y][x];

	    switch (c) {
	    // Unnecessary fillers without a mesh (used to pad multi-tile items)
	    case 0x00: case 0xFD: case 0xFE: case 0xFF: break;
	    // Two-element items
	    case 0x65:
		trackItems.push({ x: x, y: y, elevated: e, id: 0x67 });
		trackItems.push({ x: x, y: y, elevated: e, id: 0x05 });
		break;
	    case 0x66:
		trackItems.push({ x: x, y: y, elevated: e, id: 0x68 });
		trackItems.push({ x: x, y: y, elevated: e, id: 0x04 });
		break;
	    // Everything else including slope replacements
	    default:
		switch (terrainData[y][x]) {
		case 0x07: if (map07[c] != undefined) { c = map07[c]; } break;
		case 0x08: if (map08[c] != undefined) { c = map08[c]; } break;
		case 0x09: if (map09[c] != undefined) { c = map09[c]; } break;
		case 0x0A: if (map0A[c] != undefined) { c = map0A[c]; } break;
		}
		trackItems.push({ x: x, y: y, elevated: e, id: c });
	    }
	}
    }

    // Filtered terrain items
    var terrainItems = []
    for (y = 0; y < terrainData.length; y++) {
	for (x = 0; x < terrainData[y].length; x++) {
	    var t = terrainData[y][x];
	    var r = trackData[y][x];
	    if (((t == 0x07 || t == 0x09) && (r == 0x04 || r == 0x0E || r == 0x18)) ||
		((t == 0x08 || t == 0x0A) && (r == 0x05 || r == 0x0F || r == 0x19))) {
		// This is a road on a slope, whose mesh contains the terrain
	    } else {
		terrainItems.push({ x: x, y: y, elevated: t == 0x06, id: t });
	    }
	}
    }

    var models = [];
    var physicalModels = [];

    function addModel(model) {
	models.push(model);
	physicalModels.push(model);
    }

    for (i = 0; i < terrainItems.length; i++) {
	var item = terrainItems[i];
	var model = terrainModels[item.id]; // [models,orientation]
	var orientation = model[1];

	var trans = makeTrans(item.x, item.y, item.elevated, orientation);
	for (j = 0; j < model[0].length; j++) {
	    models.push({ trans: trans, model: model[0][j] });
	}

	// Special-casing the lake in the physical model
	if (model[0][0] == 'lakc') {
	    physicalModels.push({ trans: makeTrans(item.x, item.y, item.elevated, orientation + 2), model: 'lakc' });
	} else if (model[0][0] == 'lake') {
	    continue;
	} else {
	    var offset = models.length - model[0].length;
	    for (j = 0; j < model[0].length; j++) {
		physicalModels.push(models[offset + j]);
	    }
	}
    }

    // Fence
    for (i = 0; i < 30; i++) {
	addModel({ trans: makeTrans(i, 0, false, 0), model: 'fenc' });
	addModel({ trans: makeTrans(i, 29, false, 2), model: 'fenc' });
	addModel({ trans: makeTrans(0, i, false, 1), model: 'fenc' });
	addModel({ trans: makeTrans(29, i, false, 3), model: 'fenc' });
    }

    var startInfo;

    for (i = 0; i < trackItems.length; i++) {
	var item = trackItems[i];
	var model = trackModels[item.id]; // [[models,orientation,width,height]
	var orientation = model[1];
	var x = item.x + 0.5 * (model[2] - 1);
	var y = item.y + 0.5 * (model[3] - 1);

	var trans = makeTrans(x, y, item.elevated, orientation);

	for (j = 0; j < model[0].length; j++) {
	    addModel({ trans: trans, model: model[0][j] });
	}

	// Start tile: three materials * four orientations
	if (isElement([0x01, 0x86, 0x93, 0xB3, 0x87, 0x94, 0xB4, 0x88, 0x95, 0xB5, 0x89, 0x96], item.id)) {
	    startInfo = { x: item.x, y: item.y, o: orientation, elevated: item.elevated };
	}
    }

    // Remove the previous track from the physical world
    if (track) {
	jl.removeBody(track.body);
    }

    // Initialise the graphics and physics for the new track
    track = {};
    track.startInfo = startInfo;
    track.mesh = initMesh(mergeModels(models));
    track.body = physicalModel(mergeModels(physicalModels));
    track.body.moveTo(vec(trackPosition));
    jl.addBody(track.body);

    // Reinitialise the car
    initCar(carModel);
}

function loadTrack(e) {
    var file = e.target.files[0];
    var reader = new FileReader();

    reader.onloadend = function() {
	if (this.result.length == 1802) {
	    var codes = []
	    for (i in this.result) {
		codes.push(this.result.charCodeAt(i));
	    }
	    buildTrack(codes);
	} else {
	    notify("<code>" + file.name + "</code> is not a Stunts track file.");
	}
    };
    reader.readAsBinaryString(file);
}

function start() {
    if (window.File && window.FileReader && window.FileList && window.Blob) {
	document.getElementById('track-file').addEventListener('change', loadTrack, false);
    } else {
	notify("This browser doesn't sopport the file api. You cannot load tracks.");
    }

    var canvas = document.getElementById("stunts-canvas");

    initGL(canvas);
    initShaders();
    initPhysics();
    buildTrack(defaultTrack);

    gl.clearColor(0.3608, 0.9882, 0.9882, 1.0);
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.GREATER);
    gl.clearDepth(0);

    document.onkeydown = function(e) {
	switch(e.keyCode) {
	case 32:
	    car.setHBrake(1);
	    return false;
	case 38:
	    car.setAccelerate(-1);
	    return false;
	case 40:
	    car.setAccelerate(1);
	    return false;
	case 37:
	    car.setSteer([0, 1], 1);
	    return false;
	case 39:
	    car.setSteer([0, 1], -1);
	    return false;
	case 67: // 'C' - pick the next car model
	    carModel++;
	    if (carModel > 10) {
		carModel = 0;
	    }
	case 82: // 'R' - reset the car
	    initCar(carModel);
	    return false;
	}
    }

    document.onkeyup = function(e) {
	switch(e.keyCode) {
	case 32:
	    car.setHBrake(0);
	    return false;
	case 38:
	    car.setAccelerate(0);
	    return false;
	case 40:
	    car.setAccelerate(0);
	    return false;
	case 37:
	    car.setSteer([0, 1], 0);
	    return false;
	case 39:
	    car.setSteer([0, 1], 0);
	    return false;
	}
    }

    setInterval(updateScene, 15);
}
