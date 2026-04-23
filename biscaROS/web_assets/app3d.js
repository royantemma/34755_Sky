import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x303030);

const camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 10, 50000);
camera.up.set(0, 0, 1);
camera.position.set(2000, -4000, 4000);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.shadowMap.enabled = true;
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 1);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 10);
dirLight.position.set(5000, 5000, 2000);
dirLight.castShadow = true;
scene.add(dirLight);

let robotModel = new THREE.Group();
scene.add(robotModel);

const geometry = new THREE.BoxGeometry(200, 300, 150);
const material = new THREE.MeshLambertMaterial({ color: 0xff0000 });
const placeholderMesh = new THREE.Mesh(geometry, material);
placeholderMesh.position.z = 75;
robotModel.add(placeholderMesh);

const loader = new FBXLoader();

loader.load('3D/map.fbx', function (object) {
    scene.add(object);
}, undefined, function (error) {
    console.log('Map model missing. Export to "stream_server/3D/map.fbx".', error);
});

loader.load('3D/robot.fbx', function (object) {
    robotModel.remove(placeholderMesh);
    // Correct the intrinsic 180 degree yaw offset of the base robot model
    object.rotation.z = Math.PI; 
    robotModel.add(object);
}, undefined, function (error) {
    console.log('Robot model missing. Export to "stream_server/3D/robot.fbx".', error);
});

const inputs = {
    x: document.getElementById('posX'),
    y: document.getElementById('posY'),
    z: document.getElementById('posZ'),
    r: document.getElementById('rotRoll'),
    p: document.getElementById('rotPitch'),
    yw: document.getElementById('rotYaw'),
};

function updateRobotPose() {
    if (!robotModel) return;

    robotModel.position.x = parseFloat(inputs.x.value) || 0;
    robotModel.position.y = parseFloat(inputs.y.value) || 0;
    robotModel.position.z = parseFloat(inputs.z.value) || 0;

    const rollRad = THREE.MathUtils.degToRad(parseFloat(inputs.r.value) || 0);
    const pitchRad = THREE.MathUtils.degToRad(parseFloat(inputs.p.value) || 0);
    const yawRad = THREE.MathUtils.degToRad(parseFloat(inputs.yw.value) || 0);

    robotModel.rotation.set(rollRad, pitchRad, yawRad, 'XYZ'); 
}

Object.values(inputs).forEach(input => {
    input.addEventListener('input', updateRobotPose);
});

document.getElementById('resetBtn').addEventListener('click', () => {
    Object.values(inputs).forEach(input => input.value = 0);
    updateRobotPose();
});

window.addEventListener('resize', () => {
    if(container.clientWidth > 0) {
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }
});

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

animate();

// Helper to watch container resize more reliably
const resizeObserver = new ResizeObserver(() => {
    if(container.clientWidth > 0) {
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }
});
resizeObserver.observe(container);


// Listen for automatic pose updates from ArUco estimations
window.addEventListener('robotPoseUpdate', (e) => {
    if (!robotModel) return;
    
    // Only auto-update if the user hasn't explicitly focused/edited the manual inputs recently
    // If they want automatic updates to stop bouncing around, they can pause tracking, but we'll 
    // update the visual instantly. We'll update the text boxes too so they match reality.
    
    const isFocused = Object.values(inputs).some(i => document.activeElement === i);
    if (isFocused) return; // Don't snatch focus or values while user is typing
    
    const { x, y, yaw } = e.detail;
    
    // Convert yaw (radians) to degrees for UI, keep Pitch and Roll 0
    inputs.x.value = Math.round(x);
    inputs.y.value = Math.round(y);
    inputs.z.value = 0;
    inputs.yw.value = Math.round(yaw * 180 / Math.PI);
    inputs.r.value = 0;
    inputs.p.value = 0;
    
    updateRobotPose();
});
