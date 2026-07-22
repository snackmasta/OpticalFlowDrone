// ----------------------------------------------------
// 6DoF VR Controller 3D Trajectory Visualizer - Three.js
// ----------------------------------------------------

let scene, camera, renderer, controls;
let controllerGroup, trajectoryLine, gridHelper;
let trajectoryPoints = [];
const MAX_TRAJECTORY_POINTS = 1000;

// Origin offset for resetting zero-position
let positionOffset = { x: 0, y: 0, z: 0 };
let rawLatestPos = { x: 0, y: 0, z: 0 };
let currentTargetPos = { x: 0, y: 0, z: 0 };
let currentTargetQuat = new THREE.Quaternion();


// Setup DOM elements
const elStatusBadge = document.getElementById('statusBadge');
const elStatusText = document.getElementById('statusText');
const elPosX = document.getElementById('posX');
const elPosY = document.getElementById('posY');
const elPosZ = document.getElementById('posZ');
const elRoll = document.getElementById('rotRoll');
const elPitch = document.getElementById('rotPitch');
const elYaw = document.getElementById('rotYaw');
const elQw = document.getElementById('qw');
const elQx = document.getElementById('qx');
const elQy = document.getElementById('qy');
const elQz = document.getElementById('qz');
const elHeading = document.getElementById('headingVal');
const elSpeed = document.getElementById('speedVal');

// Initialize 3D Scene
function initScene() {
  const canvas = document.getElementById('canvas3d');
  
  // Scene (No fog, so objects never fade out at distance)
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x090d16);

  // Camera (Far plane expanded to 100,000 to prevent clipping)
  camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 100000);
  camera.position.set(1.5, 1.2, 2.0);

  // Renderer
  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;

  // Orbit Controls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, 0, 0);

  // Lighting
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0x06b6d4, 1.2);

  dirLight.position.set(3, 5, 2);
  dirLight.castShadow = true;
  scene.add(dirLight);

  const pointLight = new THREE.PointLight(0xec4899, 0.8, 10);
  pointLight.position.set(-2, 2, -2);
  scene.add(pointLight);

  // Grid
  gridHelper = new THREE.GridHelper(10, 20, 0x06b6d4, 0x1e293b);
  gridHelper.position.y = -0.5;
  scene.add(gridHelper);

  // Build VR Controller 3D Mesh
  createControllerMesh();

  // Create Trajectory Line
  createTrajectoryLine();

  // Window Resize Listener
  window.addEventListener('resize', onWindowResize);

  // Start Animation Loop
  animate();
}

// Construct Stylized 3D VR Controller Model
function createControllerMesh() {
  controllerGroup = new THREE.Group();

  // Handle (Cylinder)
  const handleGeo = new THREE.CylinderGeometry(0.04, 0.035, 0.22, 16);
  const handleMat = new THREE.MeshStandardMaterial({
    color: 0x1e293b,
    roughness: 0.3,
    metalness: 0.8
  });
  const handleMesh = new THREE.Mesh(handleGeo, handleMat);
  handleMesh.rotation.x = Math.PI / 6;
  handleMesh.position.set(0, -0.05, 0);
  controllerGroup.add(handleMesh);

  // Tracking Ring (Torus)
  const ringGeo = new THREE.TorusGeometry(0.08, 0.012, 16, 32);
  const ringMat = new THREE.MeshStandardMaterial({
    color: 0x06b6d4,
    emissive: 0x06b6d4,
    emissiveIntensity: 0.4,
    roughness: 0.2
  });
  const ringMesh = new THREE.Mesh(ringGeo, ringMat);
  ringMesh.rotation.x = Math.PI / 3;
  ringMesh.position.set(0, 0.06, 0.04);
  controllerGroup.add(ringMesh);

  // Joystick (Sphere + Shaft)
  const stickGeo = new THREE.SphereGeometry(0.015, 16, 16);
  const stickMat = new THREE.MeshStandardMaterial({ color: 0xec4899 });
  const stickMesh = new THREE.Mesh(stickGeo, stickMat);
  stickMesh.position.set(0, 0.04, 0);
  controllerGroup.add(stickMesh);

  scene.add(controllerGroup);
}


// Initialize Dynamic 3D Trajectory Ribbon Line
function createTrajectoryLine() {
  const lineGeo = new THREE.BufferGeometry();
  const positions = new Float32Array(MAX_TRAJECTORY_POINTS * 3);
  lineGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  lineGeo.setDrawRange(0, 0);

  const lineMat = new THREE.LineBasicMaterial({
    color: 0x06b6d4,
    linewidth: 3
  });

  trajectoryLine = new THREE.Line(lineGeo, lineMat);
  trajectoryLine.frustumCulled = false;
  scene.add(trajectoryLine);
}


// Add a 3D point to the dynamic trajectory trail
function addTrajectoryPoint(x, y, z) {
  trajectoryPoints.push(new THREE.Vector3(x, y, z));
  if (trajectoryPoints.length > MAX_TRAJECTORY_POINTS) {
    trajectoryPoints.shift();
  }

  const positions = trajectoryLine.geometry.attributes.position.array;
  for (let i = 0; i < trajectoryPoints.length; i++) {
    positions[i * 3] = trajectoryPoints[i].x;
    positions[i * 3 + 1] = trajectoryPoints[i].y;
    positions[i * 3 + 2] = trajectoryPoints[i].z;
  }

  trajectoryLine.geometry.attributes.position.needsUpdate = true;
  trajectoryLine.geometry.setDrawRange(0, trajectoryPoints.length);
}

// Connect to Telemetry EventSource (SSE)
function connectTelemetryStream() {
  const evtSource = new EventSource('/stream');

  evtSource.onopen = () => {
    elStatusBadge.className = 'status-badge connected';
    elStatusText.textContent = 'Streaming Live 50Hz';
  };

  evtSource.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      updateTelemetry(data);
    } catch (e) {
      console.error('JSON parse error:', e);
    }
  };

  evtSource.onerror = () => {
    elStatusBadge.className = 'status-badge disconnected';
    elStatusText.textContent = 'Waiting for main.py...';
  };
}

// ----------------------------------------------------
// Axis Remap & Swap State
// ----------------------------------------------------
let axisSwapConfig = {
  rollSource: 'roll',
  pitchSource: 'pitch',
  yawSource: 'yaw',
  invertRoll: false,
  invertPitch: false,
  invertYaw: false
};

function loadAxisSwapConfig() {
  try {
    const saved = localStorage.getItem('slimevr_axis_swap_config');
    if (saved) {
      axisSwapConfig = { ...axisSwapConfig, ...JSON.parse(saved) };
    }
  } catch (e) {
    console.warn('Could not load saved axis config', e);
  }
}

function saveAxisSwapConfig() {
  try {
    localStorage.setItem('slimevr_axis_swap_config', JSON.stringify(axisSwapConfig));
  } catch (e) {}
}

function getMappedAxisValue(source, invert, rawEulerObj) {
  const val = rawEulerObj[source] || 0;
  return invert ? -val : val;
}

// Update 3D Model and Telemetry Dashboard UI
function updateTelemetry(data) {
  if (!data || !data.translation || !data.rotation) return;

  const rawPos = data.translation.position || { x: 0, y: 0, z: 0 };
  rawLatestPos = rawPos;
  const rawEuler = data.rotation.euler || { roll: 90, pitch: 0, yaw: 0 };
  const rawQuat = data.rotation.quaternion || { w: 0.7071, x: 0.7071, y: 0, z: 0 };
  const vel = data.translation.velocity || { x: 0, y: 0, z: 0 };

  // Remap Roll, Pitch, and Yaw according to user settings
  const mappedRoll = getMappedAxisValue(axisSwapConfig.rollSource, axisSwapConfig.invertRoll, rawEuler);
  const mappedPitch = getMappedAxisValue(axisSwapConfig.pitchSource, axisSwapConfig.invertPitch, rawEuler);
  const mappedYaw = getMappedAxisValue(axisSwapConfig.yawSource, axisSwapConfig.invertYaw, rawEuler);

  // Calculate position adjusted for origin reset
  const posX = rawPos.x - positionOffset.x;
  const posY = rawPos.y - positionOffset.y;
  const posZ = rawPos.z - positionOffset.z;

  // Update target 3D transform
  currentTargetPos.set ? currentTargetPos.set(posX, posY, posZ) : (currentTargetPos = { x: posX, y: posY, z: posZ });

  // Apply custom axis mapping to 3D rotation quaternion if modified from default
  const isCustomMapping = axisSwapConfig.rollSource !== 'roll' ||
                          axisSwapConfig.pitchSource !== 'pitch' ||
                          axisSwapConfig.yawSource !== 'yaw' ||
                          axisSwapConfig.invertRoll ||
                          axisSwapConfig.invertPitch ||
                          axisSwapConfig.invertYaw;

  if (isCustomMapping) {
    const rollRad = THREE.MathUtils.degToRad(mappedRoll);
    const pitchRad = THREE.MathUtils.degToRad(mappedPitch);
    const yawRad = THREE.MathUtils.degToRad(mappedYaw);
    const mappedEulerObj = new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ');
    currentTargetQuat.setFromEuler(mappedEulerObj);
  } else {
    currentTargetQuat.set(rawQuat.x, rawQuat.y, rawQuat.z, rawQuat.w); // Three.js uses (x, y, z, w)
  }

  // Add point to trajectory trail
  addTrajectoryPoint(posX, posY, posZ);

  // Update Dashboard Text Metrics
  elPosX.textContent = posX.toFixed(2);
  elPosY.textContent = posY.toFixed(2);
  elPosZ.textContent = posZ.toFixed(2);

  elRoll.textContent = `${mappedRoll.toFixed(1)}°`;
  elPitch.textContent = `${mappedPitch.toFixed(1)}°`;
  elYaw.textContent = `${mappedYaw.toFixed(1)}°`;

  elQw.textContent = currentTargetQuat.w.toFixed(2);
  elQx.textContent = currentTargetQuat.x.toFixed(2);
  elQy.textContent = currentTargetQuat.y.toFixed(2);
  elQz.textContent = currentTargetQuat.z.toFixed(2);

  elHeading.textContent = `${(data.heading || 0).toFixed(1)}°`;

  const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
  elSpeed.textContent = `${speed.toFixed(2)} m/s`;

  // Update real-time sensor graphs
  pushChartData(eulerChart, mappedRoll, mappedPitch, mappedYaw);
  if (data.raw_imu) {
    updateSensorCharts(data.raw_imu);
  }
}

// ----------------------------------------------------
// Real-Time Sensor Graphs (Chart.js)
// ----------------------------------------------------
let accelChart, gyroChart, magChart, eulerChart;
const MAX_CHART_SAMPLES = 50;

function createSingleChart(canvasId, labelPrefix, customColors) {
  const canvasEl = document.getElementById(canvasId);
  if (!canvasEl) return null;
  const ctx = canvasEl.getContext('2d');

  const labels = Array.isArray(labelPrefix) ? labelPrefix : [`${labelPrefix} X`, `${labelPrefix} Y`, `${labelPrefix} Z`];
  const colors = customColors || ['#f43f5e', '#10b981', '#06b6d4'];

  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: labels[0], data: [], borderColor: colors[0], borderWidth: 1.5, pointRadius: 0, fill: false },
        { label: labels[1], data: [], borderColor: colors[1], borderWidth: 1.5, pointRadius: 0, fill: false },
        { label: labels[2], data: [], borderColor: colors[2], borderWidth: 1.5, pointRadius: 0, fill: false }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: { display: false },
        y: {
          ticks: { color: '#94a3b8', font: { size: 9 } },
          grid: { color: 'rgba(255,255,255,0.05)' }
        }
      },
      plugins: {
        legend: {
          labels: { color: '#f8fafc', font: { size: 10 }, boxWidth: 8 }
        }
      }
    }
  });
}

function initSensorCharts() {
  eulerChart = createSingleChart('eulerChart', ['Roll °', 'Pitch °', 'Yaw °'], ['#ec4899', '#10b981', '#06b6d4']);
  accelChart = createSingleChart('accelChart', 'Accel');
  gyroChart = createSingleChart('gyroChart', 'Gyro');
  magChart = createSingleChart('magChart', 'Mag');
}

function pushChartData(chart, xVal, yVal, zVal) {
  if (!chart) return;
  const labels = chart.data.labels;
  labels.push('');
  if (labels.length > MAX_CHART_SAMPLES) labels.shift();

  chart.data.datasets[0].data.push(xVal);
  if (chart.data.datasets[0].data.length > MAX_CHART_SAMPLES) chart.data.datasets[0].data.shift();

  chart.data.datasets[1].data.push(yVal);
  if (chart.data.datasets[1].data.length > MAX_CHART_SAMPLES) chart.data.datasets[1].data.shift();

  chart.data.datasets[2].data.push(zVal);
  if (chart.data.datasets[2].data.length > MAX_CHART_SAMPLES) chart.data.datasets[2].data.shift();

  chart.update('none');
}

function updateSensorCharts(raw_imu) {
  if (!raw_imu) return;
  if (raw_imu.accel) pushChartData(accelChart, raw_imu.accel.x, raw_imu.accel.y, raw_imu.accel.z);
  if (raw_imu.gyro) pushChartData(gyroChart, raw_imu.gyro.x, raw_imu.gyro.y, raw_imu.gyro.z);
  if (raw_imu.mag) pushChartData(magChart, raw_imu.mag.x, raw_imu.mag.y, raw_imu.mag.z);
}

// Animation Frame Loop
let followMode = false;

function animate() {
  requestAnimationFrame(animate);

  // Smoothly interpolate controller 3D transform (slerp / lerp)
  if (controllerGroup) {
    controllerGroup.position.lerp(new THREE.Vector3(currentTargetPos.x, currentTargetPos.y, currentTargetPos.z), 0.3);
    controllerGroup.quaternion.slerp(currentTargetQuat, 0.3);

    // Smoothly track controller position with camera target if follow mode is active
    if (followMode) {
      controls.target.lerp(controllerGroup.position, 0.1);
    }
  }

  controls.update();
  renderer.render(scene, camera);
}

// Window Resize Handler
function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

// Bind Button Listeners
const elGraphDrawer = document.getElementById('graphDrawer');
const elBtnToggleGraphs = document.getElementById('btnToggleGraphs');
const elBtnCloseDrawer = document.getElementById('btnCloseDrawer');

function toggleGraphDrawer() {
  if (!elGraphDrawer) return;
  elGraphDrawer.classList.toggle('hidden');
  const isVisible = !elGraphDrawer.classList.contains('hidden');
  if (elBtnToggleGraphs) elBtnToggleGraphs.classList.toggle('active', isVisible);
}

if (elBtnToggleGraphs) elBtnToggleGraphs.addEventListener('click', toggleGraphDrawer);
if (elBtnCloseDrawer) elBtnCloseDrawer.addEventListener('click', toggleGraphDrawer);

document.getElementById('btnCalibrateDrift').addEventListener('click', () => {
  positionOffset = { ...rawLatestPos };
  trajectoryPoints = [];
  trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnToggleFollow').addEventListener('click', (e) => {
  followMode = !followMode;
  e.currentTarget.classList.toggle('active', followMode);
});

document.getElementById('btnResetTrail').addEventListener('click', () => {
  trajectoryPoints = [];
  trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnResetPos').addEventListener('click', () => {
  positionOffset = { ...currentTargetPos };
  trajectoryPoints = [];
  trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnRecenterCam').addEventListener('click', () => {
  camera.position.set(1.5, 1.2, 2.0);
  controls.target.set(0, 0, 0);
});

let gridVisible = true;
document.getElementById('btnToggleGrid').addEventListener('click', (e) => {
  gridVisible = !gridVisible;
  gridHelper.visible = gridVisible;
  e.currentTarget.classList.toggle('active', gridVisible);
});

// ----------------------------------------------------
// Axis Swap UI Binding & Handlers
// ----------------------------------------------------
const elAxisSwapDrawer = document.getElementById('axisSwapDrawer');
const elBtnToggleAxisSwap = document.getElementById('btnToggleAxisSwap');
const elBtnQuickAxisSwap = document.getElementById('btnQuickAxisSwap');
const elBtnCloseAxisDrawer = document.getElementById('btnCloseAxisDrawer');

const elSelectRoll = document.getElementById('selectRollAxis');
const elSelectPitch = document.getElementById('selectPitchAxis');
const elSelectYaw = document.getElementById('selectYawAxis');

const elChkInvertRoll = document.getElementById('chkInvertRoll');
const elChkInvertPitch = document.getElementById('chkInvertPitch');
const elChkInvertYaw = document.getElementById('chkInvertYaw');

function toggleAxisSwapDrawer() {
  if (!elAxisSwapDrawer) return;
  elAxisSwapDrawer.classList.toggle('hidden');
  const isVisible = !elAxisSwapDrawer.classList.contains('hidden');
  if (elBtnToggleAxisSwap) elBtnToggleAxisSwap.classList.toggle('active', isVisible);
}

if (elBtnToggleAxisSwap) elBtnToggleAxisSwap.addEventListener('click', toggleAxisSwapDrawer);
if (elBtnQuickAxisSwap) elBtnQuickAxisSwap.addEventListener('click', toggleAxisSwapDrawer);
if (elBtnCloseAxisDrawer) elBtnCloseAxisDrawer.addEventListener('click', toggleAxisSwapDrawer);

function syncAxisSwapUI() {
  if (elSelectRoll) elSelectRoll.value = axisSwapConfig.rollSource;
  if (elSelectPitch) elSelectPitch.value = axisSwapConfig.pitchSource;
  if (elSelectYaw) elSelectYaw.value = axisSwapConfig.yawSource;

  if (elChkInvertRoll) elChkInvertRoll.checked = axisSwapConfig.invertRoll;
  if (elChkInvertPitch) elChkInvertPitch.checked = axisSwapConfig.invertPitch;
  if (elChkInvertYaw) elChkInvertYaw.checked = axisSwapConfig.invertYaw;
}

function updateAxisSwapFromUI() {
  if (elSelectRoll) axisSwapConfig.rollSource = elSelectRoll.value;
  if (elSelectPitch) axisSwapConfig.pitchSource = elSelectPitch.value;
  if (elSelectYaw) axisSwapConfig.yawSource = elSelectYaw.value;

  if (elChkInvertRoll) axisSwapConfig.invertRoll = elChkInvertRoll.checked;
  if (elChkInvertPitch) axisSwapConfig.invertPitch = elChkInvertPitch.checked;
  if (elChkInvertYaw) axisSwapConfig.invertYaw = elChkInvertYaw.checked;

  saveAxisSwapConfig();
}

[elSelectRoll, elSelectPitch, elSelectYaw, elChkInvertRoll, elChkInvertPitch, elChkInvertYaw].forEach(el => {
  if (el) el.addEventListener('change', updateAxisSwapFromUI);
});

// Quick Presets
document.getElementById('presetDefault')?.addEventListener('click', () => {
  axisSwapConfig = { rollSource: 'roll', pitchSource: 'pitch', yawSource: 'yaw', invertRoll: false, invertPitch: false, invertYaw: false };
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

document.getElementById('presetSwapRP')?.addEventListener('click', () => {
  axisSwapConfig = { ...axisSwapConfig, rollSource: 'pitch', pitchSource: 'roll' };
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

document.getElementById('presetSwapPY')?.addEventListener('click', () => {
  axisSwapConfig = { ...axisSwapConfig, pitchSource: 'yaw', yawSource: 'pitch' };
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

document.getElementById('presetSwapRY')?.addEventListener('click', () => {
  axisSwapConfig = { ...axisSwapConfig, rollSource: 'yaw', yawSource: 'roll' };
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

// Launch on page load
window.addEventListener('DOMContentLoaded', () => {
  loadAxisSwapConfig();
  syncAxisSwapUI();
  initScene();
  initSensorCharts();
  connectTelemetryStream();
});


