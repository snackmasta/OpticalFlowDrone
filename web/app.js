// ----------------------------------------------------
// 6DoF VR Controller 3D Trajectory Visualizer - Three.js
// ----------------------------------------------------

let scene, camera, renderer, controls;
let controllerGroup, trajectoryLine, gridHelper;
let cubeMat;
let trajectoryPoints = [];
const MAX_TRAJECTORY_POINTS = 1000;
let isDarkMode = true;

// Anchor translation state (lock position movement)
let isAnchorActive = false;
let anchoredPos = { x: 0, y: 0, z: 0 };

// Origin offset for resetting zero-position
let positionOffset = { x: 0, y: 0, z: 0 };
let rawLatestPos = { x: 0, y: 0, z: 0 };
let currentTargetPos = new THREE.Vector3(0, 0, 0);
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
  scene.background = new THREE.Color(isDarkMode ? 0x090d16 : 0xf1f5f9);

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
  gridHelper = new THREE.GridHelper(10, 20, isDarkMode ? 0x06b6d4 : 0x0284c7, isDarkMode ? 0x334155 : 0xcbd5e1);
  gridHelper.position.y = -0.5;
  scene.add(gridHelper);

  // 1x1m Active Geofence 3D Box
  createGeofence3D();

  // Build Drone 3D Mesh
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

  // Simple Cube Mesh (replaces VR Controller Model)
  const cubeGeo = new THREE.BoxGeometry(0.12, 0.12, 0.12);
  cubeMat = new THREE.MeshStandardMaterial({
    color: isDarkMode ? 0x06b6d4 : 0x0284c7,
    roughness: 0.3,
    metalness: 0.5
  });
  const cubeMesh = new THREE.Mesh(cubeGeo, cubeMat);
  cubeMesh.castShadow = true;
  cubeMesh.receiveShadow = true;
  controllerGroup.add(cubeMesh);

  // Add subtle edge outlines to highlight cube orientation
  const edgesGeo = new THREE.EdgesGeometry(cubeGeo);
  const edgesMat = new THREE.LineBasicMaterial({
    color: isDarkMode ? 0x38bdf8 : 0x0369a1,
    linewidth: 1.5
  });
  const edgesMesh = new THREE.LineSegments(edgesGeo, edgesMat);
  controllerGroup.add(edgesMesh);

  scene.add(controllerGroup);
}

let geofenceMesh = null;
let geofenceBoxWireframe = null;
let geofenceCenter = { x: 0, y: 0, z: 0 };

let breachCountdownTimer = null;
let breachCountdownSec = 30;
let isBreachCountdownActive = false;

function createGeofence3D() {
  // 1.5x1.5m Active Geofence 3D Box Geometry (1.5m x 1.5m x 1.5m)
  const boxGeo = new THREE.BoxGeometry(1.5, 1.5, 1.5);

  // Translucent Green Fill Material
  const fillMat = new THREE.MeshBasicMaterial({
    color: 0x10b981,
    transparent: true,
    opacity: 0.12,
    side: THREE.DoubleSide
  });
  geofenceMesh = new THREE.Mesh(boxGeo, fillMat);
  geofenceMesh.position.set(geofenceCenter.x, geofenceCenter.y, geofenceCenter.z);

  // Glowing Outline Wireframe
  const wireGeo = new THREE.EdgesGeometry(boxGeo);
  const wireMat = new THREE.LineBasicMaterial({ color: 0x10b981, linewidth: 2 });
  geofenceBoxWireframe = new THREE.LineSegments(wireGeo, wireMat);
  geofenceMesh.add(geofenceBoxWireframe);

  scene.add(geofenceMesh);
}

function recenterGeofenceToCube() {
  if (!controllerGroup || !geofenceMesh) return;

  // Relocate geofence center to current 3D cube position
  geofenceCenter = {
    x: controllerGroup.position.x,
    y: controllerGroup.position.y,
    z: controllerGroup.position.z
  };

  geofenceMesh.position.set(geofenceCenter.x, geofenceCenter.y, geofenceCenter.z);

  // Reset countdown
  isBreachCountdownActive = false;
  breachCountdownSec = 30;
  if (breachCountdownTimer) {
    clearInterval(breachCountdownTimer);
    breachCountdownTimer = null;
  }

  // Re-evaluate breach check to clear breach status immediately
  updateGeofenceHitboxCheck();
}

function resetGeofenceCenter() {
  geofenceCenter = { x: 0, y: 0, z: 0 };
  if (geofenceMesh) {
    geofenceMesh.position.set(0, 0, 0);
  }
  isBreachCountdownActive = false;
  breachCountdownSec = 30;
  if (breachCountdownTimer) {
    clearInterval(breachCountdownTimer);
    breachCountdownTimer = null;
  }
  updateGeofenceHitboxCheck();
}

// ----------------------------------------------------
// 3D Hitbox-Based Geofence Breach & Auto-Recenter Detection
// ----------------------------------------------------
function updateGeofenceHitboxCheck() {
  if (!geofenceMesh || !controllerGroup) return;

  // Cube size: 0.12m x 0.12m x 0.12m (half-size = 0.06m)
  // Fence size: 1.5m x 1.5m x 1.5m (half-size = 0.75m)
  const CUBE_HALF_WIDTH_M = 0.06;
  const FENCE_HALF_WIDTH_M = 0.75;

  // Displacement relative to current geofence center
  const dx = controllerGroup.position.x - geofenceCenter.x;
  const dy = controllerGroup.position.y - geofenceCenter.y;
  const dz = controllerGroup.position.z - geofenceCenter.z;

  const isBreached = (Math.abs(dx) + CUBE_HALF_WIDTH_M > FENCE_HALF_WIDTH_M) ||
                     (Math.abs(dz) + CUBE_HALF_WIDTH_M > FENCE_HALF_WIDTH_M) ||
                     (Math.abs(dy) + CUBE_HALF_WIDTH_M > FENCE_HALF_WIDTH_M);

  // Manage 30-second relocation countdown
  if (isBreached) {
    if (!isBreachCountdownActive) {
      isBreachCountdownActive = true;
      breachCountdownSec = 30;

      if (breachCountdownTimer) clearInterval(breachCountdownTimer);
      breachCountdownTimer = setInterval(() => {
        breachCountdownSec -= 1;
        if (breachCountdownSec <= 0) {
          recenterGeofenceToCube();
        } else {
          const elBadge = document.getElementById('geofenceBadge');
          if (elBadge) {
            elBadge.className = 'geofence-badge breach';
            elBadge.innerHTML = `<i class="fa-solid fa-triangle-exclamation me-1"></i> BREACH! RECENTERING IN ${breachCountdownSec}s`;
          }
        }
      }, 1000);
    }
  } else {
    if (isBreachCountdownActive) {
      isBreachCountdownActive = false;
      breachCountdownSec = 30;
      if (breachCountdownTimer) {
        clearInterval(breachCountdownTimer);
        breachCountdownTimer = null;
      }
    }
  }

  const elGeofenceBadge = document.getElementById('geofenceBadge');
  if (elGeofenceBadge) {
    if (isBreached) {
      elGeofenceBadge.className = 'geofence-badge breach';
      elGeofenceBadge.innerHTML = `<i class="fa-solid fa-triangle-exclamation me-1"></i> BREACH! RECENTERING IN ${breachCountdownSec}s`;
    } else {
      elGeofenceBadge.className = 'geofence-badge safe';
      elGeofenceBadge.innerHTML = '<i class="fa-solid fa-shield-halved me-1"></i> GEOFENCE: INSIDE (1.5x1.5m)';
    }
  }

  if (geofenceBoxWireframe && geofenceMesh) {
    if (isBreached) {
      geofenceBoxWireframe.material.color.setHex(0xef4444);
      geofenceMesh.material.color.setHex(0xef4444);
    } else {
      geofenceBoxWireframe.material.color.setHex(0x10b981);
      geofenceMesh.material.color.setHex(0x10b981);
    }
  }

  // Continuously refresh breach status (and heartbeat every 500ms while breached) to keep geofence buzzer listener active
  const now = Date.now();
  if (window.lastGeofenceBreachState !== isBreached || (isBreached && (now - (window.lastGeofenceBreachTime || 0) > 500))) {
    window.lastGeofenceBreachState = isBreached;
    window.lastGeofenceBreachTime = now;
    try {
      fetch('/api/geofence/status', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ breached: isBreached })
      }).catch(() => {});
    } catch (e) {}
  }
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
    if (!isReplayMode) {
      elStatusBadge.className = 'status-badge connected';
      elStatusText.textContent = 'Streaming Live 50Hz';
    }
  };

  evtSource.onmessage = (event) => {
    try {
      if (isReplayMode) return; // Suppress live updates while in Replay Mode
      const data = JSON.parse(event.data);
      updateTelemetry(data);
    } catch (e) {
      console.error('JSON parse error:', e);
    }
  };

  evtSource.onerror = () => {
    if (!isReplayMode) {
      elStatusBadge.className = 'status-badge disconnected';
      elStatusText.textContent = 'Waiting for main.py...';
    }
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

  // Calculate position adjusted for origin reset (inverting Y and Z for intuitive 3D camera coordinate space)
  let posX, posY, posZ;
  if (isAnchorActive) {
    posX = anchoredPos.x;
    posY = anchoredPos.y;
    posZ = anchoredPos.z;
  } else {
    posX = rawPos.x - positionOffset.x;
    posY = -(rawPos.y - positionOffset.y);
    posZ = -(rawPos.z - positionOffset.z);
  }

  // Exponential moving average filter for buttery smooth position rendering (alpha = 0.2)
  if (typeof targetPosSmooth === 'undefined') {
    window.targetPosSmooth = new THREE.Vector3(posX, posY, posZ);
  } else {
    targetPosSmooth.x += (posX - targetPosSmooth.x) * 0.2;
    targetPosSmooth.y += (posY - targetPosSmooth.y) * 0.2;
    targetPosSmooth.z += (posZ - targetPosSmooth.z) * 0.2;
  }

  // Update target 3D transform
  currentTargetPos.copy(targetPosSmooth);

  // Construct 3D orientation quaternion directly from remapped Roll, Pitch, and Yaw angles
  const rollRad = THREE.MathUtils.degToRad(mappedRoll);
  const pitchRad = THREE.MathUtils.degToRad(mappedPitch);
  const yawRad = THREE.MathUtils.degToRad(mappedYaw);
  const mappedEulerObj = new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ');
  currentTargetQuat.setFromEuler(mappedEulerObj);

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

  // Update GPS, 2D Planar Projection, and Geofence Breach Status
  updateGpsTelemetry(data.gps, data.fused_gps, posX, posZ);
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
  if (!chart || !chart.data) return;
  const labels = chart.data.labels;
  const datasets = chart.data.datasets;
  if (!datasets || datasets.length < 3) return;

  labels.push('');
  if (labels.length > MAX_CHART_SAMPLES) labels.shift();

  datasets[0].data.push(xVal != null ? xVal : 0);
  if (datasets[0].data.length > MAX_CHART_SAMPLES) datasets[0].data.shift();

  datasets[1].data.push(yVal != null ? yVal : 0);
  if (datasets[1].data.length > MAX_CHART_SAMPLES) datasets[1].data.shift();

  datasets[2].data.push(zVal != null ? zVal : 0);
  if (datasets[2].data.length > MAX_CHART_SAMPLES) datasets[2].data.shift();

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

  // Smoothly interpolate controller 3D position and orientation
  if (controllerGroup) {
    controllerGroup.position.lerp(currentTargetPos, 0.3);
    controllerGroup.quaternion.slerp(currentTargetQuat, 0.3);

    // Hitbox Bounding-Box Geofence Breach Check
    updateGeofenceHitboxCheck();

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
  resetGeofenceCenter();
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
  resetGeofenceCenter();
  trajectoryPoints = [];
  trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnRecenterCam').addEventListener('click', () => {
  camera.position.set(1.5, 1.2, 2.0);
  controls.target.set(0, 0, 0);
});

// Anchor Translation Toggle Button Handler
const elBtnToggleAnchor = document.getElementById('btnToggleAnchor');
if (elBtnToggleAnchor) {
  elBtnToggleAnchor.addEventListener('click', (e) => {
    isAnchorActive = !isAnchorActive;
    if (isAnchorActive) {
      if (typeof targetPosSmooth !== 'undefined') {
        anchoredPos = { x: targetPosSmooth.x, y: targetPosSmooth.y, z: targetPosSmooth.z };
      } else {
        anchoredPos = { x: 0, y: 0, z: 0 };
      }
    }
    e.currentTarget.classList.toggle('active', isAnchorActive);
  });
}

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
  const oldRoll = axisSwapConfig.rollSource;
  axisSwapConfig.rollSource = axisSwapConfig.pitchSource;
  axisSwapConfig.pitchSource = oldRoll;
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

document.getElementById('presetSwapPY')?.addEventListener('click', () => {
  const oldPitch = axisSwapConfig.pitchSource;
  axisSwapConfig.pitchSource = axisSwapConfig.yawSource;
  axisSwapConfig.yawSource = oldPitch;
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

document.getElementById('presetSwapRY')?.addEventListener('click', () => {
  const oldRoll = axisSwapConfig.rollSource;
  axisSwapConfig.rollSource = axisSwapConfig.yawSource;
  axisSwapConfig.yawSource = oldRoll;
  syncAxisSwapUI();
  saveAxisSwapConfig();
});

// ----------------------------------------------------
// Dark / Light Theme Logic & Inverted Controller Mesh
// ----------------------------------------------------
function loadThemeConfig() {
  try {
    const saved = localStorage.getItem('slimevr_theme');
    if (saved === 'light') {
      isDarkMode = false;
    } else {
      isDarkMode = true;
    }
  } catch (e) {
    isDarkMode = true;
  }
}

function toggleTheme() {
  isDarkMode = !isDarkMode;
  try {
    localStorage.setItem('slimevr_theme', isDarkMode ? 'dark' : 'light');
  } catch (e) {}
  applyTheme(isDarkMode);
}

function applyTheme(isDark) {
  document.body.classList.toggle('light-theme', !isDark);

  const elThemeIcon = document.getElementById('themeIcon');
  if (elThemeIcon) {
    elThemeIcon.className = isDark ? 'fa-solid fa-sun' : 'fa-solid fa-moon';
  }

  const elThemeToolBtn = document.getElementById('btnToggleThemeTool');
  if (elThemeToolBtn) {
    elThemeToolBtn.classList.toggle('active', !isDark);
  }

  // Update WebGL Scene Background
  if (scene) {
    scene.background = new THREE.Color(isDark ? 0x090d16 : 0xf1f5f9);
  }

  // Update 3D Cube Material Theme
  if (cubeMat) {
    cubeMat.color.setHex(isDark ? 0x06b6d4 : 0x0284c7);
    cubeMat.needsUpdate = true;
  }

  // Update Trajectory Line
  if (trajectoryLine && trajectoryLine.material) {
    trajectoryLine.material.color.setHex(isDark ? 0x06b6d4 : 0x0284c7);
  }

  // Update 3D Grid Helper
  if (gridHelper && scene) {
    scene.remove(gridHelper);
    if (gridHelper.geometry) gridHelper.geometry.dispose();
    const gridCenterColor = isDark ? 0x06b6d4 : 0x0284c7;
    const gridLineColor = isDark ? 0x334155 : 0xcbd5e1;
    gridHelper = new THREE.GridHelper(10, 20, gridCenterColor, gridLineColor);
    gridHelper.position.y = -0.5;
    gridHelper.visible = typeof gridVisible !== 'undefined' ? gridVisible : true;
    scene.add(gridHelper);
  }

  // Update Real-Time Sensor Graphs
  updateChartThemes(isDark);
}

function updateChartThemes(isDark) {
  const tickColor = isDark ? '#94a3b8' : '#475569';
  const gridColor = isDark ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.08)';
  const legendColor = isDark ? '#f8fafc' : '#0f172a';

  [eulerChart, accelChart, gyroChart, magChart].forEach(chart => {
    if (!chart) return;
    if (chart.options && chart.options.scales && chart.options.scales.y) {
      chart.options.scales.y.ticks.color = tickColor;
      chart.options.scales.y.grid.color = gridColor;
    }
    if (chart.options && chart.options.plugins && chart.options.plugins.legend) {
      chart.options.plugins.legend.labels.color = legendColor;
    }
    chart.update('none');
  });
}

// Bind Theme Toggle Buttons
document.getElementById('btnToggleTheme')?.addEventListener('click', toggleTheme);
document.getElementById('btnToggleThemeTool')?.addEventListener('click', toggleTheme);

// ----------------------------------------------------
// Recorded Session Logs Reader Handler
// ----------------------------------------------------
const logsDrawer = document.getElementById('logsDrawer');
const btnToggleLogs = document.getElementById('btnToggleLogs');
const btnCloseLogsDrawer = document.getElementById('btnCloseLogsDrawer');
const logSelect = document.getElementById('logSelect');
const btnRefreshLogs = document.getElementById('btnRefreshLogs');
const btnLoadSelectedLog = document.getElementById('btnLoadSelectedLog');
const logSummaryPanel = document.getElementById('logSummaryPanel');
const logMetricsGrid = document.getElementById('logMetricsGrid');
const logTableHeader = document.getElementById('logTableHeader');
const logTableBody = document.getElementById('logTableBody');

function toggleLogsDrawer() {
  if (!logsDrawer) return;
  const isHidden = logsDrawer.classList.contains('hidden');
  if (isHidden) {
    logsDrawer.classList.remove('hidden');
    btnToggleLogs?.classList.add('active');
    fetchAvailableSessionLogs();
  } else {
    logsDrawer.classList.add('hidden');
    btnToggleLogs?.classList.remove('active');
  }
}

async function fetchAvailableSessionLogs() {
  if (!logSelect) return;
  try {
    logSelect.innerHTML = '<option value="">Loading logs list...</option>';
    const resp = await fetch('/api/logs');
    const data = await resp.json();
    if (data.status === 'success' && Array.isArray(data.logs)) {
      if (data.logs.length === 0) {
        logSelect.innerHTML = '<option value="">No logs found</option>';
        return;
      }
      logSelect.innerHTML = '<option value="">-- Select a Session Log CSV (' + data.logs.length + ' found) --</option>';
      data.logs.forEach(log => {
        const opt = document.createElement('option');
        opt.value = log.relative_path || log.filename;
        opt.textContent = `${log.filename} (${log.sample_count} samples, ${log.modified_time})`;
        logSelect.appendChild(opt);
      });
      // Auto select and load the first (newest) log
      logSelect.selectedIndex = 1;
      loadSelectedSessionLog();
    } else {
      logSelect.innerHTML = '<option value="">No logs found</option>';
    }
  } catch (err) {
    console.error('Failed to fetch session logs:', err);
    logSelect.innerHTML = '<option value="">Error loading logs list</option>';
  }
}

async function loadSelectedSessionLog() {
  const selectedPath = logSelect?.value;
  if (!selectedPath) {
    return;
  }

  try {
    if (btnLoadSelectedLog) {
      btnLoadSelectedLog.innerHTML = '<i class="fa-solid fa-spinner fa-spin me-1"></i> Loading...';
    }
    const resp = await fetch(`/api/log?file=${encodeURIComponent(selectedPath)}`);
    const result = await resp.json();
    if (btnLoadSelectedLog) {
      btnLoadSelectedLog.innerHTML = '<i class="fa-solid fa-folder-open me-1"></i> Load Log Data';
    }

    if (result.status === 'success' && result.log) {
      loadedLogData = result.log;
      renderSessionLogData(result.log);
    } else {
      alert('Error loading log data: ' + (result.message || 'Unknown error'));
    }
  } catch (err) {
    if (btnLoadSelectedLog) {
      btnLoadSelectedLog.innerHTML = '<i class="fa-solid fa-folder-open me-1"></i> Load Log Data';
    }
    console.error('Error loading selected log:', err);
    alert('Failed to load session log data.');
  }
}

function renderSessionLogData(logData) {
  // 1. Render Summary Metrics
  if (logSummaryPanel && logMetricsGrid) {
    logSummaryPanel.style.display = 'block';
    logMetricsGrid.innerHTML = '';
    const summary = logData.summary || {};
    const metrics = [
      { label: 'Filename', val: logData.filename },
      { label: 'Samples', val: logData.total_samples },
      { label: 'Duration', val: summary.duration_s != null ? `${summary.duration_s} s` : '-' },
      { label: 'Max Altitude', val: summary.max_altitude_m != null ? `${summary.max_altitude_m} m` : '-' },
      { label: 'Max Speed', val: summary.max_speed_mps != null ? `${summary.max_speed_mps} m/s` : '-' },
      { label: 'Avg Speed', val: summary.avg_speed_mps != null ? `${summary.avg_speed_mps} m/s` : '-' },
      { label: 'Modified', val: logData.modified_time }
    ];

    metrics.forEach(m => {
      const item = document.createElement('div');
      item.style.background = 'rgba(15, 23, 42, 0.6)';
      item.style.padding = '6px 10px';
      item.style.borderRadius = '6px';
      item.style.border = '1px solid rgba(255,255,255,0.05)';
      item.innerHTML = `<div style="color: #94a3b8; font-size: 11px;">${m.label}</div><div style="color: #38bdf8; font-weight: 600; overflow: hidden; text-overflow: ellipsis;">${m.val}</div>`;
      logMetricsGrid.appendChild(item);
    });
  }

  // 2. Render Header
  if (logTableHeader && Array.isArray(logData.headers)) {
    logTableHeader.innerHTML = '';
    logData.headers.forEach(h => {
      const th = document.createElement('th');
      th.style.padding = '6px 10px';
      th.style.whiteSpace = 'nowrap';
      th.style.borderBottom = '1px solid rgba(255,255,255,0.1)';
      th.textContent = h;
      logTableHeader.appendChild(th);
    });
  }

  // 3. Render Sample Rows (Limit to first 500 rows for display performance)
  if (logTableBody && Array.isArray(logData.records)) {
    logTableBody.innerHTML = '';
    const recordsToDisplay = logData.records.slice(0, 500);
    recordsToDisplay.forEach((rec, idx) => {
      const tr = document.createElement('tr');
      tr.style.background = idx % 2 === 0 ? 'transparent' : 'rgba(255,255,255,0.02)';
      logData.headers.forEach(h => {
        const td = document.createElement('td');
        td.style.padding = '4px 10px';
        td.style.whiteSpace = 'nowrap';
        td.style.borderBottom = '1px solid rgba(255,255,255,0.05)';
        td.textContent = rec[h] !== undefined ? rec[h] : '';
        tr.appendChild(td);
      });
      logTableBody.appendChild(tr);
    });
  }
}

// Bind Log Drawer UI Events
btnToggleLogs?.addEventListener('click', toggleLogsDrawer);
btnCloseLogsDrawer?.addEventListener('click', () => {
  logsDrawer?.classList.add('hidden');
  btnToggleLogs?.classList.remove('active');
});
btnRefreshLogs?.addEventListener('click', fetchAvailableSessionLogs);
btnLoadSelectedLog?.addEventListener('click', loadSelectedSessionLog);
logSelect?.addEventListener('change', loadSelectedSessionLog);

// ----------------------------------------------------
// 3D Trajectory Replay Engine
// ----------------------------------------------------
let isReplayMode = false;
let replayRecords = [];
let replayIndex = 0;
let isReplayPlaying = false;
let replaySpeed = 1.0;
let isReplayLoop = false;
let replayAnimFrameId = null;
let lastReplayTime = 0;

const btnStartReplay = document.getElementById('btnStartReplay');
const replayBar = document.getElementById('replayBar');
const elReplayFilename = document.getElementById('replayFilename');
const btnReplayPlayPause = document.getElementById('btnReplayPlayPause');
const iconReplayPlayPause = document.getElementById('iconReplayPlayPause');
const btnReplayStop = document.getElementById('btnReplayStop');
const replayTimeCurrent = document.getElementById('replayTimeCurrent');
const replayTimeTotal = document.getElementById('replayTimeTotal');
const replaySlider = document.getElementById('replaySlider');
const replaySpeedSelect = document.getElementById('replaySpeedSelect');
const btnReplayLoop = document.getElementById('btnReplayLoop');
const btnReplayExit = document.getElementById('btnReplayExit');

// Helper to convert CSV record into standard Telemetry object
function recordToTelemetry(rec) {
  if (!rec) return null;
  const x_m = (rec["X Position (cm)"] !== undefined) ? rec["X Position (cm)"] / 100.0 : (rec["x"] || 0);
  const y_m = (rec["Y Position (cm)"] !== undefined) ? rec["Y Position (cm)"] / 100.0 : (rec["z"] || 0);
  const alt_m = (rec["Altitude (m)"] !== undefined) ? rec["Altitude (m)"] : (rec["y"] || 0);

  const vx = rec["VX (m/s)"] || 0;
  const vy = rec["VY (m/s)"] || 0;
  const vz = rec["VZ (m/s)"] || 0;

  const roll = rec["Roll (deg)"] || 0;
  const pitch = rec["Pitch (deg)"] || 0;
  const yaw = rec["Yaw (deg)"] || 0;
  const heading = rec["Heading (deg)"] || 0;

  const accelX = rec["Accel X (g)"] || 0;
  const accelY = rec["Accel Y (g)"] || 0;
  const gyroZ = rec["Gyro Z (dps)"] || 0;

  return {
    timestamp: rec["Timestamp (s)"] || Date.now() / 1000,
    translation: {
      position: { x: x_m, y: alt_m, z: y_m },
      velocity: { x: vx, y: vz, z: vy }
    },
    rotation: {
      euler: { roll: roll, pitch: pitch, yaw: yaw }
    },
    heading: heading,
    raw_imu: {
      accel: { x: accelX, y: accelY, z: 1.0 },
      gyro: { x: 0, y: 0, z: gyroZ },
      mag: { x: 0, y: 0, z: 0 }
    }
  };
}

function formatTimeSec(seconds) {
  const s = Math.max(0, Math.floor(seconds || 0));
  const m = Math.floor(s / 60);
  const remS = s % 60;
  return `${m.toString().padStart(2, '0')}:${remS.toString().padStart(2, '0')}`;
}

function start3DReplay() {
  if (!loadedLogData || !Array.isArray(loadedLogData.records) || loadedLogData.records.length === 0) {
    alert('Please select and load a session log CSV first before replaying.');
    return;
  }

  isReplayMode = true;
  replayRecords = loadedLogData.records;
  replayIndex = 0;
  isReplayPlaying = true;
  lastReplayTime = performance.now();

  // Reset 3D trajectory path for replay
  trajectoryPoints = [];
  if (trajectoryLine && trajectoryLine.geometry) {
    trajectoryLine.geometry.setDrawRange(0, 0);
  }

  // Update UI Elements
  if (elReplayFilename) elReplayFilename.textContent = loadedLogData.filename || 'session.csv';
  if (replayBar) replayBar.classList.remove('hidden');
  if (logsDrawer) logsDrawer.classList.add('hidden');
  if (btnToggleLogs) btnToggleLogs.classList.remove('active');

  // Update status badge
  if (elStatusBadge && elStatusText) {
    elStatusBadge.className = 'status-badge connected';
    elStatusBadge.style.background = 'rgba(236, 72, 153, 0.2)';
    elStatusBadge.style.borderColor = '#ec4899';
    elStatusText.textContent = `Replaying: ${loadedLogData.filename}`;
  }

  // Set total duration display
  const summary = loadedLogData.summary || {};
  const totalDuration = summary.duration_s || (replayRecords.length / 50.0);
  if (replayTimeTotal) replayTimeTotal.textContent = formatTimeSec(totalDuration);

  updateReplayPlayPauseIcon();
  requestAnimationFrame(replayLoopStep);
}

function replayLoopStep(timestamp) {
  if (!isReplayMode) return;

  if (isReplayPlaying && replayRecords.length > 0) {
    const elapsedMs = timestamp - lastReplayTime;

    // Determine target step interval (assume ~20ms per frame at 1x speed = 50Hz)
    let t0 = replayRecords[Math.max(0, replayIndex - 1)]?.["Timestamp (s)"];
    let t1 = replayRecords[replayIndex]?.["Timestamp (s)"];
    let dt_sec = 0.02;
    if (typeof t0 === 'number' && typeof t1 === 'number' && t1 > t0) {
      dt_sec = t1 - t0;
    }

    const stepThresholdMs = (dt_sec * 1000.0) / replaySpeed;

    if (elapsedMs >= stepThresholdMs) {
      lastReplayTime = timestamp;

      // Process current frame telemetry
      const rec = replayRecords[replayIndex];
      const telemData = recordToTelemetry(rec);
      if (telemData) {
        updateTelemetry(telemData);
      }

      // Update progress slider & time text
      const progress = (replayIndex / (replayRecords.length - 1)) * 100;
      if (replaySlider) replaySlider.value = progress;

      const currentRecordTime = rec["Timestamp (s)"];
      const startRecordTime = replayRecords[0]["Timestamp (s)"];
      let relSec = replayIndex * dt_sec;
      if (typeof currentRecordTime === 'number' && typeof startRecordTime === 'number') {
        relSec = Math.max(0, currentRecordTime - startRecordTime);
      }
      if (replayTimeCurrent) replayTimeCurrent.textContent = formatTimeSec(relSec);

      replayIndex++;

      if (replayIndex >= replayRecords.length) {
        if (isReplayLoop) {
          replayIndex = 0;
          trajectoryPoints = [];
        } else {
          isReplayPlaying = false;
          updateReplayPlayPauseIcon();
        }
      }
    }
  }

  if (isReplayMode) {
    replayAnimFrameId = requestAnimationFrame(replayLoopStep);
  }
}

function updateReplayPlayPauseIcon() {
  if (!iconReplayPlayPause) return;
  if (isReplayPlaying) {
    iconReplayPlayPause.className = 'fa-solid fa-pause';
    btnReplayPlayPause?.classList.add('active');
  } else {
    iconReplayPlayPause.className = 'fa-solid fa-play';
    btnReplayPlayPause?.classList.remove('active');
  }
}

function toggleReplayPlayPause() {
  isReplayPlaying = !isReplayPlaying;
  lastReplayTime = performance.now();
  updateReplayPlayPauseIcon();
}

function stopReplay() {
  isReplayPlaying = false;
  replayIndex = 0;
  trajectoryPoints = [];
  if (trajectoryLine && trajectoryLine.geometry) {
    trajectoryLine.geometry.setDrawRange(0, 0);
  }
  if (replaySlider) replaySlider.value = 0;
  if (replayTimeCurrent) replayTimeCurrent.textContent = '00:00';
  updateReplayPlayPauseIcon();

  if (replayRecords.length > 0) {
    updateTelemetry(recordToTelemetry(replayRecords[0]));
  }
}

function exitReplayMode() {
  isReplayMode = false;
  isReplayPlaying = false;
  if (replayAnimFrameId) {
    cancelAnimationFrame(replayAnimFrameId);
  }
  if (replayBar) replayBar.classList.add('hidden');

  // Restore live stream status badge
  if (elStatusBadge && elStatusText) {
    elStatusBadge.className = 'status-badge disconnected';
    elStatusBadge.style.background = '';
    elStatusBadge.style.borderColor = '';
    elStatusText.textContent = 'Waiting for main.py...';
  }

  // Clear trajectory trail
  trajectoryPoints = [];
  if (trajectoryLine && trajectoryLine.geometry) {
    trajectoryLine.geometry.setDrawRange(0, 0);
  }
}

// Bind Replay Bar Controls
btnStartReplay?.addEventListener('click', start3DReplay);
btnReplayPlayPause?.addEventListener('click', toggleReplayPlayPause);
btnReplayStop?.addEventListener('click', stopReplay);
btnReplayExit?.addEventListener('click', exitReplayMode);

btnReplayLoop?.addEventListener('click', () => {
  isReplayLoop = !isReplayLoop;
  btnReplayLoop.classList.toggle('active', isReplayLoop);
});

replaySpeedSelect?.addEventListener('change', (e) => {
  replaySpeed = parseFloat(e.target.value) || 1.0;
});

replaySlider?.addEventListener('input', (e) => {
  if (!replayRecords || replayRecords.length === 0) return;
  const val = parseFloat(e.target.value);
  replayIndex = Math.min(
    replayRecords.length - 1,
    Math.max(0, Math.floor((val / 100.0) * (replayRecords.length - 1)))
  );

  const rec = replayRecords[replayIndex];
  if (rec) {
    updateTelemetry(recordToTelemetry(rec));
  }
});

let eventSource = null;

function connectTelemetryStream() {
  if (eventSource) {
    try {
      eventSource.close();
    } catch (e) {}
  }

  eventSource = new EventSource('/stream');

  eventSource.onmessage = (event) => {
    if (isReplayMode) return;
    try {
      const data = JSON.parse(event.data);
      updateTelemetry(data);
    } catch (err) {
      console.error('Telemetry JSON parse error:', err);
    }
  };

  eventSource.onerror = (err) => {
    if (elStatusBadge && elStatusText) {
      elStatusBadge.className = 'status-badge disconnected';
      elStatusText.textContent = 'Disconnected (Reconnecting...)';
    }
  };

  eventSource.onopen = () => {
    if (elStatusBadge && elStatusText) {
      elStatusBadge.className = 'status-badge active';
      elStatusText.textContent = 'Connected (50Hz Stream)';
    }
  };
}

// Launch on page load
window.addEventListener('DOMContentLoaded', () => {
  loadThemeConfig();
  loadAxisSwapConfig();
  syncAxisSwapUI();
  initScene();
  initSensorCharts();
  applyTheme(isDarkMode);
  connectTelemetryStream();
});

// ----------------------------------------------------
// Real-World GPS & 2D Tangent Plane Projection Map
// ----------------------------------------------------
// ----------------------------------------------------
// Real-World GPS & 2D Tangent Plane Projection Dual Maps
// ----------------------------------------------------
let leafletMapRaw = null;
let leafletMapFused = null;
let markerRaw = null;
let markerFused = null;
let originMarkerRaw = null;
let originMarkerFused = null;
let polylineRaw = null;
let polylineFused = null;
let accuracyCircleFused = null;

let rawTrailPoints = [];
let fusedTrailPoints = [];
let isSyncingMaps = false;

let currentGpsState = {
  lat: null,
  lon: null,
  alt_m: 0.0,
  speed_kmh: 0.0,
  satellites: 0,
  fix_status: 'SEARCHING FOR SATELLITES...',
  projected_x_m: 0.0,
  projected_y_m: 0.0,
  origin: { lat: null, lon: null }
};

function initLeafletDualMaps() {
  const containerRaw = document.getElementById('leafletMapRaw');
  const containerFused = document.getElementById('leafletMapFused');
  if (!containerRaw || !containerFused || (leafletMapRaw && leafletMapFused)) return;

  const initLat = (currentGpsState.lat != null) ? currentGpsState.lat : 0.0;
  const initLon = (currentGpsState.lon != null) ? currentGpsState.lon : 0.0;

  // Custom Leaflet Markers
  const homeIcon = L.divIcon({
    className: 'custom-leaflet-icon-home',
    html: '<div style="background:#ef4444; width:14px; height:14px; border-radius:50%; border:2px solid #fff; box-shadow:0 0 8px #ef4444;"></div>',
    iconSize: [14, 14],
    iconAnchor: [7, 7]
  });

  const rawDroneIcon = L.divIcon({
    className: 'custom-leaflet-icon-raw',
    html: '<div style="background:#f43f5e; width:16px; height:16px; border-radius:50%; border:2px solid #fff; box-shadow:0 0 10px #f43f5e;"></div>',
    iconSize: [16, 16],
    iconAnchor: [8, 8]
  });

  const fusedDroneIcon = L.divIcon({
    className: 'custom-leaflet-icon-fused',
    html: '<div style="background:#06b6d4; width:18px; height:18px; border-radius:50%; border:3px solid #fff; box-shadow:0 0 14px #06b6d4;"></div>',
    iconSize: [18, 18],
    iconAnchor: [9, 9]
  });

  // 1. Initialize MAP 1: RAW UNFILTERED GPS
  leafletMapRaw = L.map('leafletMapRaw', { zoomControl: false, maxZoom: 28 }).setView([initLat, initLon], 17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 28,
    maxNativeZoom: 19,
    attribution: '© OpenStreetMap'
  }).addTo(leafletMapRaw);
  L.control.zoom({ position: 'topright' }).addTo(leafletMapRaw);

  originMarkerRaw = L.marker([initLat, initLon], { icon: homeIcon }).addTo(leafletMapRaw).bindPopup('Raw Home Origin');
  markerRaw = L.marker([initLat, initLon], { icon: rawDroneIcon }).addTo(leafletMapRaw).bindPopup('Raw Unfiltered GPS');
  polylineRaw = L.polyline([], { color: '#f43f5e', weight: 3, opacity: 0.75, dashArray: '4,6' }).addTo(leafletMapRaw);

  // 2. Initialize MAP 2: SENSOR FUSED GPS
  leafletMapFused = L.map('leafletMapFused', { zoomControl: false, maxZoom: 28 }).setView([initLat, initLon], 17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 28,
    maxNativeZoom: 19,
    attribution: '© OpenStreetMap'
  }).addTo(leafletMapFused);
  L.control.zoom({ position: 'topright' }).addTo(leafletMapFused);

  originMarkerFused = L.marker([initLat, initLon], { icon: homeIcon }).addTo(leafletMapFused).bindPopup('Fused Home Origin (0,0)');
  markerFused = L.marker([initLat, initLon], { icon: fusedDroneIcon }).addTo(leafletMapFused).bindPopup('50Hz Sensor Fused Position');
  polylineFused = L.polyline([], { color: '#06b6d4', weight: 4, opacity: 0.9 }).addTo(leafletMapFused);

  accuracyCircleFused = L.circle([initLat, initLon], {
    color: '#06b6d4',
    fillColor: '#06b6d4',
    fillOpacity: 0.15,
    radius: 0.5
  }).addTo(leafletMapFused);

  // Synchronize Pan & Zoom between maps
  function syncMaps(sourceMap, targetMap) {
    if (isSyncingMaps) return;
    isSyncingMaps = true;
    targetMap.setView(sourceMap.getCenter(), sourceMap.getZoom(), { animate: false });
    isSyncingMaps = false;
  }

  leafletMapRaw.on('move', () => syncMaps(leafletMapRaw, leafletMapFused));
  leafletMapFused.on('move', () => syncMaps(leafletMapFused, leafletMapRaw));
}

let geofenceRectRaw = null;
let geofenceRectFused = null;

function updateLeafletGeofenceRectangles(originLat, originLon) {
  if (originLat == null || originLon == null) return;

  const latOffset = (0.5 / 6378137.0) * (180.0 / Math.PI);
  const lonOffset = (0.5 / (6378137.0 * Math.cos(originLat * Math.PI / 180.0))) * (180.0 / Math.PI);

  const bounds = [
    [originLat - latOffset, originLon - lonOffset],
    [originLat + latOffset, originLon + lonOffset]
  ];

  if (leafletMapRaw) {
    if (!geofenceRectRaw) {
      geofenceRectRaw = L.rectangle(bounds, {
        color: '#10b981',
        weight: 2,
        fillColor: '#10b981',
        fillOpacity: 0.12,
        dashArray: '5,5'
      }).addTo(leafletMapRaw).bindPopup('1m x 1m Active Geofence');
    } else {
      geofenceRectRaw.setBounds(bounds);
    }
  }

  if (leafletMapFused) {
    if (!geofenceRectFused) {
      geofenceRectFused = L.rectangle(bounds, {
        color: '#10b981',
        weight: 2,
        fillColor: '#10b981',
        fillOpacity: 0.12,
        dashArray: '5,5'
      }).addTo(leafletMapFused).bindPopup('1m x 1m Active Geofence');
    } else {
      geofenceRectFused.setBounds(bounds);
    }
  }
}

function updateGpsTelemetry(gps, fusedGps, active3dPosX, active3dPosZ) {
  if (gps || fusedGps) {
    currentGpsState = { ...currentGpsState, ...gps, ...fusedGps };
  }

  const elFixBadge = document.getElementById('gpsFixBadge');
  const elSats = document.getElementById('gpsSats');
  const elLat = document.getElementById('gpsLat');
  const elLon = document.getElementById('gpsLon');
  const elProjX = document.getElementById('gpsProjX');
  const elProjY = document.getElementById('gpsProjY');

  const fixStatus = gps ? gps.fix_status : (fusedGps && fusedGps.raw_gps ? fusedGps.raw_gps.fix_status : 'SEARCHING...');
  const satsCount = gps ? gps.satellites : (fusedGps && fusedGps.raw_gps ? fusedGps.raw_gps.satellites : 0);

  if (elFixBadge) {
    elFixBadge.textContent = fixStatus || 'SEARCHING...';
    if (fixStatus && (fixStatus.includes('FIX') || fixStatus.includes('3D'))) {
      elFixBadge.className = 'status-badge-sm active';
    } else {
      elFixBadge.className = 'status-badge-sm';
    }
  }
  if (elSats) elSats.textContent = satsCount != null ? satsCount : 0;

  // Extract Raw vs Fused values
  const rawLat = (gps && gps.lat != null) ? gps.lat : (fusedGps && fusedGps.raw_gps ? fusedGps.raw_gps.lat : null);
  const rawLon = (gps && gps.lon != null) ? gps.lon : (fusedGps && fusedGps.raw_gps ? fusedGps.raw_gps.lon : null);

  const fusedLat = fusedGps ? fusedGps.fused_lat : rawLat;
  const fusedLon = fusedGps ? fusedGps.fused_lon : rawLon;

  if (elLat) elLat.textContent = fusedLat != null ? fusedLat.toFixed(6) : '-';
  if (elLon) elLon.textContent = fusedLon != null ? fusedLon.toFixed(6) : '-';

  // Projected X and Y meters (aligned with active zero-calibrated 3D cube coordinates)
  const projX = (typeof controllerGroup !== 'undefined' && controllerGroup) ? controllerGroup.position.x : ((active3dPosX !== undefined) ? active3dPosX : (fusedGps ? fusedGps.fused_x_m : (gps ? gps.projected_x_m : 0.0)));
  const projY = (typeof controllerGroup !== 'undefined' && controllerGroup) ? -controllerGroup.position.z : ((active3dPosZ !== undefined) ? -active3dPosZ : (fusedGps ? fusedGps.fused_y_m : (gps ? gps.projected_y_m : 0.0)));

  if (elProjX) elProjX.textContent = `${(projX || 0).toFixed(2)} m`;
  if (elProjY) elProjY.textContent = `${(projY || 0).toFixed(2)} m`;

  // Update Geofence 3D Hitbox Containment Status
  updateGeofenceHitboxCheck();

  // Update Modal Overlay Panel
  const elOvlStatus = document.getElementById('overlayGpsStatus');
  const elOvlSats = document.getElementById('overlayGpsSats');
  const elOvlSpeed = document.getElementById('overlayGpsSpeed');
  const elOvlAlt = document.getElementById('overlayGpsAlt');
  const elOvlRawLat = document.getElementById('overlayRawLat');
  const elOvlRawLon = document.getElementById('overlayRawLon');
  const elOvlLat = document.getElementById('overlayGpsLat');
  const elOvlLon = document.getElementById('overlayGpsLon');
  const elOvlX = document.getElementById('overlayGpsX');
  const elOvlY = document.getElementById('overlayGpsY');
  const elOvlOrigin = document.getElementById('overlayGpsOrigin');
  const elOvlAccuracy = document.getElementById('overlayGpsAccuracy');

  if (elOvlStatus) elOvlStatus.textContent = fixStatus || 'SEARCHING...';
  if (elOvlSats) elOvlSats.textContent = satsCount != null ? satsCount : 0;
  if (elOvlSpeed) elOvlSpeed.textContent = `${((fusedGps ? fusedGps.speed_kmh : (gps ? gps.speed_kmh : 0)) || 0).toFixed(1)} km/h`;
  if (elOvlAlt) elOvlAlt.textContent = `${((fusedGps ? fusedGps.fused_alt_m : (gps ? gps.alt_m : 0)) || 0).toFixed(1)} m`;

  if (elOvlRawLat) elOvlRawLat.textContent = `${rawLat != null ? rawLat.toFixed(6) : '-'}°`;
  if (elOvlRawLon) elOvlRawLon.textContent = `${rawLon != null ? rawLon.toFixed(6) : '-'}°`;

  if (elOvlLat) elOvlLat.textContent = `${fusedLat != null ? fusedLat.toFixed(6) : '-'}°`;
  if (elOvlLon) elOvlLon.textContent = `${fusedLon != null ? fusedLon.toFixed(6) : '-'}°`;

  if (elOvlX) elOvlX.textContent = `${(projX || 0).toFixed(2)} m`;
  if (elOvlY) elOvlY.textContent = `${(projY || 0).toFixed(2)} m`;

  if (elOvlAccuracy && fusedGps && fusedGps.accuracy_radius_m != null) {
    elOvlAccuracy.textContent = `±${fusedGps.accuracy_radius_m.toFixed(2)} m`;
  }

  const activeOrigin = (fusedGps && fusedGps.origin) || (gps && gps.origin) || currentGpsState.origin;
  if (elOvlOrigin && activeOrigin && activeOrigin.lat != null && activeOrigin.lon != null) {
    elOvlOrigin.textContent = `${activeOrigin.lat.toFixed(6)}, ${activeOrigin.lon.toFixed(6)}`;
    updateLeafletGeofenceRectangles(activeOrigin.lat, activeOrigin.lon);
  }

  // Update Leaflet Map 1: RAW UNFILTERED GPS
  if (leafletMapRaw && rawLat != null && rawLon != null) {
    const rawPos = [rawLat, rawLon];
    if (markerRaw) markerRaw.setLatLng(rawPos);
    if (activeOrigin && activeOrigin.lat != null && originMarkerRaw) {
      originMarkerRaw.setLatLng([activeOrigin.lat, activeOrigin.lon]);
    }
    rawTrailPoints.push(rawPos);
    if (rawTrailPoints.length > 500) rawTrailPoints.shift();
    if (polylineRaw) polylineRaw.setLatLngs(rawTrailPoints);
  }

  // Update Leaflet Map 2: 50Hz SENSOR FUSED GPS
  if (leafletMapFused && fusedLat != null && fusedLon != null) {
    const fusedPos = [fusedLat, fusedLon];
    if (markerFused) markerFused.setLatLng(fusedPos);
    if (activeOrigin && activeOrigin.lat != null && originMarkerFused) {
      originMarkerFused.setLatLng([activeOrigin.lat, activeOrigin.lon]);
    }

    if (accuracyCircleFused && fusedGps && fusedGps.accuracy_radius_m != null) {
      accuracyCircleFused.setLatLng(fusedPos);
      accuracyCircleFused.setRadius(fusedGps.accuracy_radius_m);
    }

    fusedTrailPoints.push(fusedPos);
    if (fusedTrailPoints.length > 500) fusedTrailPoints.shift();
    if (polylineFused) polylineFused.setLatLngs(fusedTrailPoints);
  }
}

const gpsMapDrawer = document.getElementById('gpsMapDrawer');
const btnToggleGpsMap = document.getElementById('btnToggleGpsMap');
const btnToggleGpsModal = document.getElementById('btnToggleGpsModal');
const btnCloseGpsDrawer = document.getElementById('btnCloseGpsDrawer');
const btnSetGpsOrigin = document.getElementById('btnSetGpsOrigin');

const btnMapSplit = document.getElementById('btnMapSplit');
const btnMapFused = document.getElementById('btnMapFused');
const btnMapRaw = document.getElementById('btnMapRaw');
const rawMapBox = document.getElementById('rawMapBox');
const fusedMapBox = document.getElementById('fusedMapBox');

function setMapViewMode(mode) {
  [btnMapSplit, btnMapFused, btnMapRaw].forEach(btn => btn?.classList.remove('active'));

  if (mode === 'split') {
    btnMapSplit?.classList.add('active');
    if (rawMapBox) rawMapBox.style.display = 'flex';
    if (fusedMapBox) fusedMapBox.style.display = 'flex';
    if (gpsMapDrawer) gpsMapDrawer.style.width = '880px';
  } else if (mode === 'fused') {
    btnMapFused?.classList.add('active');
    if (rawMapBox) rawMapBox.style.display = 'none';
    if (fusedMapBox) fusedMapBox.style.display = 'flex';
    if (gpsMapDrawer) gpsMapDrawer.style.width = '560px';
  } else if (mode === 'raw') {
    btnMapRaw?.classList.add('active');
    if (rawMapBox) rawMapBox.style.display = 'flex';
    if (fusedMapBox) fusedMapBox.style.display = 'none';
    if (gpsMapDrawer) gpsMapDrawer.style.width = '560px';
  }

  setTimeout(() => {
    if (leafletMapRaw) leafletMapRaw.invalidateSize();
    if (leafletMapFused) leafletMapFused.invalidateSize();
  }, 200);
}

btnMapSplit?.addEventListener('click', () => setMapViewMode('split'));
btnMapFused?.addEventListener('click', () => setMapViewMode('fused'));
btnMapRaw?.addEventListener('click', () => setMapViewMode('raw'));

function toggleGpsMapDrawer() {
  if (!gpsMapDrawer) return;
  const isHidden = gpsMapDrawer.classList.contains('hidden');
  if (isHidden) {
    gpsMapDrawer.classList.remove('hidden');
    btnToggleGpsMap?.classList.add('active');
    initLeafletDualMaps();
    setTimeout(() => {
      if (leafletMapRaw) leafletMapRaw.invalidateSize();
      if (leafletMapFused) leafletMapFused.invalidateSize();
    }, 200);
  } else {
    gpsMapDrawer.classList.add('hidden');
    btnToggleGpsMap?.classList.remove('active');
  }
}

btnToggleGpsMap?.addEventListener('click', toggleGpsMapDrawer);
btnToggleGpsModal?.addEventListener('click', toggleGpsMapDrawer);
btnCloseGpsDrawer?.addEventListener('click', toggleGpsMapDrawer);

btnSetGpsOrigin?.addEventListener('click', async () => {
  if (currentGpsState.lat != null && currentGpsState.lon != null) {
    try {
      await fetch('/api/gps/origin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ lat: currentGpsState.lat, lon: currentGpsState.lon })
      });
      alert(`Reference Home Origin set to (${currentGpsState.lat.toFixed(6)}, ${currentGpsState.lon.toFixed(6)})`);
    } catch (e) {
      console.error('Failed to set GPS origin:', e);
    }
  }
});



