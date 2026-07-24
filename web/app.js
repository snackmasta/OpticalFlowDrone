// ----------------------------------------------------
// 6DoF VR Controller 3D Trajectory Visualizer - Three.js
// ----------------------------------------------------

let scene, camera, renderer, controls;
let controllerGroup, trajectoryLine, gridHelper;
let handleMat, ringMat, stickMat;
let trajectoryPoints = [];
const MAX_TRAJECTORY_POINTS = 1000;
let isDarkMode = true;

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

  // Handle (Cylinder) - theme-inverted for high visibility
  const handleGeo = new THREE.CylinderGeometry(0.04, 0.035, 0.22, 16);
  handleMat = new THREE.MeshStandardMaterial({
    color: isDarkMode ? 0xe2e8f0 : 0x0f172a,
    roughness: isDarkMode ? 0.2 : 0.3,
    metalness: isDarkMode ? 0.5 : 0.8
  });
  const handleMesh = new THREE.Mesh(handleGeo, handleMat);
  handleMesh.rotation.x = Math.PI / 6;
  handleMesh.position.set(0, -0.05, 0);
  controllerGroup.add(handleMesh);

  // Tracking Ring (Torus)
  const ringGeo = new THREE.TorusGeometry(0.08, 0.012, 16, 32);
  ringMat = new THREE.MeshStandardMaterial({
    color: isDarkMode ? 0x06b6d4 : 0x0284c7,
    emissive: isDarkMode ? 0x06b6d4 : 0x0284c7,
    emissiveIntensity: isDarkMode ? 0.5 : 0.3,
    roughness: 0.2
  });
  const ringMesh = new THREE.Mesh(ringGeo, ringMat);
  ringMesh.rotation.x = Math.PI / 3;
  ringMesh.position.set(0, 0.06, 0.04);
  controllerGroup.add(ringMesh);

  // Joystick (Sphere + Shaft)
  const stickGeo = new THREE.SphereGeometry(0.015, 16, 16);
  stickMat = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0xec4899 : 0xdb2777 });
  const stickMesh = new THREE.Mesh(stickGeo, stickMat);
  stickMesh.position.set(0, 0.04, 0);
  controllerGroup.add(stickMesh);

  scene.add(controllerGroup);
  
  // Create 6DOF Robotic Linkage Arm connected from base (0,0,0)
  createRoboticArmLinkages();
}

// ----------------------------------------------------
// 6DOF Robotic Manipulator Arm & IK Solver (Base at 0,0,0)
// ----------------------------------------------------
let roboticArmGroup;
let j1Group, j2Group, j3Group, j4Group, j5Group, j6Group;

// Link lengths (meters)
const L1 = 0.25; // Base height (Joint 1 to Joint 2)
const L2 = 0.40; // Upper arm length (Joint 2 to Joint 3)
const L3 = 0.35; // Forearm length (Joint 3 to Wrist Center)
const L4 = 0.12; // Wrist offset (Wrist Center to Controller tip)

function createRoboticArmLinkages() {
  roboticArmGroup = new THREE.Group();
  scene.add(roboticArmGroup);

  // Materials
  const baseMat = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x1e293b : 0x475569, metalness: 0.8, roughness: 0.2 });
  const jointMat = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0xec4899 : 0xdb2777, metalness: 0.6, roughness: 0.3 });
  const linkMat1 = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x06b6d4 : 0x0284c7, metalness: 0.7, roughness: 0.3 });
  const linkMat2 = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x3b82f6 : 0x2563eb, metalness: 0.7, roughness: 0.3 });

  // Base Pedestal at (0, 0, 0)
  const basePedestal = new THREE.Mesh(
    new THREE.CylinderGeometry(0.12, 0.16, 0.05, 32),
    baseMat
  );
  basePedestal.position.set(0, -0.025, 0);
  roboticArmGroup.add(basePedestal);

  // Joint 1 Group (Base Yaw - rot around Y at 0,0,0)
  j1Group = new THREE.Group();
  roboticArmGroup.add(j1Group);

  // Link 1 (Base column height L1)
  const link1Mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.045, 0.055, L1, 24),
    linkMat1
  );
  link1Mesh.position.set(0, L1 / 2, 0);
  j1Group.add(link1Mesh);

  // Joint 1 housing / pivot sphere
  const j1Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.05, 24, 24), jointMat);
  j1Sphere.position.set(0, L1, 0);
  j1Group.add(j1Sphere);

  // Joint 2 Group (Shoulder Pitch - pivot at 0, L1, 0)
  j2Group = new THREE.Group();
  j2Group.position.set(0, L1, 0);
  j1Group.add(j2Group);

  // Link 2 (Upper Arm length L2)
  const link2Mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.038, 0.042, L2, 24),
    linkMat2
  );
  link2Mesh.position.set(0, L2 / 2, 0);
  j2Group.add(link2Mesh);

  const j2Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.045, 24, 24), jointMat);
  j2Sphere.position.set(0, L2, 0);
  j2Group.add(j2Sphere);

  // Joint 3 Group (Elbow Pitch - pivot at 0, L2, 0)
  j3Group = new THREE.Group();
  j3Group.position.set(0, L2, 0);
  j2Group.add(j3Group);

  // Link 3 (Forearm length L3)
  const link3Mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.03, 0.035, L3, 24),
    linkMat1
  );
  link3Mesh.position.set(0, L3 / 2, 0);
  j3Group.add(link3Mesh);

  const j3Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.038, 24, 24), jointMat);
  j3Sphere.position.set(0, L3, 0);
  j3Group.add(j3Sphere);

  // Joint 4 Group (Forearm Roll - pivot at 0, L3, 0)
  j4Group = new THREE.Group();
  j4Group.position.set(0, L3, 0);
  j3Group.add(j4Group);

  // Joint 5 Group (Wrist Pitch)
  j5Group = new THREE.Group();
  j4Group.add(j5Group);

  const j5Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.032, 24, 24), jointMat);
  j5Group.add(j5Sphere);

  // Joint 6 Group (Wrist Roll / Flange)
  j6Group = new THREE.Group();
  j5Group.add(j6Group);

  const flangeMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.025, 0.03, L4, 24),
    linkMat2
  );
  flangeMesh.position.set(0, L4 / 2, 0);
  j6Group.add(flangeMesh);

  // Attach VR Controller mesh to the tip of Joint 6 flange (0, L4, 0)
  if (controllerGroup) {
    scene.remove(controllerGroup);
    controllerGroup.position.set(0, L4, 0);
    controllerGroup.rotation.set(0, 0, 0);
    j6Group.add(controllerGroup);
  }
}

// Compute 6-DOF Inverse Kinematics for Manipulator Arm
function updateRoboticArm() {
  if (!j1Group || !currentTargetPos) return;

  const elJ1 = document.getElementById('j1Angle');
  const elJ2 = document.getElementById('j2Angle');
  const elJ3 = document.getElementById('j3Angle');
  const elJ4 = document.getElementById('j4Angle');
  const elJ5 = document.getElementById('j5Angle');
  const elJ6 = document.getElementById('j6Angle');

  // Target end-effector position and orientation
  const targetPos = currentTargetPos;
  const targetQuat = currentTargetQuat;

  // Tool direction vector (Y-axis of target end-effector)
  const toolDir = new THREE.Vector3(0, 1, 0).applyQuaternion(targetQuat);

  // Wrist Center Position (P_wc = P_target - L4 * toolDir)
  const pWC = new THREE.Vector3().copy(targetPos).sub(toolDir.clone().multiplyScalar(L4));

  // 1. Joint 1: Base Yaw (rotation around Y-axis at base)
  const theta1 = Math.atan2(pWC.x, pWC.z);

  // 2. Joint 2 & 3: Planar Shoulder and Elbow Pitch
  const r = Math.sqrt(pWC.x * pWC.x + pWC.z * pWC.z); // Horizontal radial distance
  const yPrime = pWC.y - L1; // Height relative to Shoulder joint

  let D = Math.sqrt(r * r + yPrime * yPrime);
  // Clamp reach D to valid reach limits
  const maxReach = L2 + L3 - 0.001;
  const minReach = Math.abs(L2 - L3) + 0.001;
  D = THREE.MathUtils.clamp(D, minReach, maxReach);

  // Cosine law for elbow interior angle
  const cosGamma = (L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3);
  const gamma = Math.acos(THREE.MathUtils.clamp(cosGamma, -1, 1));
  const theta3 = Math.PI - gamma; // Elbow bend angle

  // Shoulder angle calculation
  const phi1 = Math.atan2(yPrime, r);
  const cosPhi2 = (L2 * L2 + D * D - L3 * L3) / (2 * L2 * D);
  const phi2 = Math.acos(THREE.MathUtils.clamp(cosPhi2, -1, 1));
  const theta2 = (Math.PI / 2) - (phi1 + phi2);

  // Apply positional joint angles (J1, J2, J3)
  j1Group.rotation.y = theta1;
  j2Group.rotation.z = -theta2;
  j3Group.rotation.z = -theta3;

  // 3. Wrist Orientation (Joints 4, 5, 6)
  // Compute forward orientation matrix of Frame 3 (up to Joint 3)
  const qJ1 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), theta1);
  const qJ2 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -theta2);
  const qJ3 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -theta3);

  const qArm3 = new THREE.Quaternion().copy(qJ1).multiply(qJ2).multiply(qJ3);
  
  // Relative quaternion for Wrist R_36 = R_03^T * R_target
  const qWrist = qArm3.clone().invert().multiply(targetQuat);
  const eulerWrist = new THREE.Euler().setFromQuaternion(qWrist, 'YXZ');

  const theta4 = eulerWrist.y;
  const theta5 = eulerWrist.x;
  const theta6 = eulerWrist.z;

  j4Group.rotation.y = theta4;
  j5Group.rotation.x = theta5;
  j6Group.rotation.z = theta6;

  // Force update matrix world to obtain accurate world position of tool endpoint
  roboticArmGroup.updateMatrixWorld(true);

  if (controllerGroup) {
    const toolEndpointPos = new THREE.Vector3();
    controllerGroup.getWorldPosition(toolEndpointPos);

    const lastPoint = trajectoryPoints[trajectoryPoints.length - 1];
    if (!lastPoint || lastPoint.distanceTo(toolEndpointPos) > 0.01) {
      addTrajectoryPoint(toolEndpointPos.x, toolEndpointPos.y, toolEndpointPos.z);
    }
  }

  // Update Telemetry HUD with angles in degrees
  if (elJ1) elJ1.textContent = `${THREE.MathUtils.radToDeg(theta1).toFixed(1)}°`;
  if (elJ2) elJ2.textContent = `${THREE.MathUtils.radToDeg(theta2).toFixed(1)}°`;
  if (elJ3) elJ3.textContent = `${THREE.MathUtils.radToDeg(theta3).toFixed(1)}°`;
  if (elJ4) elJ4.textContent = `${THREE.MathUtils.radToDeg(theta4).toFixed(1)}°`;
  if (elJ5) elJ5.textContent = `${THREE.MathUtils.radToDeg(theta5).toFixed(1)}°`;
  if (elJ6) elJ6.textContent = `${THREE.MathUtils.radToDeg(theta6).toFixed(1)}°`;
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

  // Calculate position adjusted for origin reset (inverting Y and Z for intuitive 3D camera coordinate space)
  const posX = rawPos.x - positionOffset.x;
  const posY = -(rawPos.y - positionOffset.y);
  const posZ = -(rawPos.z - positionOffset.z);

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

  // Update 6DOF Robotic Manipulator Arm via Inverse Kinematics
  updateRoboticArm();

  // Smoothly track controller target position with camera target if follow mode is active
  if (followMode && currentTargetPos) {
    controls.target.lerp(currentTargetPos, 0.1);
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

  // Invert Controller handle & ring colors for maximum contrast against theme background
  if (handleMat) {
    handleMat.color.setHex(isDark ? 0xe2e8f0 : 0x0f172a);
    handleMat.metalness = isDark ? 0.5 : 0.8;
    handleMat.roughness = isDark ? 0.2 : 0.3;
    handleMat.needsUpdate = true;
  }
  if (ringMat) {
    ringMat.color.setHex(isDark ? 0x06b6d4 : 0x0284c7);
    ringMat.emissive.setHex(isDark ? 0x06b6d4 : 0x0284c7);
    ringMat.emissiveIntensity = isDark ? 0.5 : 0.3;
    ringMat.needsUpdate = true;
  }
  if (stickMat) {
    stickMat.color.setHex(isDark ? 0xec4899 : 0xdb2777);
    stickMat.needsUpdate = true;
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


