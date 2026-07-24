// ----------------------------------------------------
// 6DoF VR Controller 3D Trajectory Visualizer - Three.js
// ----------------------------------------------------

let scene, camera, renderer, controls;
let controllerGroup, trajectoryLine, gridHelper, ceilingGrid;
let handleMat, ringMat, stickMat;
let trajectoryPoints = [];
const MAX_TRAJECTORY_POINTS = 1000;
let isDarkMode = true;

// Base Roof Mount Constants (Hanging arm from ceiling)
const ROOF_Y = 1.8; // Ceiling mounting height in meters
const L1 = 0.25;    // Base column height (Roof down to Joint 2)
const L2 = 0.40;    // Upper arm length (Joint 2 to Joint 3)
const L3 = 0.35;    // Forearm length (Joint 3 to Wrist Center)
const L4 = 0.12;    // Wrist offset (Wrist Center to Controller tip)
const TOTAL_ARM_LENGTH = L1 + L2 + L3 + L4; // 1.12m
const RESTING_TIP_Y = ROOF_Y - TOTAL_ARM_LENGTH; // 0.68m hanging straight down

// Robot Base Pose Configuration State (Position X,Y,Z & Orientation Roll,Pitch,Yaw)
let robotBaseConfig = {
  x: 0,
  y: ROOF_Y,
  z: 0,
  roll: 0,
  pitch: 0,
  yaw: 0
};

// Hardware IP & Live UDP Control State
let robotIP = '192.168.137.94';
let hardwareSyncEnabled = true;
let lastSentHardwareTime = 0;
let lastSentServoValues = { a1: -1, a2: -1, a3: -1, a4: -1 };

function loadRobotIPConfig() {
  try {
    const savedIP = localStorage.getItem('robot_arm_ip');
    if (savedIP) robotIP = savedIP;
    const savedSync = localStorage.getItem('robot_hardware_sync');
    if (savedSync !== null) hardwareSyncEnabled = savedSync === 'true';
  } catch (e) {}
}

function saveRobotIPConfig() {
  try {
    localStorage.setItem('robot_arm_ip', robotIP);
    localStorage.setItem('robot_hardware_sync', hardwareSyncEnabled ? 'true' : 'false');
  } catch (e) {}
}

function syncIPUI() {
  const elHeaderIP = document.getElementById('inputRobotIPHeader');
  const elDrawerIP = document.getElementById('inputRobotIPDrawer');
  const elHeaderSync = document.getElementById('chkEnableHardwareSync');
  const elDrawerSync = document.getElementById('chkEnableHardwareSyncDrawer');

  if (elHeaderIP) elHeaderIP.value = robotIP;
  if (elDrawerIP) elDrawerIP.value = robotIP;
  if (elHeaderSync) elHeaderSync.checked = hardwareSyncEnabled;
  if (elDrawerSync) elDrawerSync.checked = hardwareSyncEnabled;
}

function updateIPFromUI(val) {
  if (val && val.trim()) {
    robotIP = val.trim();
    syncIPUI();
    saveRobotIPConfig();
  }
}

function updateHardwareSyncFromUI(enabled) {
  hardwareSyncEnabled = enabled;
  syncIPUI();
  saveRobotIPConfig();
}

let lastSendTime = 0;
let pendingSendTimeout = null;
let lastSentServoValues = { a1: -1, a2: -1, a3: -1, a4: -1 };

function updateHardwareStatusText(txt) {
  const el = document.getElementById('hardwareStatusText');
  if (el) el.textContent = txt;
}

function sendHardwareArmCommand(a1, a2, a3, a4) {
  if (!hardwareSyncEnabled) return;

  const intA1 = Math.round(THREE.MathUtils.clamp(a1, 0, 180));
  const intA2 = Math.round(THREE.MathUtils.clamp(a2, 0, 180));
  const intA3 = Math.round(THREE.MathUtils.clamp(a3, 0, 180));
  const intA4 = Math.round(THREE.MathUtils.clamp(a4, 0, 180));

  // Skip sending duplicate commands if angles have not changed
  if (
    intA1 === lastSentServoValues.a1 &&
    intA2 === lastSentServoValues.a2 &&
    intA3 === lastSentServoValues.a3 &&
    intA4 === lastSentServoValues.a4
  ) {
    return;
  }

  const executeSend = () => {
    lastSentServoValues = { a1: intA1, a2: intA2, a3: intA3, a4: intA4 };
    fetch(`/send?ip=${encodeURIComponent(robotIP)}&a1=${intA1}&a2=${intA2}&a3=${intA3}&a4=${intA4}`)
      .then(res => res.json())
      .then(data => {
        if (data && data.status === 'ok') {
          updateHardwareStatusText(`Sent -> IP: ${data.ip}:${data.port} | S1: ${data.a1}°, S2: ${data.a2}°, S3: ${data.a3}°, S4: ${data.a4}°`);
        } else {
          updateHardwareStatusText(`Error [${data.ip || robotIP}]: ${data.message || 'Transmission failed'}`);
        }
      })
      .catch(err => {
        updateHardwareStatusText(`Network error connecting to server`);
      });
  };

  const now = Date.now();
  if (now - lastSendTime > 25) { // 40Hz smooth realtime control (matching gui_server.py)
    executeSend();
    lastSendTime = now;
  } else {
    clearTimeout(pendingSendTimeout);
    pendingSendTimeout = setTimeout(() => {
      executeSend();
      lastSendTime = Date.now();
    }, 25);
  }
}

// Control Mode & Manual Overrides State ('telemetry', 'manual_fk', 'manual_ik')
let controlMode = 'telemetry';
let manualFKState = { s1: 90, s2: 90, s3: 90, s4: 90 };
let manualIKState = { x: 0, y: RESTING_TIP_Y, z: 0, roll: 0, pitch: 0, yaw: 0 };

// Origin offset for resetting zero-position
let positionOffset = { x: 0, y: 0, z: 0 };
let rawLatestPos = { x: 0, y: 0, z: 0 };
let currentTargetPos = new THREE.Vector3(0, RESTING_TIP_Y, 0);
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
  camera.position.set(1.6, 1.4, 2.2);

  // Renderer
  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;

  // Orbit Controls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, 1.0, 0);

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

  // Floor Grid
  gridHelper = new THREE.GridHelper(10, 20, isDarkMode ? 0x06b6d4 : 0x0284c7, isDarkMode ? 0x334155 : 0xcbd5e1);
  gridHelper.position.y = 0;
  scene.add(gridHelper);

  // Ceiling Grid Structure (Roof Surface)
  ceilingGrid = new THREE.GridHelper(6, 12, isDarkMode ? 0xec4899 : 0xdb2777, isDarkMode ? 0x334155 : 0xcbd5e1);
  ceilingGrid.position.y = ROOF_Y;
  scene.add(ceilingGrid);

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
  
  // Create 6DOF Robotic Linkage Arm hanging from roof (0, ROOF_Y, 0)
  createRoboticArmLinkages();
}

// ----------------------------------------------------
// 4-DOF Robotic Manipulator Arm & Kinematics (J1, J2, J3, J4)
// ----------------------------------------------------
let roboticArmGroup;
let j1Group, j2Group, j3Group, j4Group;

function loadRobotBaseConfig() {
  try {
    const saved = localStorage.getItem('slimevr_robot_base_config');
    if (saved) {
      robotBaseConfig = { ...robotBaseConfig, ...JSON.parse(saved) };
    }
  } catch (e) {
    console.warn('Could not load saved robot base config', e);
  }
}

function saveRobotBaseConfig() {
  try {
    localStorage.setItem('slimevr_robot_base_config', JSON.stringify(robotBaseConfig));
  } catch (e) {}
}

function syncRobotBaseUI() {
  const elX = document.getElementById('inputBaseX');
  const elY = document.getElementById('inputBaseY');
  const elZ = document.getElementById('inputBaseZ');
  const elRoll = document.getElementById('inputBaseRoll');
  const elPitch = document.getElementById('inputBasePitch');
  const elYaw = document.getElementById('inputBaseYaw');

  if (elX) elX.value = robotBaseConfig.x.toFixed(2);
  if (elY) elY.value = robotBaseConfig.y.toFixed(2);
  if (elZ) elZ.value = robotBaseConfig.z.toFixed(2);
  if (elRoll) elRoll.value = robotBaseConfig.roll.toFixed(0);
  if (elPitch) elPitch.value = robotBaseConfig.pitch.toFixed(0);
  if (elYaw) elYaw.value = robotBaseConfig.yaw.toFixed(0);
}

function updateRobotBaseFromUI() {
  const elX = document.getElementById('inputBaseX');
  const elY = document.getElementById('inputBaseY');
  const elZ = document.getElementById('inputBaseZ');
  const elRoll = document.getElementById('inputBaseRoll');
  const elPitch = document.getElementById('inputBasePitch');
  const elYaw = document.getElementById('inputBaseYaw');

  if (elX) robotBaseConfig.x = parseFloat(elX.value) || 0;
  if (elY) robotBaseConfig.y = parseFloat(elY.value) || 0;
  if (elZ) robotBaseConfig.z = parseFloat(elZ.value) || 0;
  if (elRoll) robotBaseConfig.roll = parseFloat(elRoll.value) || 0;
  if (elPitch) robotBaseConfig.pitch = parseFloat(elPitch.value) || 0;
  if (elYaw) robotBaseConfig.yaw = parseFloat(elYaw.value) || 0;

  updateRobotBaseTransform();
  saveRobotBaseConfig();
}

function updateRobotBaseTransform() {
  if (!roboticArmGroup) return;

  roboticArmGroup.position.set(robotBaseConfig.x, robotBaseConfig.y, robotBaseConfig.z);

  const rollRad = THREE.MathUtils.degToRad(robotBaseConfig.roll);
  const pitchRad = THREE.MathUtils.degToRad(robotBaseConfig.pitch);
  const yawRad = THREE.MathUtils.degToRad(robotBaseConfig.yaw);

  const euler = new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ');
  roboticArmGroup.rotation.copy(euler);

  if (ceilingGrid) {
    ceilingGrid.position.set(robotBaseConfig.x, robotBaseConfig.y, robotBaseConfig.z);
    ceilingGrid.rotation.copy(euler);
  }
}

function createRoboticArmLinkages() {
  roboticArmGroup = new THREE.Group();
  scene.add(roboticArmGroup);
  updateRobotBaseTransform();

  // Materials
  const roofPlateMat = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x0f172a : 0x334155, metalness: 0.9, roughness: 0.2 });
  const baseMat = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x1e293b : 0x475569, metalness: 0.8, roughness: 0.2 });
  const jointMat = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0xec4899 : 0xdb2777, metalness: 0.6, roughness: 0.3 });
  const linkMat1 = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x06b6d4 : 0x0284c7, metalness: 0.7, roughness: 0.3 });
  const linkMat2 = new THREE.MeshStandardMaterial({ color: isDarkMode ? 0x3b82f6 : 0x2563eb, metalness: 0.7, roughness: 0.3 });

  // Roof Ceiling Structural Slab
  const roofSlabGeo = new THREE.CylinderGeometry(0.7, 0.7, 0.03, 32);
  const roofSlabMat = new THREE.MeshStandardMaterial({
    color: isDarkMode ? 0x0f172a : 0x475569,
    metalness: 0.8,
    roughness: 0.3
  });
  const roofSlab = new THREE.Mesh(roofSlabGeo, roofSlabMat);
  roofSlab.position.set(0, 0.015, 0);
  roboticArmGroup.add(roofSlab);

  // Inverted Base Mounting Flange
  const roofPlate = new THREE.Mesh(
    new THREE.CylinderGeometry(0.32, 0.35, 0.03, 32),
    roofPlateMat
  );
  roofPlate.position.set(0, -0.015, 0);
  roboticArmGroup.add(roofPlate);

  // Glowing Cyan LED Accent Ring
  const roofRingMat = new THREE.MeshStandardMaterial({
    color: isDarkMode ? 0x06b6d4 : 0x0284c7,
    emissive: isDarkMode ? 0x06b6d4 : 0x0284c7,
    emissiveIntensity: 0.8
  });
  const roofRing = new THREE.Mesh(new THREE.TorusGeometry(0.33, 0.01, 16, 32), roofRingMat);
  roofRing.rotation.x = Math.PI / 2;
  roofRing.position.set(0, -0.03, 0);
  roboticArmGroup.add(roofRing);

  // Base Pedestal Mount
  const basePedestal = new THREE.Mesh(
    new THREE.CylinderGeometry(0.20, 0.12, 0.06, 32),
    baseMat
  );
  basePedestal.position.set(0, -0.05, 0);
  roboticArmGroup.add(basePedestal);

  // Joint 1 Group (Base Yaw)
  j1Group = new THREE.Group();
  roboticArmGroup.add(j1Group);

  // Link 1 (Base column along -Y by L1)
  const link1Mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.055, 0.045, L1, 24),
    linkMat1
  );
  link1Mesh.position.set(0, -L1 / 2, 0);
  j1Group.add(link1Mesh);

  const j1Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.05, 24, 24), jointMat);
  j1Sphere.position.set(0, -L1, 0);
  j1Group.add(j1Sphere);

  // Joint 2 Group (Shoulder Pitch - pivot at 0, -L1, 0)
  j2Group = new THREE.Group();
  j2Group.position.set(0, -L1, 0);
  j1Group.add(j2Group);

  // Link 2 (Upper Arm length L2)
  const link2Mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.042, 0.038, L2, 24),
    linkMat2
  );
  link2Mesh.position.set(0, -L2 / 2, 0);
  j2Group.add(link2Mesh);

  const j2Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.045, 24, 24), jointMat);
  j2Sphere.position.set(0, -L2, 0);
  j2Group.add(j2Sphere);

  // Joint 3 Group (Elbow Pitch - pivot at 0, -L2, 0)
  j3Group = new THREE.Group();
  j3Group.position.set(0, -L2, 0);
  j2Group.add(j3Group);

  // Link 3 (Forearm length L3)
  const link3Mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.035, 0.03, L3, 24),
    linkMat1
  );
  link3Mesh.position.set(0, -L3 / 2, 0);
  j3Group.add(link3Mesh);

  const j3Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.038, 24, 24), jointMat);
  j3Sphere.position.set(0, -L3, 0);
  j3Group.add(j3Sphere);

  // Joint 4 Group (Wrist Pitch - attached directly to Joint 3 at 0, -L3, 0)
  j4Group = new THREE.Group();
  j4Group.position.set(0, -L3, 0);
  j3Group.add(j4Group);

  const j4Sphere = new THREE.Mesh(new THREE.SphereGeometry(0.032, 24, 24), jointMat);
  j4Group.add(j4Sphere);

  const flangeMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.03, 0.025, L4, 24),
    linkMat2
  );
  flangeMesh.position.set(0, -L4 / 2, 0);
  j4Group.add(flangeMesh);

  // Attach VR Controller mesh directly to Joint 4 flange tip (0, -L4, 0)
  if (controllerGroup) {
    scene.remove(controllerGroup);
    controllerGroup.position.set(0, -L4, 0);
    controllerGroup.rotation.set(0, 0, 0);
    j4Group.add(controllerGroup);
  }
}

// Compute 4-DOF Kinematics for Manipulator Arm (J1, J2, J3, J4)
function updateRoboticArm() {
  if (!j1Group || !roboticArmGroup) return;

  const elJ1 = document.getElementById('j1Angle');
  const elJ2 = document.getElementById('j2Angle');
  const elJ3 = document.getElementById('j3Angle');
  const elJ4 = document.getElementById('j4Angle');

  if (controlMode === 'manual_fk') {
    // ------------------------------------------------
    // Manual Forward Kinematics (Servo Angles 0° - 180°, centered at 90°)
    // ------------------------------------------------
    const s1 = manualFKState.s1 !== undefined ? manualFKState.s1 : 90; // Servo 1 (GPIO 1 / Elbow Pitch J3)
    const s2 = manualFKState.s2 !== undefined ? manualFKState.s2 : 90; // Servo 2 (GPIO 3 / Shoulder Pitch J2)
    const s3 = manualFKState.s3 !== undefined ? manualFKState.s3 : 90; // Servo 3 (GPIO 5 / Base Yaw J1)
    const s4 = manualFKState.s4 !== undefined ? manualFKState.s4 : 90; // Servo 4 (GPIO 4 / Wrist Pitch J4)

    // Convert Servo 0-180° into 3D rendering rotation angles relative to 90° center
    const theta1 = THREE.MathUtils.degToRad(s3 - 90); // Base Yaw J1 = Servo 3
    const theta2 = THREE.MathUtils.degToRad(s2 - 90); // Shoulder Pitch J2 = Servo 2
    const theta3 = THREE.MathUtils.degToRad(s1 - 90); // Elbow Pitch J3 = Servo 1
    const theta4 = THREE.MathUtils.degToRad(s4 - 90); // Wrist Pitch J4 = Servo 4

    j1Group.rotation.y = theta1;
    j2Group.rotation.z = -theta2;
    j3Group.rotation.z = -theta3;
    j4Group.rotation.x = theta4;

    if (elJ1) elJ1.textContent = `${s3.toFixed(0)}°`;
    if (elJ2) elJ2.textContent = `${s2.toFixed(0)}°`;
    if (elJ3) elJ3.textContent = `${s1.toFixed(0)}°`;
    if (elJ4) elJ4.textContent = `${s4.toFixed(0)}°`;

    // Stream UDP packet: a1=Servo1, a2=Servo2, a3=Servo3, a4=Servo4
    sendHardwareArmCommand(s1, s2, s3, s4);

  } else {
    // ------------------------------------------------
    // Inverse Kinematics Mode (Telemetry or Manual IK)
    // ------------------------------------------------
    let targetPos, targetQuat;

    if (controlMode === 'manual_ik') {
      targetPos = new THREE.Vector3(manualIKState.x, manualIKState.y, manualIKState.z);
      const rollRad = THREE.MathUtils.degToRad(manualIKState.roll);
      const pitchRad = THREE.MathUtils.degToRad(manualIKState.pitch);
      const yawRad = THREE.MathUtils.degToRad(manualIKState.yaw);
      targetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ'));
    } else {
      if (!currentTargetPos) return;
      targetPos = currentTargetPos;
      targetQuat = currentTargetQuat;
    }

    // Tool direction vector (in resting pose, points DOWN along -Y axis in local tool frame)
    const toolDir = new THREE.Vector3(0, -1, 0).applyQuaternion(targetQuat);

    // Wrist Center Position in World Coordinates: P_wc_world = targetPos - L4 * toolDir
    const pWCWorld = new THREE.Vector3().copy(targetPos).sub(toolDir.clone().multiplyScalar(L4));

    // Transform Wrist Center & target quaternion into Robot Base Local Frame
    const basePos = roboticArmGroup.position;
    const baseQuat = roboticArmGroup.quaternion;
    const baseQuatInv = baseQuat.clone().invert();

    // Wrist center position relative to base frame origin
    const pWCLocal = pWCWorld.clone().sub(basePos).applyQuaternion(baseQuatInv);

    // Target orientation relative to base frame orientation
    const targetQuatLocal = baseQuatInv.clone().multiply(targetQuat);

    // Shoulder Pivot position in Base Local Space is (0, -L1, 0)
    const dx = pWCLocal.x;
    const dz = pWCLocal.z;
    const dy = pWCLocal.y - (-L1); // dy relative to shoulder pivot (0, -L1, 0)

    // 1. Joint 1: Base Yaw (rotation around vertical local Y-axis)
    const theta1 = Math.atan2(dx, dz);

    // 2. Joint 2 & 3: Planar Shoulder and Elbow Pitch in local base frame
    const r = Math.sqrt(dx * dx + dz * dz); // Horizontal radial distance
    let D = Math.sqrt(r * r + dy * dy);    // Distance from Shoulder Pivot to Wrist Center

    // Clamp reach D to valid kinematics limits
    const maxReach = L2 + L3 - 0.001;
    const minReach = Math.abs(L2 - L3) + 0.001;
    D = THREE.MathUtils.clamp(D, minReach, maxReach);

    // Cosine law for elbow interior angle (gamma)
    const cosGamma = (L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3);
    const gamma = Math.acos(THREE.MathUtils.clamp(cosGamma, -1, 1));
    const theta3 = Math.PI - gamma; // Elbow bend angle (0 = fully extended straight down)

    // Shoulder angle relative to straight down (-Y direction in local frame)
    const alpha = Math.atan2(r, -dy);
    const cosBeta = (L2 * L2 + D * D - L3 * L3) / (2 * L2 * D);
    const beta = Math.acos(THREE.MathUtils.clamp(cosBeta, -1, 1));
    const theta2 = alpha + beta; // Shoulder pitch angle

    // Apply positional joint angles (J1, J2, J3)
    j1Group.rotation.y = theta1;
    j2Group.rotation.z = -theta2;
    j3Group.rotation.z = -theta3;

    // 3. Wrist Pitch (Joint 4) in local frame
    const qJ1 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), theta1);
    const qJ2 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -theta2);
    const qJ3 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -theta3);

    const qArm3 = new THREE.Quaternion().copy(qJ1).multiply(qJ2).multiply(qJ3);

    // Relative quaternion for Wrist Pitch: R_wrist = R_03^T * R_targetLocal
    const qWrist = qArm3.clone().invert().multiply(targetQuatLocal);
    const eulerWrist = new THREE.Euler().setFromQuaternion(qWrist, 'YXZ');

    const theta4 = eulerWrist.x;
    j4Group.rotation.x = theta4;

    // Compute servo angles (0° - 180°, centered at 90°)
    const s3 = Math.round(THREE.MathUtils.clamp(90 + THREE.MathUtils.radToDeg(theta1), 0, 180));
    const s2 = Math.round(THREE.MathUtils.clamp(90 + THREE.MathUtils.radToDeg(theta2), 0, 180));
    const s1 = Math.round(THREE.MathUtils.clamp(90 + THREE.MathUtils.radToDeg(theta3), 0, 180));
    const s4 = Math.round(THREE.MathUtils.clamp(90 + THREE.MathUtils.radToDeg(theta4), 0, 180));

    // Update Telemetry HUD with angles in degrees
    if (elJ1) elJ1.textContent = `${s3}°`;
    if (elJ2) elJ2.textContent = `${s2}°`;
    if (elJ3) elJ3.textContent = `${s1}°`;
    if (elJ4) elJ4.textContent = `${s4}°`;

    // Stream commands to physical ESP8266 robot arm over UDP
    sendHardwareArmCommand(s1, s2, s3, s4);
  }

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
  if (!elStatusBadge || !elStatusText) return;

  // Set initial status immediately so UI is never stuck on "Connecting..."
  elStatusBadge.className = 'status-badge disconnected';
  elStatusText.textContent = 'Manual Debug Mode';

  try {
    const evtSource = new EventSource('/stream');

    evtSource.onopen = () => {
      elStatusBadge.className = 'status-badge connected';
      elStatusText.textContent = 'Streaming Live 50Hz';
    };

    evtSource.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.status === 'connected') {
          elStatusBadge.className = 'status-badge connected';
          elStatusText.textContent = 'Streaming Live 50Hz';
        } else {
          elStatusBadge.className = 'status-badge disconnected';
          elStatusText.textContent = 'Manual Debug Mode';
        }
        updateTelemetry(data);
      } catch (e) {
        console.error('JSON parse error:', e);
      }
    };

    evtSource.onerror = () => {
      elStatusBadge.className = 'status-badge disconnected';
      elStatusText.textContent = 'Manual Debug Mode';
    };
  } catch (e) {
    elStatusBadge.className = 'status-badge disconnected';
    elStatusText.textContent = 'Manual Debug Mode';
  }
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

  // Calculate position adjusted for origin reset (hanging from ceiling with resting position tip at RESTING_TIP_Y)
  const posX = rawPos.x - positionOffset.x;
  const posY = RESTING_TIP_Y - (rawPos.y - positionOffset.y);
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

document.getElementById('btnCalibrateDrift')?.addEventListener('click', () => {
  positionOffset = { ...rawLatestPos };
  trajectoryPoints = [];
  if (trajectoryLine && trajectoryLine.geometry) trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnToggleFollow')?.addEventListener('click', (e) => {
  followMode = !followMode;
  e.currentTarget.classList.toggle('active', followMode);
});

document.getElementById('btnResetTrail')?.addEventListener('click', () => {
  trajectoryPoints = [];
  if (trajectoryLine && trajectoryLine.geometry) trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnResetPos')?.addEventListener('click', () => {
  positionOffset = { ...currentTargetPos };
  trajectoryPoints = [];
  if (trajectoryLine && trajectoryLine.geometry) trajectoryLine.geometry.setDrawRange(0, 0);
});

document.getElementById('btnRecenterCam')?.addEventListener('click', () => {
  if (camera) camera.position.set(1.6, 1.4, 2.2);
  if (controls) controls.target.set(0, 1.0, 0);
});

let gridVisible = true;
document.getElementById('btnToggleGrid')?.addEventListener('click', (e) => {
  gridVisible = !gridVisible;
  if (gridHelper) gridHelper.visible = gridVisible;
  if (ceilingGrid) ceilingGrid.visible = gridVisible;
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
// Robot Base Config UI Binding & Handlers
// ----------------------------------------------------
const elBaseConfigDrawer = document.getElementById('baseConfigDrawer');
const elBtnToggleBaseConfig = document.getElementById('btnToggleBaseConfig');
const elBtnQuickRobotBase = document.getElementById('btnQuickRobotBase');
const elBtnCloseBaseDrawer = document.getElementById('btnCloseBaseDrawer');

function toggleBaseConfigDrawer() {
  if (!elBaseConfigDrawer) return;
  elBaseConfigDrawer.classList.toggle('hidden');
  const isVisible = !elBaseConfigDrawer.classList.contains('hidden');
  if (elBtnToggleBaseConfig) elBtnToggleBaseConfig.classList.toggle('active', isVisible);
}

if (elBtnToggleBaseConfig) elBtnToggleBaseConfig.addEventListener('click', toggleBaseConfigDrawer);
if (elBtnQuickRobotBase) elBtnQuickRobotBase.addEventListener('click', toggleBaseConfigDrawer);
if (elBtnCloseBaseDrawer) elBtnCloseBaseDrawer.addEventListener('click', toggleBaseConfigDrawer);

['inputBaseX', 'inputBaseY', 'inputBaseZ', 'inputBaseRoll', 'inputBasePitch', 'inputBaseYaw'].forEach(id => {
  const el = document.getElementById(id);
  if (el) {
    el.addEventListener('input', updateRobotBaseFromUI);
    el.addEventListener('change', updateRobotBaseFromUI);
  }
});

// Presets
document.getElementById('presetBaseRoof')?.addEventListener('click', () => {
  robotBaseConfig = { x: 0, y: ROOF_Y, z: 0, roll: 0, pitch: 0, yaw: 0 };
  syncRobotBaseUI();
  updateRobotBaseTransform();
  saveRobotBaseConfig();
});

document.getElementById('presetBaseFloor')?.addEventListener('click', () => {
  robotBaseConfig = { x: 0, y: 0.25, z: 0, roll: 180, pitch: 0, yaw: 0 };
  syncRobotBaseUI();
  updateRobotBaseTransform();
  saveRobotBaseConfig();
});

document.getElementById('presetBaseSide')?.addEventListener('click', () => {
  robotBaseConfig = { x: 0.5, y: 1.0, z: 0, roll: 0, pitch: 90, yaw: 0 };
  syncRobotBaseUI();
  updateRobotBaseTransform();
  saveRobotBaseConfig();
});

document.getElementById('presetBaseReset')?.addEventListener('click', () => {
  robotBaseConfig = { x: 0, y: ROOF_Y, z: 0, roll: 0, pitch: 0, yaw: 0 };
  syncRobotBaseUI();
  updateRobotBaseTransform();
  saveRobotBaseConfig();
});

// ----------------------------------------------------
// Manual Robot Control & Debug Handlers
// ----------------------------------------------------
const elManualControlDrawer = document.getElementById('manualControlDrawer');
const elBtnToggleManualControl = document.getElementById('btnToggleManualControl');
const elBtnQuickManualControl = document.getElementById('btnQuickManualControl');
const elBtnCloseManualDrawer = document.getElementById('btnCloseManualDrawer');
const elSelectControlMode = document.getElementById('selectControlMode');

const elSectionFK = document.getElementById('sectionManualFK');
const elSectionIK = document.getElementById('sectionManualIK');

function toggleManualControlDrawer() {
  if (!elManualControlDrawer) return;
  elManualControlDrawer.classList.toggle('hidden');
  const isVisible = !elManualControlDrawer.classList.contains('hidden');
  if (elBtnToggleManualControl) elBtnToggleManualControl.classList.toggle('active', isVisible);
}

if (elBtnToggleManualControl) elBtnToggleManualControl.addEventListener('click', toggleManualControlDrawer);
if (elBtnQuickManualControl) elBtnQuickManualControl.addEventListener('click', toggleManualControlDrawer);
if (elBtnCloseManualDrawer) elBtnCloseManualDrawer.addEventListener('click', toggleManualControlDrawer);

if (elSelectControlMode) {
  elSelectControlMode.addEventListener('change', (e) => {
    controlMode = e.target.value;
    updateControlModeSections();
  });
}

function updateControlModeSections() {
  if (elSectionFK) elSectionFK.classList.toggle('hidden', controlMode !== 'manual_fk');
  if (elSectionIK) elSectionIK.classList.toggle('hidden', controlMode !== 'manual_ik');
}

// Bind Sliders S1, S2, S3, S4 for Manual Servo FK Mode
[1, 2, 3, 4].forEach(num => {
  const slider = document.getElementById(`sliderS${num}`);
  const valText = document.getElementById(`valS${num}Slider`);
  if (slider) {
    const handler = (e) => {
      const val = parseFloat(e.target.value) || 90;
      manualFKState[`s${num}`] = val;
      if (valText) valText.textContent = `${val.toFixed(0)}°`;
      if (controlMode === 'manual_fk') {
        sendHardwareArmCommand(
          manualFKState.s1 !== undefined ? manualFKState.s1 : 90,
          manualFKState.s2 !== undefined ? manualFKState.s2 : 90,
          manualFKState.s3 !== undefined ? manualFKState.s3 : 90,
          manualFKState.s4 !== undefined ? manualFKState.s4 : 90
        );
      }
    };
    slider.addEventListener('input', handler);
    slider.addEventListener('change', handler);
  }
});

// Bind Robot IP input fields
['inputRobotIPHeader', 'inputRobotIPDrawer'].forEach(id => {
  const el = document.getElementById(id);
  if (el) {
    el.addEventListener('input', (e) => updateIPFromUI(e.target.value));
    el.addEventListener('change', (e) => updateIPFromUI(e.target.value));
  }
});

// Bind Hardware Sync checkboxes
['chkEnableHardwareSync', 'chkEnableHardwareSyncDrawer'].forEach(id => {
  const el = document.getElementById(id);
  if (el) {
    el.addEventListener('change', (e) => updateHardwareSyncFromUI(e.target.checked));
  }
});

// Test Send button handler
document.getElementById('btnTestHardwarePing')?.addEventListener('click', () => {
  const s1 = manualFKState.s1 !== undefined ? manualFKState.s1 : 90;
  const s2 = manualFKState.s2 !== undefined ? manualFKState.s2 : 90;
  const s3 = manualFKState.s3 !== undefined ? manualFKState.s3 : 90;
  const s4 = manualFKState.s4 !== undefined ? manualFKState.s4 : 90;
  
  // Force bypass throttling for manual test send button
  lastSendTime = 0;
  clearTimeout(pendingSendTimeout);
  sendHardwareArmCommand(s1, s2, s3, s4);
});

// Hardware Servo Angle Presets (Matching gui_server.py / gui_control.ps1)
function syncFKSlidersUI() {
  [1, 2, 3, 4].forEach(num => {
    const slider = document.getElementById(`sliderS${num}`);
    const valText = document.getElementById(`valS${num}Slider`);
    const val = manualFKState[`s${num}`] !== undefined ? manualFKState[`s${num}`] : 90;
    if (slider) slider.value = val;
    if (valText) valText.textContent = `${val.toFixed(0)}°`;
  });
}

function setServoPresets(a1, a2, a3, a4) {
  if (controlMode !== 'manual_fk') {
    controlMode = 'manual_fk';
    if (elSelectControlMode) elSelectControlMode.value = 'manual_fk';
    updateControlModeSections();
  }
  manualFKState = { s1: a1, s2: a2, s3: a3, s4: a4 };
  syncFKSlidersUI();
  sendHardwareArmCommand(a1, a2, a3, a4);
}

document.getElementById('presetMin0')?.addEventListener('click', () => setServoPresets(0, 0, 0, 0));
document.getElementById('presetNeutral90')?.addEventListener('click', () => setServoPresets(90, 90, 90, 90));
document.getElementById('presetMax180')?.addEventListener('click', () => setServoPresets(180, 180, 180, 180));

// Auto-Sweep Test
let isSweeping = false;
let sweepTimer = null;

document.getElementById('presetSweep')?.addEventListener('click', (e) => {
  const btn = e.currentTarget;
  if (isSweeping) {
    clearInterval(sweepTimer);
    isSweeping = false;
    btn.textContent = 'Sweep Test';
    btn.classList.remove('active');
  } else {
    if (controlMode !== 'manual_fk') {
      controlMode = 'manual_fk';
      if (elSelectControlMode) elSelectControlMode.value = 'manual_fk';
      updateControlModeSections();
    }
    isSweeping = true;
    btn.textContent = 'Stop Sweep';
    btn.classList.add('active');
    let sweepAngle = 0;
    let sweepDir = 5;
    sweepTimer = setInterval(() => {
      sweepAngle += sweepDir;
      if (sweepAngle >= 180) { sweepAngle = 180; sweepDir = -5; }
      if (sweepAngle <= 0) { sweepAngle = 0; sweepDir = 5; }
      setServoPresets(sweepAngle, sweepAngle, sweepAngle, sweepAngle);
    }, 30);
  }
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

  // Update 3D Grid Helpers (Floor & Ceiling)
  if (gridHelper && scene) {
    scene.remove(gridHelper);
    if (gridHelper.geometry) gridHelper.geometry.dispose();
    const gridCenterColor = isDark ? 0x06b6d4 : 0x0284c7;
    const gridLineColor = isDark ? 0x334155 : 0xcbd5e1;
    gridHelper = new THREE.GridHelper(10, 20, gridCenterColor, gridLineColor);
    gridHelper.position.y = 0;
    gridHelper.visible = typeof gridVisible !== 'undefined' ? gridVisible : true;
    scene.add(gridHelper);
  }
  if (ceilingGrid && scene) {
    scene.remove(ceilingGrid);
    if (ceilingGrid.geometry) ceilingGrid.geometry.dispose();
    const ceilingCenterColor = isDark ? 0xec4899 : 0xdb2777;
    const ceilingLineColor = isDark ? 0x334155 : 0xcbd5e1;
    ceilingGrid = new THREE.GridHelper(6, 12, ceilingCenterColor, ceilingLineColor);
    ceilingGrid.position.set(robotBaseConfig.x, robotBaseConfig.y, robotBaseConfig.z);
    const rollRad = THREE.MathUtils.degToRad(robotBaseConfig.roll);
    const pitchRad = THREE.MathUtils.degToRad(robotBaseConfig.pitch);
    const yawRad = THREE.MathUtils.degToRad(robotBaseConfig.yaw);
    ceilingGrid.rotation.copy(new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ'));
    ceilingGrid.visible = typeof gridVisible !== 'undefined' ? gridVisible : true;
    scene.add(ceilingGrid);
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

function startApp() {
  loadThemeConfig();
  loadAxisSwapConfig();
  loadRobotBaseConfig();
  loadRobotIPConfig();
  syncAxisSwapUI();
  syncRobotBaseUI();
  syncIPUI();
  initScene();
  initSensorCharts();
  applyTheme(isDarkMode);
  connectTelemetryStream();
}

if (document.readyState === 'loading') {
  window.addEventListener('DOMContentLoaded', startApp);
} else {
  startApp();
}


