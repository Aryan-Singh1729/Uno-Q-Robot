const socket = io(`http://${window.location.host}`);
const canvas = document.getElementById('camera');
const context = canvas.getContext('2d');
const placeholder = document.getElementById('placeholder');
const state = document.getElementById('state');
const detail = document.getElementById('detail');
const lidarCanvas = document.getElementById('lidar');
const lidarContext = lidarCanvas.getContext('2d');
const lidarState = document.getElementById('lidar-state');
const lidarDetail = document.getElementById('lidar-detail');
const frontDistance = document.getElementById('front-distance');
const nearestDistance = document.getElementById('nearest-distance');
const navEvent = document.getElementById('nav-event');
const ultrasonic = document.getElementById('ultrasonic');
const imu = document.getElementById('imu');
const follower = document.getElementById('follower');
let currentBitmap = null;

function drawLidar(message) {
  const width = lidarCanvas.width;
  const height = lidarCanvas.height;
  const centerX = width / 2;
  const centerY = height / 2;
  const radius = Math.min(width, height) * 0.44;
  const maxDisplayMm = 4000;
  const stopMm = Number(message.stop_distance_mm || 100);
  lidarContext.clearRect(0, 0, width, height);
  lidarContext.fillStyle = '#020607';
  lidarContext.fillRect(0, 0, width, height);

  lidarContext.lineWidth = 1;
  lidarContext.strokeStyle = '#19474c';
  lidarContext.fillStyle = '#6f999b';
  lidarContext.font = '12px monospace';
  [1000, 2000, 3000, 4000].forEach((distance) => {
    const ring = radius * distance / maxDisplayMm;
    lidarContext.beginPath();
    lidarContext.arc(centerX, centerY, ring, 0, Math.PI * 2);
    lidarContext.stroke();
    lidarContext.fillText(`${distance / 1000}m`, centerX + 5, centerY - ring + 14);
  });
  lidarContext.beginPath();
  lidarContext.moveTo(centerX, centerY - radius);
  lidarContext.lineTo(centerX, centerY + radius);
  lidarContext.moveTo(centerX - radius, centerY);
  lidarContext.lineTo(centerX + radius, centerY);
  lidarContext.stroke();

  const stopRadius = Math.max(6, radius * stopMm / maxDisplayMm);
  lidarContext.strokeStyle = '#ff5058';
  lidarContext.lineWidth = 3;
  lidarContext.beginPath();
  lidarContext.arc(centerX, centerY, stopRadius, 0, Math.PI * 2);
  lidarContext.stroke();

  lidarContext.fillStyle = '#4de0d9';
  const points = Array.isArray(message.points) ? message.points : [];
  points.forEach((point) => {
    const angle = Number(point[0]);
    const distance = Number(point[1]);
    if (!Number.isFinite(angle) || !Number.isFinite(distance) || distance <= 0) return;
    const plotDistance = Math.min(distance, maxDisplayMm) * radius / maxDisplayMm;
    const radians = angle * Math.PI / 180;
    const x = centerX + Math.sin(radians) * plotDistance;
    const y = centerY - Math.cos(radians) * plotDistance;
    lidarContext.fillRect(x - 2, y - 2, 4, 4);
  });

  lidarContext.fillStyle = '#fff';
  lidarContext.beginPath();
  lidarContext.arc(centerX, centerY, 6, 0, Math.PI * 2);
  lidarContext.fill();
  lidarContext.fillStyle = '#ffcb57';
  lidarContext.beginPath();
  lidarContext.moveTo(centerX, centerY - 14);
  lidarContext.lineTo(centerX - 7, centerY - 2);
  lidarContext.lineTo(centerX + 7, centerY - 2);
  lidarContext.closePath();
  lidarContext.fill();
}

socket.on('connect', () => {
  detail.textContent = 'Connected to the robot. Waiting for frames and scans.';
});

socket.on('disconnect', () => {
  state.textContent = 'offline';
  state.className = 'badge error';
  lidarState.textContent = 'offline';
  lidarState.className = 'badge error';
  detail.textContent = 'Connection to the UNO Q was lost.';
});

socket.on('camera_frame', async (message) => {
  const response = await fetch(`data:${message.image_type};base64,${message.image}`);
  const bitmap = await createImageBitmap(await response.blob());
  canvas.width = bitmap.width;
  canvas.height = bitmap.height;
  context.drawImage(bitmap, 0, 0);
  if (currentBitmap) currentBitmap.close();
  currentBitmap = bitmap;
  placeholder.hidden = true;
});

socket.on('robot_status', (message) => {
  state.textContent = message.state || 'unknown';
  state.className = message.state === 'camera_error' ? 'badge error' : 'badge';
  detail.textContent = message.detail || '';
  if ((message.state || '').startsWith('follow')) {
    follower.textContent = `Follower: ${message.state}${message.detail ? ` - ${message.detail}` : ''}`;
  }
});

socket.on('lidar_scan', (message) => {
  drawLidar(message);
  const connected = Boolean(message.connected && message.scan_fresh);
  const emergency = Boolean(message.emergency_active);
  lidarState.textContent = emergency ? 'emergency' : connected ? 'live' : 'offline';
  lidarState.className = emergency ? 'badge error' : connected ? 'badge' : 'badge warning';
  const distance = Number(message.front_distance_mm || 0);
  frontDistance.textContent = distance > 0 ? `Front: ${(distance / 10).toFixed(1)} cm` : 'Front: --';
  const nearest = Number(message.nearest_distance_mm || 0);
  const nearestAngle = Number(message.nearest_angle_deg);
  nearestDistance.textContent = nearest > 0
    ? `Nearest: ${(nearest / 10).toFixed(1)} cm @ ${Number.isFinite(nearestAngle) ? nearestAngle : '--'} deg`
    : 'Nearest: --';
  lidarDetail.textContent = message.last_error ||
    `${Array.isArray(message.points) ? message.points.length : 0} plotted points; stop at 10.0 cm`;
  const event = message.navigation_event || {};
  navEvent.textContent = `Navigation: ${event.code || 'monitoring'}${event.detail ? ` — ${event.detail}` : ''}`;
  const supplemental = message.supplemental || {};
  const sensorText = (ready, value, label) => ready && Number(value) > 0
    ? `${label}: ${(Number(value) / 10).toFixed(1)} cm`
    : `${label}: unavailable`;
  ultrasonic.textContent = sensorText(supplemental.ultrasonic_ready, supplemental.ultrasonic_mm, 'Front ultrasonic');
  const roll = Number(supplemental.roll_deg);
  const pitch = Number(supplemental.pitch_deg);
  imu.textContent = supplemental.mpu6050_ready && Number.isFinite(roll) && Number.isFinite(pitch)
    ? `Tilt: roll ${roll.toFixed(1)}°, pitch ${pitch.toFixed(1)}°`
    : 'Tilt: unavailable';
});

document.getElementById('stop').addEventListener('click', () => {
  socket.emit('emergency_stop', {});
  state.textContent = 'stopping';
  detail.textContent = 'Emergency stop sent.';
});
