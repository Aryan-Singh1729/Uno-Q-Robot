const socket = io(`http://${window.location.host}`);
const canvas = document.getElementById('camera');
const context = canvas.getContext('2d');
const placeholder = document.getElementById('placeholder');
const state = document.getElementById('state');
const detail = document.getElementById('detail');
let currentBitmap = null;

socket.on('connect', () => {
  detail.textContent = 'Connected to the robot. Waiting for frames.';
});

socket.on('disconnect', () => {
  state.textContent = 'offline';
  state.className = 'badge error';
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
});

document.getElementById('stop').addEventListener('click', () => {
  socket.emit('emergency_stop', {});
  state.textContent = 'stopping';
  detail.textContent = 'Emergency stop sent.';
});
