// joystick.js
let canvas, ctx;
let isDragging = false;

function initJoystick(socket) {
  canvas = document.getElementById("joystick");
  ctx = canvas.getContext("2d");

  function resizeCanvas() {
    const image = document.getElementById("camera-feed");
    canvas.width = image.clientWidth;
    canvas.height = image.clientHeight;
    drawOverlay();
  }

  function sendSV(left, right) {
    const cmd = `&SV ${Math.round(left)} ${Math.round(right)}`;
    socket.emit("user-command", cmd);
  }

  function processJoystickInput(e) {
    const rect = canvas.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;

    let clientX, clientY;
    if (e.touches) {
      clientX = e.touches[0].clientX;
      clientY = e.touches[0].clientY;
    } else {
      clientX = e.clientX;
      clientY = e.clientY;
    }

    const relX = clientX - rect.left;
    const relY = clientY - rect.top;

    let x = relX - centerX;
    let y = centerY - relY;

    const maxRadius = centerY * 0.8;
    const dist = Math.sqrt(x * x + y * y);
    if (dist > maxRadius) {
      const scale = maxRadius / dist;
      x *= scale;
      y *= scale;
    }

    const forward = (y / maxRadius) * 200;
    const turn = (x / maxRadius) * 100;   // Attempt to make steering easier 250708

    const left = forward + turn;
    const right = forward - turn;

    sendSV(left, right);
  }

  function drawOverlay() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 5;

    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const maxRadius = canvas.height * 0.8 / 2;

    [0.2, 0.4, 0.6, 0.8].forEach(r => {
      ctx.beginPath();
      ctx.arc(centerX, centerY, maxRadius * r, 0, 2 * Math.PI);
      ctx.stroke();
    });

    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.fill();
  }

  // Event listeners
  canvas.addEventListener("mousedown", (e) => {
    isDragging = true;
    processJoystickInput(e);
  });

  canvas.addEventListener("mousemove", (e) => {
    if (isDragging) processJoystickInput(e);
  });

  canvas.addEventListener("mouseup", () => {
    isDragging = false;
    sendSV(0, 0);
  });

  canvas.addEventListener("touchstart", (e) => {
    e.preventDefault();
    isDragging = true;
    processJoystickInput(e);
  });

  canvas.addEventListener("touchmove", (e) => {
    e.preventDefault();
    if (isDragging) processJoystickInput(e);
  });

  canvas.addEventListener("touchend", (e) => {
    e.preventDefault();
    isDragging = false;
    sendSV(0, 0);
  });

  window.addEventListener("resize", resizeCanvas);
  resizeCanvas();
}
