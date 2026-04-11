const canvas = document.querySelector("#routeCanvas");
const ctx = canvas.getContext("2d");
const state = {
    points: [{x: 0, y: 0}],
    obstacles: [],
    current: {x: 0, y: 0, heading: 0, status: "idle"},
    view: {
        scale: 5,
        offsetX: 0,
        offsetY: 0,
        dragging: false,
        pointerId: null,
        lastPointerX: 0,
        lastPointerY: 0,
    },
};

const els = {
    status: document.querySelector("#status"),
    x: document.querySelector("#x"),
    y: document.querySelector("#y"),
    heading: document.querySelector("#heading"),
    obstacle: document.querySelector("#obstacle"),
    message: document.querySelector("#message"),
    stepDistance: document.querySelector("#stepDistance"),
};

const CARDINAL_ANGLES = [0, 90, 180, 270];

function routePayload(kind) {
    if (kind === "square") {
        return {
            segments: [
                {action: "forward", distance_cm: 30},
                {action: "turn_right", angle_deg: 90},
                {action: "forward", distance_cm: 30},
                {action: "turn_right", angle_deg: 90},
                {action: "forward", distance_cm: 30},
                {action: "turn_right", angle_deg: 90},
                {action: "forward", distance_cm: 30},
            ],
        };
    }
    return {segments: [{action: "forward", distance_cm: 100}]};
}

function normalizeAngle(angle) {
    return ((angle % 360) + 360) % 360;
}

function serverHeadingToDisplay(serverHeading) {
    return normalizeAngle(360 - serverHeading);
}

function displayHeadingToServer(displayHeading) {
    return normalizeAngle(360 - displayHeading);
}

function roundToCardinal(angle) {
    const normalized = normalizeAngle(angle);
    return CARDINAL_ANGLES.reduce((closest, candidate) => {
        const currentDistance = Math.abs(normalized - closest);
        const candidateDistance = Math.abs(normalized - candidate);
        return candidateDistance < currentDistance ? candidate : closest;
    }, CARDINAL_ANGLES[0]);
}

function buildTurnSegments(targetHeading) {
    const currentHeading = roundToCardinal(state.current.rawHeading);
    const desiredHeading = normalizeAngle(targetHeading);
    const rightDelta = normalizeAngle(desiredHeading - currentHeading);
    const leftDelta = normalizeAngle(currentHeading - desiredHeading);

    if (rightDelta === 0 || leftDelta === 0) {
        return [];
    }
    if (rightDelta <= leftDelta) {
        return [{action: "turn_right", angle_deg: rightDelta}];
    }
    return [{action: "turn_left", angle_deg: leftDelta}];
}

function movementPayload(direction) {
    const distance = Math.max(1, Number(els.stepDistance.value) || 0);
    if (direction === "forward") {
        return {segments: [{action: "forward", distance_cm: distance}]};
    }
    if (direction === "backward") {
        return {segments: [{action: "backward", distance_cm: distance}]};
    }

    const targetDisplayHeading = direction === "right" ? 90 : 270;
    const targetHeading = displayHeadingToServer(targetDisplayHeading);
    return {
        segments: [
            ...buildTurnSegments(targetHeading),
            {action: "forward", distance_cm: distance},
        ],
    };
}

async function sendRoute(kind) {
    state.points = [{x: 0, y: 0}];
    state.obstacles = [];
    draw();
    await fetch("/v1/drive/route", {
        method: "POST",
        headers: {"content-type": "application/json"},
        body: JSON.stringify(routePayload(kind)),
    });
}

async function sendMovement(direction) {
    await fetch("/v1/drive/route", {
        method: "POST",
        headers: {"content-type": "application/json"},
        body: JSON.stringify(movementPayload(direction)),
    });
}

async function stopRoute() {
    await fetch("/v1/drive/stop", {method: "POST"});
}

function connect() {
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${protocol}://${location.host}/v1/motion/ws`);

    ws.addEventListener("message", (event) => {
        update(JSON.parse(event.data));
    });
    ws.addEventListener("close", () => {
        els.status.textContent = "нет связи";
        els.message.textContent = "Переподключение...";
        setTimeout(connect, 1000);
    });
}

function update(event) {
    const displayX = -event.x_cm;
    const displayHeading = serverHeadingToDisplay(event.heading_deg);
    const displayObstacleX =
        event.obstacle_x_cm === null || event.obstacle_x_cm === undefined
            ? null
            : -event.obstacle_x_cm;

    state.current = {
        x: displayX,
        y: event.y_cm,
        heading: displayHeading,
        rawHeading: event.heading_deg,
        status: event.status,
    };

    if (event.type === "position") {
        const last = state.points[state.points.length - 1];
        if (!last || Math.hypot(last.x - displayX, last.y - event.y_cm) >= 0.25) {
            state.points.push({x: displayX, y: event.y_cm});
        }
    }
    if (event.type === "obstacle" && displayObstacleX !== null && event.obstacle_y_cm !== null) {
        addObstacle(displayObstacleX, event.obstacle_y_cm);
    }

    els.status.textContent = statusLabel(event.status);
    els.x.textContent = `${displayX.toFixed(1)} см`;
    els.y.textContent = `${event.y_cm.toFixed(1)} см`;
    els.heading.textContent = `${displayHeading.toFixed(0)}°`;
    els.obstacle.textContent =
        event.obstacle_cm === null || event.obstacle_cm === undefined
            ? "нет данных"
            : `${event.obstacle_cm.toFixed(1)} см`;
    els.message.textContent = event.message || "События приходят";

    draw();
}

function resetView() {
    state.view.offsetX = 0;
    state.view.offsetY = 0;
    draw();
}

function addObstacle(x, y) {
    const exists = state.obstacles.some((obstacle) => Math.hypot(obstacle.x - x, obstacle.y - y) < 2);
    if (!exists) {
        state.obstacles.push({x, y});
    }
}

function statusLabel(status) {
    return {
        idle: "ожидание",
        moving: "движение",
        turning: "поворот",
        blocked: "препятствие",
        error: "ошибка",
        stopped: "остановлено",
    }[status] || status;
}

function draw() {
    const width = canvas.width;
    const height = canvas.height;
    const {scale, offsetX, offsetY} = state.view;
    const originX = width / 2 + offsetX;
    const originY = height / 2 + offsetY;

    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, width, height);

    ctx.strokeStyle = "#e1e7ee";
    ctx.lineWidth = 1;
    for (let i = -500; i <= 500; i += 10) {
        ctx.beginPath();
        ctx.moveTo(originX + i * scale, originY - 500 * scale);
        ctx.lineTo(originX + i * scale, originY + 500 * scale);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(originX - 500 * scale, originY + i * scale);
        ctx.lineTo(originX + 500 * scale, originY + i * scale);
        ctx.stroke();
    }

    ctx.strokeStyle = "#c43d4b";
    ctx.strokeRect(originX - 500 * scale, originY - 500 * scale, 80 * scale, 80 * scale);

    ctx.fillStyle = "#111827";
    state.obstacles.forEach((obstacle) => {
        const size = 10;
        const x = originX + obstacle.x * scale - size / 2;
        const y = originY - obstacle.y * scale - size / 2;
        ctx.fillRect(x, y, size, size);
    });

    ctx.strokeStyle = "#1f7a5c";
    ctx.lineWidth = 4;
    ctx.beginPath();
    state.points.forEach((point, index) => {
        const x = originX + point.x * scale;
        const y = originY - point.y * scale;
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    });
    ctx.stroke();

    const carX = originX + state.current.x * scale;
    const carY = originY - state.current.y * scale;
    const angle = (state.current.heading - 90) * (Math.PI / 180);
    ctx.save();
    ctx.translate(carX, carY);
    ctx.rotate(angle);
    ctx.fillStyle = state.current.status === "error" ? "#c43d4b" : "#246b8f";
    ctx.beginPath();
    ctx.moveTo(14, 0);
    ctx.lineTo(-10, -8);
    ctx.lineTo(-10, 8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
}

function startDrag(event) {
    state.view.dragging = true;
    state.view.pointerId = event.pointerId;
    state.view.lastPointerX = event.clientX;
    state.view.lastPointerY = event.clientY;
    canvas.classList.add("dragging");
    canvas.setPointerCapture(event.pointerId);
}

function drag(event) {
    if (!state.view.dragging || state.view.pointerId !== event.pointerId) {
        return;
    }

    const deltaX = event.clientX - state.view.lastPointerX;
    const deltaY = event.clientY - state.view.lastPointerY;
    state.view.offsetX += deltaX;
    state.view.offsetY += deltaY;
    state.view.lastPointerX = event.clientX;
    state.view.lastPointerY = event.clientY;
    draw();
}

function stopDrag(event) {
    if (state.view.pointerId !== null && event.pointerId !== state.view.pointerId) {
        return;
    }

    state.view.dragging = false;
    state.view.pointerId = null;
    canvas.classList.remove("dragging");
}

document.querySelector("#startSquare").addEventListener("click", () => sendRoute("square"));
document.querySelector("#startLine").addEventListener("click", () => sendRoute("line"));
document.querySelector("#resetView").addEventListener("click", resetView);
document.querySelector("#stop").addEventListener("click", stopRoute);
document.querySelector("#moveForward").addEventListener("click", () => sendMovement("forward"));
document.querySelector("#moveBackward").addEventListener("click", () => sendMovement("backward"));
document.querySelector("#moveLeft").addEventListener("click", () => sendMovement("left"));
document.querySelector("#moveRight").addEventListener("click", () => sendMovement("right"));
canvas.addEventListener("pointerdown", startDrag);
canvas.addEventListener("pointermove", drag);
canvas.addEventListener("pointerup", stopDrag);
canvas.addEventListener("pointercancel", stopDrag);
canvas.addEventListener("pointerleave", stopDrag);

draw();
connect();
