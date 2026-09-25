import { rosConnection, type ConnectionState } from './rosConnection';
import { initTeleop } from './teleop';
import { RobotViewer } from './robotViewer';
import { subscribeRobotState } from './jointStateSubscriber';
import { subscribeLidar } from './lidarSubscriber';
import { fetchMap } from './mapService';
import { subscribeMapFrame } from './mapFrameSubscriber';
import { publishInitialPose, publishGoalPose } from './navControls';

// web_video_server wants the *base* raw image topic (sensor_msgs/msg/Image) --
// it does its own re-encoding to MJPEG internally. Pointing it at the
// already-compressed /camera/image_raw/compressed topic causes a silent
// type mismatch (CompressedImage vs Image) and zero frames delivered.
const CAMERA_STREAM_URL =
  `http://${location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg&quality=50`;

function initConnectionBadge(): void {
  const badge = document.getElementById('connection-badge');
  if (!badge) return;

  const labels: Record<ConnectionState, string> = {
    connecting: 'connecting…',
    connected: 'connected',
    error: 'error',
    closed: 'disconnected',
  };

  rosConnection.onStateChange((state) => {
    badge.textContent = labels[state];
    badge.className = `badge badge--${state}`;
  });
}

function initCameraFeed(): void {
  const img = document.getElementById('camera-feed') as HTMLImageElement | null;
  if (!img) return;

  const connect = () => {
    img.src = `${CAMERA_STREAM_URL}&_=${Date.now()}`;
  };

  img.addEventListener('error', () => {
    setTimeout(connect, 2000);
  });

  connect();
}

function initRobotViewer(): void {
  const container = document.getElementById('robot-viewer');
  if (!container) return;

  const viewer = new RobotViewer(container);
  viewer
    .load()
    .then(() => {
      subscribeRobotState(viewer);
      subscribeLidar(viewer);
      subscribeMapFrame(viewer);
      fetchMap(viewer);
    })
    .catch((err) => console.error('Failed to load robot URDF', err));

  type ViewMode = 'orbit' | 'pan' | 'pose' | 'goal';
  let mode: ViewMode = 'orbit';

  const modeButtons = document.querySelectorAll<HTMLButtonElement>('[data-view-mode]');
  for (const button of modeButtons) {
    button.addEventListener('click', () => {
      mode = button.dataset.viewMode as ViewMode;
      viewer.setInteractionMode(mode);
      for (const other of modeButtons) other.classList.toggle('is-active', other === button);
    });
  }

  // RViz-style "2D Pose Estimate": press to plant the position, drag to aim
  // the heading, release to publish -- rather than a single click, which
  // has no direction to derive a heading from and so always meant "face
  // +X". Pointer events (not mouse/touch separately) so this works the
  // same with a mouse and with a touchscreen; domElement already sets
  // touch-action: none for exactly this kind of custom gesture handling.
  // mousemove/mouseup listen on window, not domElement, so a drag that
  // strays outside the canvas (or releases there) still ends cleanly
  // instead of leaving the drag "stuck".
  const MIN_HEADING_DRAG_METERS = 0.05;
  let poseDragStart: { x: number; y: number } | null = null;

  viewer.domElement.addEventListener('pointerdown', (event) => {
    if (mode !== 'pose' && mode !== 'goal') return;
    const point = viewer.pickMapPoint(event.clientX, event.clientY);
    if (!point) return;
    poseDragStart = point;
    viewer.showPoseArrow(point, point);
  });

  window.addEventListener('pointermove', (event) => {
    if (!poseDragStart) return;
    const point = viewer.pickMapPoint(event.clientX, event.clientY);
    if (point) viewer.showPoseArrow(poseDragStart, point);
  });

  window.addEventListener('pointerup', (event) => {
    if (!poseDragStart) return;
    const start = poseDragStart;
    poseDragStart = null;
    viewer.hidePoseArrow();

    const end = viewer.pickMapPoint(event.clientX, event.clientY) ?? start;
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const yaw = Math.hypot(dx, dy) > MIN_HEADING_DRAG_METERS ? Math.atan2(dy, dx) : 0;
    if (mode === 'pose') publishInitialPose(start.x, start.y, yaw);
    else publishGoalPose(start.x, start.y, yaw);
  });
}

initConnectionBadge();
initCameraFeed();
initTeleop();
initRobotViewer();
