import ROSLIB from 'roslib';
import { rosConnection } from './rosConnection';
import { MAPS_DIR } from './paths';

// slam_toolbox's save-map service resolves a relative name against
// process_manager.py's cwd (wherever `ros2 launch rosbridge.launch.py`
// happened to be run from), not the repo -- MAPS_DIR must stay absolute so
// the file lands in the same place Nav2's map dropdown (nav2Controls.ts)
// looks for it.

interface SaveMapResponse {
  result: number; // slam_toolbox/srv/SaveMap: RESULT_SUCCESS = 0
}

// `/slam_toolbox/save_map` is advertised by the slam_toolbox node itself --
// the same service its own RViz panel's "Save Map" button calls -- so it
// only exists while a SLAM session (the dashboard's "Start SLAM" button) is
// actually running. Gated on the same /dashboard/slam/state topic
// processControls.ts already listens to, rather than a fresh state machine.
export function initMapSave(): void {
  const input = document.getElementById('map-save-name') as HTMLInputElement | null;
  const button = document.getElementById('map-save-btn') as HTMLButtonElement | null;
  const status = document.getElementById('map-save-status');
  if (!input || !button || !status) return;

  let slamRunning = false;
  button.disabled = true;

  const slamState = new ROSLIB.Topic({
    ros: rosConnection.ros,
    name: '/dashboard/slam/state',
    messageType: 'std_msgs/msg/String',
  });
  slamState.subscribe((message) => {
    slamRunning = (message as unknown as { data: string }).data === 'running';
    button.disabled = !slamRunning;
  });

  const saveMap = new ROSLIB.Service({
    ros: rosConnection.ros,
    name: '/slam_toolbox/save_map',
    serviceType: 'slam_toolbox/srv/SaveMap',
  });

  button.addEventListener('click', () => {
    if (!slamRunning) return;

    // Keep it to a plain filename slam_toolbox can drop straight into
    // MAPS_DIR -- no slashes/spaces that could write outside it or need
    // escaping.
    const name = input.value.trim().replace(/[^A-Za-z0-9_-]/g, '');
    if (!name) {
      status.textContent = 'Enter a map name first.';
      status.className = 'hint hint--err';
      return;
    }

    button.disabled = true;
    status.textContent = 'Saving…';
    status.className = 'hint';

    saveMap.callService(
      new ROSLIB.ServiceRequest({ name: { data: `${MAPS_DIR}/${name}` } }),
      (response) => {
        const success = (response as unknown as SaveMapResponse).result === 0;
        status.textContent = success
          ? `Saved ${name}.pgm / ${name}.yaml`
          : 'Save failed — check the slam_toolbox log.';
        status.className = success ? 'hint hint--ok' : 'hint hint--err';
        button.disabled = !slamRunning;
        // Lets nav2Controls.ts's map dropdown pick up the new file without
        // the user having to hit its own refresh button.
        if (success) window.dispatchEvent(new CustomEvent('learnbot:map-saved'));
      },
    );
  });
}
