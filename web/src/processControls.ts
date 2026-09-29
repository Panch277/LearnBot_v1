import ROSLIB from 'roslib';
import { rosConnection } from './rosConnection';

interface TriggerResponse {
  success: boolean;
  message: string;
}

type ProcessState = 'stopped' | 'running' | 'stopping';

const LABELS: Record<string, { start: string; stop: string }> = {
  slam: { start: 'Start SLAM', stop: 'Stop SLAM' },
  nav2: { start: 'Start Nav2', stop: 'Stop Nav2' },
};

// Wires one toggle button (e.g. <button data-process-toggle="slam">) to the
// matching process_manager.py service/state-topic pair, plus an optional
// status line (<p data-process-status="slam">) for rejection messages --
// e.g. Nav2's "Select a map first" / "Stop slam first" (see
// process_manager.py's conflicts_with).
function initProcessToggle(button: HTMLButtonElement, name: string): void {
  const labels = LABELS[name] ?? { start: `Start ${name}`, stop: `Stop ${name}` };
  const status = document.querySelector<HTMLElement>(`[data-process-status="${name}"]`);

  const toggle = new ROSLIB.Service({
    ros: rosConnection.ros,
    name: `/dashboard/${name}/toggle`,
    serviceType: 'std_srvs/srv/Trigger',
  });

  const state = new ROSLIB.Topic({
    ros: rosConnection.ros,
    name: `/dashboard/${name}/state`,
    messageType: 'std_msgs/msg/String',
  });

  let lastState: ProcessState = 'stopped';

  const render = (value: ProcessState) => {
    lastState = value;
    button.disabled = value === 'stopping';
    button.classList.toggle('is-active', value !== 'stopped');
    button.textContent = value === 'running' ? labels.stop : value === 'stopping' ? 'Stopping…' : labels.start;
  };

  render('stopped');
  state.subscribe((message) => {
    render((message as unknown as { data: ProcessState }).data);
  });

  button.addEventListener('click', () => {
    button.disabled = true;
    if (status) status.textContent = '';
    toggle.callService(new ROSLIB.ServiceRequest({}), (response) => {
      const { success, message } = response as unknown as TriggerResponse;
      if (!success) {
        // Rejected (e.g. already mid-stop, a conflicting process running,
        // or -- Nav2 -- no map selected yet) -- nothing actually changed,
        // so there's no incoming /state message to re-enable the button.
        // Re-render the last known state ourselves instead of leaving it
        // stuck disabled.
        console.warn(`[${name}] toggle rejected: ${message}`);
        if (status) {
          status.textContent = message;
          status.className = 'hint hint--err';
          // Nothing here re-checks whether the blocking condition (e.g. a
          // conflicting process) has since cleared, so don't leave a stale
          // reason on screen indefinitely -- it was only ever a snapshot of
          // why *this* click failed.
          setTimeout(() => {
            if (status.textContent === message) status.textContent = '';
          }, 5000);
        }
        render(lastState);
      }
    });
  });
}

export function initProcessControls(): void {
  const buttons = document.querySelectorAll<HTMLButtonElement>('[data-process-toggle]');
  for (const button of buttons) {
    const name = button.dataset.processToggle;
    if (name) initProcessToggle(button, name);
  }
}
