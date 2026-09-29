import ROSLIB from 'roslib';
import { rosConnection } from './rosConnection';
import { MAPS_DIR } from './paths';

interface TriggerResponse {
  success: boolean;
  message: string;
}

// Lists the maps process_manager.py can see in MAPS_DIR, populates the
// dropdown, and publishes the current selection as an absolute .yaml path
// to /dashboard/nav2/map_path -- process_manager.py just remembers the
// latest value and reads it when "Start Nav2" is clicked (see the comment
// at the top of process_manager.py for why this is a topic and not a
// service argument). Publishing happens on every selection change, well
// before the user gets to clicking the toggle, so there's no ordering race
// to worry about between the two.
export function initNav2Controls(): void {
  const select = document.getElementById('nav2-map-select') as HTMLSelectElement | null;
  const refreshBtn = document.getElementById('nav2-map-refresh') as HTMLButtonElement | null;
  if (!select || !refreshBtn) return;

  const listMaps = new ROSLIB.Service({
    ros: rosConnection.ros,
    name: '/dashboard/maps/list',
    serviceType: 'std_srvs/srv/Trigger',
  });

  const mapPathTopic = new ROSLIB.Topic({
    ros: rosConnection.ros,
    name: '/dashboard/nav2/map_path',
    messageType: 'std_msgs/msg/String',
  });

  const publishSelection = () => {
    if (select.value) {
      mapPathTopic.publish(new ROSLIB.Message({ data: `${MAPS_DIR}/${select.value}.yaml` }));
    }
  };

  const refresh = () => {
    const previous = select.value;
    listMaps.callService(new ROSLIB.ServiceRequest({}), (response) => {
      const { success, message } = response as unknown as TriggerResponse;
      const names = success && message ? message.split(',') : [];

      select.innerHTML = '';
      if (names.length === 0) {
        const option = document.createElement('option');
        option.textContent = 'No saved maps';
        option.disabled = true;
        select.appendChild(option);
        select.disabled = true;
        return;
      }

      select.disabled = false;
      for (const name of names) {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = name;
        select.appendChild(option);
      }
      // Keep whatever was already selected if it's still there (e.g. after
      // a manual refresh), otherwise default to the first map.
      select.value = names.includes(previous) ? previous : names[0];
      publishSelection();
    });
  };

  select.addEventListener('change', publishSelection);
  refreshBtn.addEventListener('click', refresh);
  window.addEventListener('learnbot:map-saved', refresh);

  refresh();
}
