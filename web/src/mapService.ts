import ROSLIB from 'roslib';
import { rosConnection } from './rosConnection';
import type { RobotViewer } from './robotViewer';

interface OccupancyGridMsg {
  info: { resolution: number; width: number; height: number; origin: { position: { x: number; y: number; z: number } } };
  data: number[];
}

interface GetMapResponse {
  map: OccupancyGridMsg;
}

function applyOccupancyGrid(viewer: RobotViewer, grid: OccupancyGridMsg): void {
  const { info, data } = grid;
  viewer.setMap({
    width: info.width,
    height: info.height,
    resolution: info.resolution,
    originX: info.origin.position.x,
    originY: info.origin.position.y,
    originZ: info.origin.position.z,
    data,
  });
}

// map_server publishes /map exactly once with transient-local (latched)
// durability -- roslib.js's Topic has no way to request that QoS, so a
// subscription started any time after map_server's one-shot publish (which,
// realistically, is almost always -- nav2 is normally already running by
// the time this dashboard connects) would simply never receive it. Calling
// the /map_server/map service instead sidesteps durability entirely: it's a
// fresh request/response, not a subscription waiting on a past publish.
const RETRY_DELAY_MS = 3000;

// nav2 isn't necessarily running yet when this dashboard loads -- it's a
// separate launch step, and it's entirely normal to start the dashboard
// first. The service just won't exist until nav2's map_server comes up, so
// keep retrying rather than failing once and leaving the map panel
// permanently blank; it's a cheap call with nothing else waiting on it.
export function fetchMap(viewer: RobotViewer): void {
  const getMap = new ROSLIB.Service({
    ros: rosConnection.ros,
    name: '/map_server/map',
    serviceType: 'nav_msgs/srv/GetMap',
  });

  const attempt = () => {
    getMap.callService(
      new ROSLIB.ServiceRequest({}),
      (response) => applyOccupancyGrid(viewer, (response as unknown as GetMapResponse).map),
      () => setTimeout(attempt, RETRY_DELAY_MS),
    );
  };

  attempt();
}

// slam_toolbox republishes /map periodically while mapping (every
// map_update_interval, see config/mapper_params_online_async.yaml) -- unlike
// map_server's one-shot latched publish above, a plain topic subscription
// is enough to catch those updates, so this is what makes the map grow live
// in the 3D view while SLAM runs. Left as a separate subscription rather
// than replacing fetchMap(): map_server's single publish can still be
// missed by a subscription started after it (the exact problem fetchMap's
// GetMap-service retry works around), so both stay wired up in main.ts.
export function subscribeMapUpdates(viewer: RobotViewer): void {
  const mapTopic = new ROSLIB.Topic({
    ros: rosConnection.ros,
    name: '/map',
    messageType: 'nav_msgs/msg/OccupancyGrid',
  });

  mapTopic.subscribe((message) => applyOccupancyGrid(viewer, message as unknown as OccupancyGridMsg));
}
