import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';

const URDF_URL = '/robot/robot.urdf';

export class RobotViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;

  // Everything ROS-related (robot, lidar points, grid) lives under this
  // group. ROS is Z-up, right-handed; three.js is Y-up. Rather than
  // remembering to convert every position/quaternion we work with, we
  // apply the Z-up -> Y-up correction exactly once here, at the edge, and
  // keep all our own math in native ROS coordinates everywhere else.
  // (Same class of bug we hit -- and fixed via "Mesh up-axis" -- in
  // Foxglove earlier; STL has no up-axis metadata so nothing auto-corrects
  // it for us here either.)
  private readonly rosGroup = new THREE.Group();
  private readonly grid: THREE.GridHelper;
  private readonly raycaster = new THREE.Raycaster();

  // rosGroup itself is treated as the fixed "map" frame: the occupancy-grid
  // plane is a direct, static child of it (positioned once from the map's
  // own origin and never moved again). The robot -- and everything attached
  // to it, lidar and TF axes included -- instead lives under this odomFrame
  // group, whose pose is kept in sync with the live map->odom correction
  // AMCL publishes. That's what makes the robot visibly move to where you
  // click when setting the initial pose or driving around, with the map
  // staying put underneath it -- the same way RViz/Foxglove render it with
  // "map" as the fixed frame. (An earlier version had this backwards: the
  // map moved to align with a robot rendered fixed at its odom-relative
  // position, which put the map in the right place but left the robot
  // looking like it never responded to localization at all.)
  private readonly odomFrame = new THREE.Group();
  private mapMesh: THREE.Mesh | undefined;

  private robot: URDFRobot | undefined;
  private lidarPoints: THREE.Points | undefined;
  private poseArrow: THREE.ArrowHelper | undefined;

  constructor(container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);

    this.scene.background = new THREE.Color(0x0b0f14);
    this.rosGroup.rotateX(-Math.PI / 2);
    this.scene.add(this.rosGroup);

    // GridHelper is authored for a plain Y-up three.js world -- it lies flat
    // in its own local XZ plane (y=0), which IS already the correct floor
    // plane once rendered. Adding it as a child of rosGroup (like everything
    // ROS-related) would additionally put it through the Z-up -> Y-up
    // correction meant for ROS-frame content, tipping it into a near-vertical
    // plane instead of a floor -- so, unlike the robot, it goes directly on
    // the scene, at the world's native ground plane.
    //
    // The robot chassis is ~20-30cm across, so a default multi-meter grid
    // would dwarf it the same way the default Foxglove grid did -- keep it
    // small and fine-grained.
    this.grid = new THREE.GridHelper(2, 20, 0x2a3a4a, 0x1a242e);
    this.scene.add(this.grid);

    this.rosGroup.add(this.odomFrame);

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const keyLight = new THREE.DirectionalLight(0xffffff, 0.8);
    keyLight.position.set(1, 1, 2);
    this.scene.add(keyLight);

    // Camera lives outside rosGroup, so it sees the already Y-up-corrected
    // world -- plain three.js Y-up conventions apply here, not ROS Z-up.
    const { clientWidth: width, clientHeight: height } = container;
    this.camera = new THREE.PerspectiveCamera(50, width / height, 0.01, 100);
    this.camera.position.set(0.6, 0.45, 0.6);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.05, 0);
    this.controls.update();

    this.resize(width, height);
    new ResizeObserver(() => this.resize(container.clientWidth, container.clientHeight)).observe(container);

    this.animate();
  }

  async load(): Promise<void> {
    // URDFLoader.loadAsync resolves once the link/joint hierarchy is built,
    // *not* once every mesh's geometry has actually loaded -- STL fetching
    // happens via async callbacks that attach later. A LoadingManager set up
    // before loading starts (registering onLoad now, not after -- otherwise
    // it may have already fired for these small local files by the time we
    // attach it) lets us wait for the real thing: all meshes in place.
    const manager = new THREE.LoadingManager();
    const meshesLoaded = new Promise<void>((resolve) => {
      manager.onLoad = () => resolve();
    });

    const loader = new URDFLoader(manager);
    loader.packages = { learnbot: '/robot' };
    this.robot = await loader.loadAsync(URDF_URL);
    this.robot.rotation.set(0, 0, 0);
    this.odomFrame.add(this.robot);

    await meshesLoaded;

    this.addTfAxes();
    this.alignGridToWheels();
    this.attachLidar();
  }

  // /diff_cont/odom (like most diff-drive odometry) always reports z=0 --
  // it's a flat-ground planar pose, not the real height of base_link above
  // the ground. But base_link's own origin (from the CAD export) sits well
  // above true wheel-ground-contact height, offset down through several
  // fixed joints to the wheels. So parking the grid at ROS z=0 puts it at
  // base_link's height, not the ground -- well above the wheels.
  //
  // Rather than hand-deriving that offset from the xacro's joint origins
  // (fragile -- breaks silently if the CAD model changes), measure it
  // directly off the loaded geometry: the robot's lowest point at rest,
  // before any odom translation is applied, *is* the ground. Do this once
  // at load time -- pure yaw motion afterwards doesn't change that offset.
  private alignGridToWheels(): void {
    if (!this.robot) return;
    const bounds = new THREE.Box3().setFromObject(this.robot);
    this.grid.position.y = bounds.min.y;
  }

  // Small RGB (X/Y/Z) axis triads for the frames most useful to sanity-check
  // against `ros2 topic echo`/tf2_echo: "odom" (as located within the fixed
  // map frame, via the live map->odom correction), the robot's own
  // base_link, and both drive wheels. Each is parented under the object
  // that already represents that frame's live transform (odomFrame itself
  // for "odom", the robot root for base_link, the named URDF links for the
  // wheels), so they track pose/joint updates automatically with no extra
  // subscription code -- and being descendants of rosGroup, they inherit
  // the same Z-up -> Y-up correction as everything else ROS.
  private addTfAxes(): void {
    const odomAxes = new THREE.AxesHelper(0.15);
    this.odomFrame.add(odomAxes);

    if (!this.robot) return;

    const baseLinkAxes = new THREE.AxesHelper(0.08);
    this.robot.add(baseLinkAxes);

    for (const linkName of ['left_wheel', 'right_wheel']) {
      const link = this.robot.links[linkName];
      if (!link) {
        console.warn(`RobotViewer: link "${linkName}" not found in URDF, skipping its TF axes`);
        continue;
      }
      link.add(new THREE.AxesHelper(0.05));
    }
  }

  // A THREE.Points cloud, parented directly under the "laser_frame" URDF
  // link (like the wheel TF axes) so it inherits that link's live transform
  // for free -- /scan's ranges are already in the lidar's own local frame,
  // so no extra pose math is needed here beyond the polar->Cartesian
  // conversion done per-point in setLidarScan.
  private attachLidar(): void {
    if (!this.robot) return;
    const link = this.robot.links['laser_frame'];
    if (!link) {
      console.warn('RobotViewer: link "laser_frame" not found in URDF, skipping lidar overlay');
      return;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
    const material = new THREE.PointsMaterial({ color: 0xff3d5a, size: 0.015, sizeAttenuation: true });

    this.lidarPoints = new THREE.Points(geometry, material);
    link.add(this.lidarPoints);
  }

  // Converts one LaserScan to Cartesian points in the lidar's own frame and
  // rewrites them into the existing GPU buffer in place; the underlying
  // Float32Array is only reallocated when a scan needs more capacity than
  // it currently has (the ray count is normally constant per sensor), not
  // on every incoming message.
  setLidarScan(scan: {
    ranges: number[];
    angleMin: number;
    angleIncrement: number;
    rangeMin: number;
    rangeMax: number;
  }): void {
    if (!this.lidarPoints) return;

    const attribute = this.lidarPoints.geometry.getAttribute('position') as THREE.BufferAttribute;
    const positions =
      attribute.array.length >= scan.ranges.length * 3
        ? (attribute.array as Float32Array)
        : new Float32Array(scan.ranges.length * 3);

    let count = 0;
    for (let i = 0; i < scan.ranges.length; i++) {
      const range = scan.ranges[i];
      if (!Number.isFinite(range) || range < scan.rangeMin || range > scan.rangeMax) continue;

      const angle = scan.angleMin + i * scan.angleIncrement;
      positions[count * 3] = range * Math.cos(angle);
      positions[count * 3 + 1] = range * Math.sin(angle);
      positions[count * 3 + 2] = 0;
      count++;
    }

    if (positions !== attribute.array) {
      this.lidarPoints.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    } else {
      attribute.needsUpdate = true;
    }
    this.lidarPoints.geometry.setDrawRange(0, count);
  }

  // Renders a nav_msgs/OccupancyGrid as a textured plane. Values are drawn
  // straight to a canvas (white=free, black=occupied, gray=unknown) rather
  // than pulled in as an image file -- the grid arrives as raw probability
  // data, not a picture.
  setMap(map: {
    width: number;
    height: number;
    resolution: number;
    originX: number;
    originY: number;
    originZ: number;
    data: ArrayLike<number>;
  }): void {
    if (this.mapMesh) {
      this.rosGroup.remove(this.mapMesh);
      this.mapMesh.geometry.dispose();
      (this.mapMesh.material as THREE.Material).dispose();
    }

    const canvas = document.createElement('canvas');
    canvas.width = map.width;
    canvas.height = map.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const image = ctx.createImageData(map.width, map.height);
    for (let row = 0; row < map.height; row++) {
      // OccupancyGrid row 0 is the bottom of the map (ROS Y increases
      // upward); canvas row 0 is the top -- flip vertically so the texture
      // isn't mirrored.
      const destRow = map.height - 1 - row;
      for (let col = 0; col < map.width; col++) {
        const value = map.data[row * map.width + col];
        const gray = value < 0 ? 128 : 255 - Math.round((value / 100) * 255);
        const i = (destRow * map.width + col) * 4;
        image.data[i] = gray;
        image.data[i + 1] = gray;
        image.data[i + 2] = gray;
        image.data[i + 3] = 255;
      }
    }
    ctx.putImageData(image, 0, 0);

    const texture = new THREE.CanvasTexture(canvas);
    texture.magFilter = THREE.NearestFilter;
    texture.minFilter = THREE.NearestFilter;

    const widthMeters = map.width * map.resolution;
    const heightMeters = map.height * map.resolution;
    // PlaneGeometry's default orientation (spans local X/Y, normal along Z)
    // already matches the ROS ground-plane convention, so -- unlike
    // GridHelper -- it needs no extra rotation to sit flat once rendered.
    const geometry = new THREE.PlaneGeometry(widthMeters, heightMeters);
    const material = new THREE.MeshBasicMaterial({ map: texture, transparent: true, opacity: 0.85 });
    this.mapMesh = new THREE.Mesh(geometry, material);

    // OccupancyGrid.info.origin is the pose of cell (0,0) -- the plane's
    // corner -- but PlaneGeometry is centered on its own origin, so shift by
    // half the extents to line the two up.
    this.mapMesh.position.set(
      map.originX + widthMeters / 2,
      map.originY + heightMeters / 2,
      map.originZ,
    );

    this.rosGroup.add(this.mapMesh);
  }

  // AMCL publishes map->odom on /tf as it localizes/corrects; feed that into
  // odomFrame so the robot (rendered relative to odom) moves to its correct
  // place within the fixed map, rather than moving the map underneath a
  // fixed robot.
  setMapOdomTransform(position: THREE.Vector3Like, quaternion: THREE.QuaternionLike): void {
    this.odomFrame.position.set(position.x, position.y, position.z);
    this.odomFrame.quaternion.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  }

  // Converts a mouse/touch screen position into a point on the ground plane,
  // expressed in "map frame" coordinates -- what /initialpose and
  // /goal_pose need. Raycasts against a math plane at the same height as
  // the grid/wheels rather than requiring a hit on the map mesh itself, so
  // clicks still work at the map's edges or before it has loaded. rosGroup
  // *is* the map frame here (just Z-up -> Y-up rotated, no translation), so
  // converting into its local space is exactly the map-frame coordinates.
  pickMapPoint(clientX: number, clientY: number): { x: number; y: number } | null {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(ndc, this.camera);

    const groundY = this.grid.position.y;
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -groundY);
    const hit = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(plane, hit)) return null;

    const local = this.rosGroup.worldToLocal(hit.clone());
    return { x: local.x, y: local.y };
  }

  get domElement(): HTMLCanvasElement {
    return this.renderer.domElement;
  }

  // Live preview of the heading being dragged out for /initialpose or
  // /goal_pose, in the same map-frame (x, y) coordinates pickMapPoint
  // returns -- rosGroup is the map frame, so these need no extra
  // conversion, same as mapMesh's position above. Reused across a single
  // drag (position/direction/length updated in place) rather than
  // recreated per mousemove.
  showPoseArrow(from: { x: number; y: number }, to: { x: number; y: number }): void {
    const dir = new THREE.Vector3(to.x - from.x, to.y - from.y, 0);
    const length = dir.length();
    if (length < 1e-6) dir.set(1, 0, 0);
    else dir.normalize();

    if (!this.poseArrow) {
      this.poseArrow = new THREE.ArrowHelper(dir, new THREE.Vector3(from.x, from.y, 0), 0.05, 0x33c1ff, 0.06, 0.04);
      this.rosGroup.add(this.poseArrow);
    } else {
      this.poseArrow.position.set(from.x, from.y, 0);
      this.poseArrow.setDirection(dir);
    }
    this.poseArrow.setLength(Math.max(length, 0.05), 0.06, 0.04);
  }

  hidePoseArrow(): void {
    if (!this.poseArrow) return;
    this.rosGroup.remove(this.poseArrow);
    (this.poseArrow.line.material as THREE.Material).dispose();
    (this.poseArrow.cone.material as THREE.Material).dispose();
    this.poseArrow = undefined;
  }

  // Default left-drag is orbit-rotate. "move" swaps it to pan instead
  // (touch's one-finger drag switches the same way), since rotate is the
  // wrong tool for just repositioning the view -- especially on a phone,
  // where there's no separate right-click/two-finger gesture to reach for
  // it. "pose"/"goal" disable left-drag camera control entirely (an
  // unmatched THREE.MOUSE value is a safe no-op for OrbitControls) so a
  // click there is unambiguously "place a pose", not "also nudge the
  // camera".
  setInteractionMode(mode: 'orbit' | 'pan' | 'pose' | 'goal'): void {
    const left = mode === 'orbit' ? THREE.MOUSE.ROTATE : mode === 'pan' ? THREE.MOUSE.PAN : null;
    this.controls.mouseButtons.LEFT = left;
    this.controls.touches.ONE = mode === 'pan' ? THREE.TOUCH.PAN : THREE.TOUCH.ROTATE;
  }

  setJointValue(name: string, value: number): void {
    this.robot?.setJointValue(name, value);
  }

  setBasePose(position: THREE.Vector3Like, quaternion: THREE.QuaternionLike): void {
    if (!this.robot) return;
    this.robot.position.set(position.x, position.y, position.z);
    this.robot.quaternion.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  }

  private resize(width: number, height: number): void {
    if (width === 0 || height === 0) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };
}
