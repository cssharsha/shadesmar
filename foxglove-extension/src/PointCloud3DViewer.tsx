import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type Config = {
  pointCloudTopic: string;
  cameraPoseTopic: string;
  cameraInfoTopic: string;
  keyframePosesTopic: string;
  publishRate: number;
};

type PoseStamped = {
  header: {
    stamp: { sec: number; nsec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
};

type CameraInfo = {
  header: {
    stamp: { sec: number; nsec: number };
    frame_id: string;
  };
  height: number;
  width: number;
  distortion_model: string;
  D: number[];
  K: number[];
  R: number[];
  P: number[];
};

function PointCloud3DViewer({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config] = useState<Config>(() => {
    const partialConfig = context.initialState as Partial<Config>;
    return {
      pointCloudTopic: partialConfig.pointCloudTopic ?? "/gaussian_splats/pointcloud",
      cameraPoseTopic: partialConfig.cameraPoseTopic ?? "/viewer/camera_pose",
      cameraInfoTopic: partialConfig.cameraInfoTopic ?? "/viewer/camera_info",
      keyframePosesTopic: partialConfig.keyframePosesTopic ?? "/gaussian_splats/keyframe_poses",
      publishRate: partialConfig.publishRate ?? 10, // Hz
    };
  });

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [pointCount, setPointCount] = useState(0);
  const [messageCount, setMessageCount] = useState(0);
  const [debugLogs, setDebugLogs] = useState<string[]>([]);
  const [showDebug, setShowDebug] = useState(true);
  const [currentPose, setCurrentPose] = useState<{ position: THREE.Vector3; quaternion: THREE.Quaternion } | null>(null);
  const [showCameraHelper, setShowCameraHelper] = useState(true);
  const [keyframePoses, setKeyframePoses] = useState<Array<{ position: THREE.Vector3; quaternion: THREE.Quaternion }>>([]);

  const addDebugLog = (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setDebugLogs((prev) => [...prev.slice(-20), `[${timestamp}] ${message}`]); // Keep last 20 logs
  };

  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const pointCloudRef = useRef<THREE.Points | null>(null);
  const advertisePoseIdRef = useRef<string | undefined | void>();
  const advertiseCameraInfoIdRef = useRef<string | undefined | void>();
  const helperCameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const cameraHelperRef = useRef<THREE.CameraHelper | null>(null);
  const cameraMarkerRef = useRef<THREE.Group | null>(null);
  const keyframeAxesGroupRef = useRef<THREE.Group | null>(null);

  // Initialize THREE.js scene
  useEffect(() => {
    if (!containerRef.current) return;

    const container = containerRef.current;
    const width = container.clientWidth;
    const height = container.clientHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1e1e1e);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(5, 5, 5);
    camera.up.set(0, 0, 1);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 0.1;
    controls.maxDistance = 500;
    controls.target.set(0, 0, 0);
    controlsRef.current = controls;

    const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x222222);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    const cameraMarker = new THREE.Group();
    const sphereGeometry = new THREE.SphereGeometry(0.2, 16, 16);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    cameraMarker.add(sphere);
    const coneGeometry = new THREE.ConeGeometry(0.1, 0.5, 8);
    const coneMaterial = new THREE.MeshBasicMaterial({ color: 0xffff00 });
    const cone = new THREE.Mesh(coneGeometry, coneMaterial);
    cone.position.set(0, 0, -0.5);
    cone.rotation.x = Math.PI / 2;
    cameraMarker.add(cone);
    scene.add(cameraMarker);
    cameraMarkerRef.current = cameraMarker;

    const helperCamera = new THREE.PerspectiveCamera(75, 16 / 9, 0.1, 10);
    helperCamera.position.set(0, 0, 5);
    helperCameraRef.current = helperCamera;

    const cameraHelper = new THREE.CameraHelper(helperCamera);
    cameraHelper.visible = false;
    scene.add(cameraHelper);
    cameraHelperRef.current = cameraHelper;

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(5, 5, 5);
    scene.add(directionalLight);

    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
      controls.dispose();
      renderer.dispose();
      if (container && renderer.domElement) {
        container.removeChild(renderer.domElement);
      }
    };
  }, []);

  // Render keyframe poses as axis helpers
  useEffect(() => {
    if (!sceneRef.current) return;

    // Remove existing keyframe axes
    if (keyframeAxesGroupRef.current) {
      sceneRef.current.remove(keyframeAxesGroupRef.current);
      keyframeAxesGroupRef.current.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          child.geometry.dispose();
          if (Array.isArray(child.material)) {
            child.material.forEach((mat) => mat.dispose());
          } else {
            child.material.dispose();
          }
        }
      });
    }

    // Create new group for keyframe axes
    const axesGroup = new THREE.Group();
    const axisSize = 0.3; // Small but visible

    keyframePoses.forEach((pose) => {
      // Create axis helper for this pose
      const axisHelper = new THREE.AxesHelper(axisSize);
      axisHelper.position.copy(pose.position);
      axisHelper.quaternion.copy(pose.quaternion);
      axesGroup.add(axisHelper);
    });

    sceneRef.current.add(axesGroup);
    keyframeAxesGroupRef.current = axesGroup;

    if (keyframePoses.length > 0) {
      addDebugLog(`✓ Rendered ${keyframePoses.length} keyframe poses`);
    }
  }, [keyframePoses]);

  // Advertise topics
  useEffect(() => {
    advertisePoseIdRef.current = context.advertise?.(config.cameraPoseTopic, "geometry_msgs/PoseStamped", {
      datatypes: new Map([
        ["geometry_msgs/PoseStamped", { definitions: [{ name: "header", type: "std_msgs/Header", isArray: false, isComplex: true }, { name: "pose", type: "geometry_msgs/Pose", isArray: false, isComplex: true }] }],
        ["std_msgs/Header", { definitions: [{ name: "stamp", type: "time", isArray: false }, { name: "frame_id", type: "string", isArray: false }] }],
        ["geometry_msgs/Pose", { definitions: [{ name: "position", type: "geometry_msgs/Point", isArray: false, isComplex: true }, { name: "orientation", type: "geometry_msgs/Quaternion", isArray: false, isComplex: true }] }],
        ["geometry_msgs/Point", { definitions: [{ name: "x", type: "float64", isArray: false }, { name: "y", type: "float64", isArray: false }, { name: "z", type: "float64", isArray: false }] }],
        ["geometry_msgs/Quaternion", { definitions: [{ name: "x", type: "float64", isArray: false }, { name: "y", type: "float64", isArray: false }, { name: "z", type: "float64", isArray: false }, { name: "w", type: "float64", isArray: false }] }],
      ]),
    });
    advertiseCameraInfoIdRef.current = context.advertise?.(config.cameraInfoTopic, "sensor_msgs/CameraInfo", {
      datatypes: new Map([
        ["sensor_msgs/CameraInfo", { definitions: [ { name: "header", type: "std_msgs/Header", isArray: false, isComplex: true }, { name: "height", type: "uint32", isArray: false }, { name: "width", type: "uint32", isArray: false }, { name: "distortion_model", type: "string", isArray: false }, { name: "D", type: "float64", isArray: true }, { name: "K", type: "float64", isArray: true, arrayLength: 9 }, { name: "R", type: "float64", isArray: true, arrayLength: 9 }, { name: "P", type: "float64", isArray: true, arrayLength: 12 }, ] }],
        ["std_msgs/Header", { definitions: [{ name: "stamp", type: "time", isArray: false }, { name: "frame_id", type: "string", isArray: false }] }],
      ]),
    });

    return () => {
      if (advertisePoseIdRef.current) context.unadvertise?.(advertisePoseIdRef.current);
      if (advertiseCameraInfoIdRef.current) context.unadvertise?.(advertiseCameraInfoIdRef.current);
    };
  }, [config.cameraPoseTopic, config.cameraInfoTopic, context]);

  // Publish camera pose and intrinsics periodically
  useEffect(() => {
    const interval = setInterval(() => {
      if (!cameraRef.current || !rendererRef.current) return;

      const camera = cameraRef.current;
      const canvas = rendererRef.current.domElement;

      // Get camera pose in world frame.
      // After controls.update(), the camera's matrixWorld is up to date.
      const position = new THREE.Vector3();
      const quaternion = new THREE.Quaternion();
      camera.getWorldPosition(position);
      camera.getWorldQuaternion(quaternion);

      // Update the camera marker to show where the camera is
      if (cameraMarkerRef.current) {
        cameraMarkerRef.current.position.copy(position);
        cameraMarkerRef.current.quaternion.copy(quaternion);
        cameraMarkerRef.current.visible = true;
      }

      // Update the helper camera to visualize the published pose
      if (helperCameraRef.current && cameraHelperRef.current) {
        helperCameraRef.current.position.copy(position);
        helperCameraRef.current.quaternion.copy(quaternion);
        helperCameraRef.current.updateMatrixWorld();
        cameraHelperRef.current.update();
        cameraHelperRef.current.visible = showCameraHelper;
      }

      setCurrentPose({ position: position.clone(), quaternion: quaternion.clone() });

      const now = Date.now();
      const sec = Math.floor(now / 1000);
      const nsec = (now % 1000) * 1000000;
      const stamp = { sec, nsec };
      const header = { stamp, frame_id: "world" };

      // Publish Pose
      const poseMsg: PoseStamped = { header, pose: { position: { x: position.x, y: position.y, z: position.z }, orientation: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w } } };

      // Debug log every 10th publish to avoid spam
      if (Math.random() < 0.1) {
        console.log(`[PointCloud3DViewer] Publishing pose: pos=[${position.x.toFixed(3)}, ${position.y.toFixed(3)}, ${position.z.toFixed(3)}], quat=[w=${quaternion.w.toFixed(3)}, x=${quaternion.x.toFixed(3)}, y=${quaternion.y.toFixed(3)}, z=${quaternion.z.toFixed(3)}]`);
      }

      context.publish?.(config.cameraPoseTopic, poseMsg);

      // Publish CameraInfo
      const height = canvas.clientHeight;
      const width = canvas.clientWidth;
      const fov = camera.fov * (Math.PI / 180); // fov is in degrees, convert to radians
      const fy = height / (2 * Math.tan(fov / 2));
      const fx = fy * camera.aspect;
      const cx = width / 2;
      const cy = height / 2;

      const K = [fx, 0, cx, 0, fy, cy, 0, 0, 1];
      const P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0];
      const R = [1, 0, 0, 0, 1, 0, 0, 0, 1]; // Identity
      const D: number[] = []; // No distortion

      const cameraInfoMsg: CameraInfo = { header, height, width, distortion_model: "plumb_bob", D, K, R, P };
      context.publish?.(config.cameraInfoTopic, cameraInfoMsg);

    }, 1000 / config.publishRate);

    return () => clearInterval(interval);
  }, [config.cameraPoseTopic, config.cameraInfoTopic, config.publishRate, context, showCameraHelper]);

  // Subscribe to point cloud topic
  useLayoutEffect(() => {
    context.onRender = (state, done) => {
      setRenderDone(() => done);

      // Process point cloud messages
      if (state.currentFrame && sceneRef.current) {
        for (const message of state.currentFrame) {
          if (message.topic === config.pointCloudTopic) {
            const pointCloudMsg = message.message as any;
            setMessageCount((prev) => prev + 1);

            if (pointCloudRef.current) {
              sceneRef.current.remove(pointCloudRef.current);
              pointCloudRef.current.geometry.dispose();
              (pointCloudRef.current.material as THREE.Material).dispose();
            }

            if (pointCloudMsg.data && pointCloudMsg.fields) {
              try {
                let bytes: Uint8Array;
                if (typeof pointCloudMsg.data === "string") {
                  const binaryString = atob(pointCloudMsg.data);
                  bytes = new Uint8Array(binaryString.length);
                  for (let i = 0; i < binaryString.length; i++) {
                    bytes[i] = binaryString.charCodeAt(i);
                  }
                } else if (pointCloudMsg.data instanceof Uint8Array) {
                  bytes = pointCloudMsg.data;
                } else {
                  throw new Error(`Unsupported data type for point cloud: ${typeof pointCloudMsg.data}`);
                }

                const pointStride = pointCloudMsg.point_stride;
                const numPoints = Math.floor(bytes.length / pointStride);
                const positions = new Float32Array(numPoints * 3);
                const colors = new Float32Array(numPoints * 3);

                for (let i = 0; i < numPoints; i++) {
                  const offset = i * pointStride;
                  const posView = new DataView(bytes.buffer, offset, 12);
                  positions[i * 3 + 0] = posView.getFloat32(0, true);
                  positions[i * 3 + 1] = posView.getFloat32(4, true);
                  positions[i * 3 + 2] = posView.getFloat32(8, true);
                  colors[i * 3 + 0] = (bytes[offset + 12] ?? 255) / 255;
                  colors[i * 3 + 1] = (bytes[offset + 13] ?? 255) / 255;
                  colors[i * 3 + 2] = (bytes[offset + 14] ?? 255) / 255;
                }

                const geometry = new THREE.BufferGeometry();
                geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
                geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
                const material = new THREE.PointsMaterial({ size: 0.05, vertexColors: true });
                const points = new THREE.Points(geometry, material);
                sceneRef.current.add(points);
                pointCloudRef.current = points;
                setPointCount(numPoints);
                addDebugLog(`✓ Rendered ${numPoints} points successfully`);
              } catch (error) {
                addDebugLog(`✗ Parse error: ${(error as Error).message}`);
                console.error("[PointCloud3DViewer] Failed to parse point cloud:", error);
              }
            } else {
              addDebugLog(`✗ Message missing data or fields`);
            }
          } else if (message.topic === config.keyframePosesTopic) {
            // Message structure: { timestamp: {sec, nsec}, frame_id: string, poses: [...] }
            const posesInFrameMsg = message.message as any;
            addDebugLog(`📍 Keyframe pose message received on topic ${message.topic}`);

            try {
              if (posesInFrameMsg.poses && Array.isArray(posesInFrameMsg.poses)) {
                addDebugLog(`   Found poses array with ${posesInFrameMsg.poses.length} poses, frame="${posesInFrameMsg.frame_id}"`);

                const poses = posesInFrameMsg.poses.map((pose: any) => {
                  return {
                    position: new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z),
                    quaternion: new THREE.Quaternion(
                      pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w
                    ),
                  };
                });
                setKeyframePoses(poses);
                addDebugLog(`✓ Successfully parsed and rendered ${poses.length} keyframe poses`);
              } else {
                addDebugLog(`✗ Message missing poses array. Keys: ${Object.keys(posesInFrameMsg).join(', ')}`);
              }
            } catch (error) {
              addDebugLog(`✗ Failed to parse keyframe poses: ${(error as Error).message}`);
              addDebugLog(`   Raw message keys: ${Object.keys(posesInFrameMsg).join(', ')}`);
            }
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");
    context.subscribe([
      { topic: config.pointCloudTopic },
      { topic: config.keyframePosesTopic }
    ]);
  }, [config.pointCloudTopic, config.keyframePosesTopic, context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ width: "100%", height: "100%", display: "flex", flexDirection: "column", backgroundColor: "#1e1e1e", color: "#ffffff" }}>
      <div style={{ padding: "10px", backgroundColor: "#2a2a2a", borderBottom: "1px solid #444" }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <div style={{ fontSize: "14px", fontWeight: "bold" }}>3D Point Cloud Viewer</div>
          <div style={{ display: "flex", gap: "5px" }}>
            <button onClick={() => setShowCameraHelper(!showCameraHelper)} style={{ padding: "4px 8px", fontSize: "10px", backgroundColor: showCameraHelper ? "#0066cc" : "#444", color: "#fff", border: "none", borderRadius: "3px", cursor: "pointer" }}>
              {showCameraHelper ? "📷 Helper On" : "📷 Helper Off"}
            </button>
            <button onClick={() => setShowDebug(!showDebug)} style={{ padding: "4px 8px", fontSize: "10px", backgroundColor: "#444", color: "#fff", border: "none", borderRadius: "3px", cursor: "pointer" }}>
              {showDebug ? "Hide Debug" : "Show Debug"}
            </button>
          </div>
        </div>
        <div style={{ fontSize: "11px", color: "#888", marginTop: "4px" }}>
          <div>Point Cloud Topic: {config.pointCloudTopic}</div>
          <div>
            {pointCount > 0 ? <span style={{ color: "#4caf50" }}>✓ {pointCount.toLocaleString()} points loaded</span> : messageCount > 0 ? <span style={{ color: "#ff9800" }}>⚠ {messageCount} messages received (parsing issue?)</span> : <span style={{ color: "#f44336" }}>⚠ No messages received</span>}
          </div>
          <div>Camera Pose Topic: {config.cameraPoseTopic} ({config.publishRate} Hz)</div>
          <div>Camera Info Topic: {config.cameraInfoTopic}</div>
          {currentPose && (
            <div style={{ marginTop: "4px", fontSize: "10px", fontFamily: "monospace", color: "#aaa" }}>
              Pos: [{currentPose.position.x.toFixed(2)}, {currentPose.position.y.toFixed(2)}, {currentPose.position.z.toFixed(2)}] | Quat: [w={currentPose.quaternion.w.toFixed(2)}, x={currentPose.quaternion.x.toFixed(2)}, y={currentPose.quaternion.y.toFixed(2)}, z={currentPose.quaternion.z.toFixed(2)}]
            </div>
          )}
          {keyframePoses.length > 0 && (
            <div style={{ marginTop: "6px", fontSize: "10px", color: "#aaa" }}>
              <div style={{ color: "#4caf50", marginBottom: "2px" }}>✓ {keyframePoses.length} keyframe poses loaded</div>
              {keyframePoses.slice(0, 3).map((pose, idx) => (
                <div key={idx} style={{ fontFamily: "monospace", fontSize: "9px", color: "#999", marginLeft: "8px" }}>
                  KF{idx + 1}: Pos=[{pose.position.x.toFixed(2)}, {pose.position.y.toFixed(2)}, {pose.position.z.toFixed(2)}] Quat=[w={pose.quaternion.w.toFixed(2)}, x={pose.quaternion.x.toFixed(2)}, y={pose.quaternion.y.toFixed(2)}, z={pose.quaternion.z.toFixed(2)}]
                </div>
              ))}
            </div>
          )}
        </div>
      </div>
      <div ref={containerRef} style={{ flex: 1, position: "relative" }} />
      {showDebug && (
        <div style={{ height: "150px", backgroundColor: "#0d0d0d", borderTop: "1px solid #444", padding: "8px", overflowY: "auto", fontFamily: "monospace", fontSize: "11px" }}>
          <div style={{ marginBottom: "8px", color: "#aaa" }}>
            Debug Logs:
            <button onClick={() => setDebugLogs([])} style={{ marginLeft: "10px", padding: "2px 6px", fontSize: "10px", backgroundColor: "#333", color: "#fff", border: "none", borderRadius: "2px", cursor: "pointer" }}>
              Clear
            </button>
          </div>
          {debugLogs.length === 0 ? <div style={{ color: "#666" }}>No logs yet...</div> : debugLogs.map((log, idx) => <div key={idx} style={{ marginBottom: "4px", color: "#ddd" }}>{log}</div>)}
        </div>
      )}
    </div>
  );
}

export function initPointCloud3DViewer(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<PointCloud3DViewer context={context} />);

  return () => {
    root.unmount();
  };
}
