import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type Config = {
  pointCloudTopic: string;
  cameraPoseTopic: string;
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

function PointCloud3DViewer({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config] = useState<Config>(() => {
    const partialConfig = context.initialState as Partial<Config>;
    return {
      pointCloudTopic: partialConfig.pointCloudTopic ?? "/gaussian_splats/pointcloud",
      cameraPoseTopic: partialConfig.cameraPoseTopic ?? "/viewer/camera_pose",
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
  const advertiseIdRef = useRef<string | undefined | void>();
  const helperCameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const cameraHelperRef = useRef<THREE.CameraHelper | null>(null);
  const cameraMarkerRef = useRef<THREE.Group | null>(null);

  // Initialize THREE.js scene
  useEffect(() => {
    if (!containerRef.current) return;

    const container = containerRef.current;
    const width = container.clientWidth;
    const height = container.clientHeight;

    // Create scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1e1e1e);
    sceneRef.current = scene;

    // Create camera with Z-up coordinate system
    const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(5, 5, 5); // View from an angle
    camera.up.set(0, 0, 1); // Z is up
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    // Create renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Add orbit controls with Z-up
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 0.1;
    controls.maxDistance = 500;
    controls.target.set(0, 0, 0); // Look at origin
    controlsRef.current = controls;

    // Add grid helper on XY plane (Z-up coordinate system)
    // Rotate grid 90 degrees so it's horizontal with Z-up
    const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x222222);
    gridHelper.rotation.x = Math.PI / 2; // Rotate to XY plane
    scene.add(gridHelper);

    // Add axes helper (Red=X, Green=Y, Blue=Z)
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    // Create a visible marker to show camera position
    const cameraMarker = new THREE.Group();

    // Add a sphere at camera position
    const sphereGeometry = new THREE.SphereGeometry(0.2, 16, 16);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    cameraMarker.add(sphere);

    // Add a cone pointing in view direction
    const coneGeometry = new THREE.ConeGeometry(0.1, 0.5, 8);
    const coneMaterial = new THREE.MeshBasicMaterial({ color: 0xffff00 });
    const cone = new THREE.Mesh(coneGeometry, coneMaterial);
    cone.position.set(0, 0, -0.5); // Point forward
    cone.rotation.x = Math.PI / 2; // Point cone along Z axis
    cameraMarker.add(cone);

    scene.add(cameraMarker);
    cameraMarkerRef.current = cameraMarker;

    // Create a helper camera to visualize the published camera pose
    const helperCamera = new THREE.PerspectiveCamera(75, 16 / 9, 0.1, 10);
    helperCamera.position.set(0, 0, 5);
    helperCameraRef.current = helperCamera;

    // Create camera helper to visualize the frustum
    const cameraHelper = new THREE.CameraHelper(helperCamera);
    cameraHelper.visible = false; // Start with helper off
    scene.add(cameraHelper);
    cameraHelperRef.current = cameraHelper;

    // Add ambient light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    // Add directional light
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(5, 5, 5);
    scene.add(directionalLight);

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    // Handle resize
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
      container.removeChild(renderer.domElement);
    };
  }, []);

  // Advertise camera pose topic
  useEffect(() => {
    if (advertiseIdRef.current) {
      return;
    }

    advertiseIdRef.current = context.advertise?.(config.cameraPoseTopic, "geometry_msgs/PoseStamped", {
      datatypes: new Map([
        [
          "geometry_msgs/PoseStamped",
          {
            definitions: [
              { name: "header", type: "std_msgs/Header", isArray: false, isComplex: true },
              { name: "pose", type: "geometry_msgs/Pose", isArray: false, isComplex: true },
            ],
          },
        ],
        [
          "std_msgs/Header",
          {
            definitions: [
              { name: "stamp", type: "time", isArray: false },
              { name: "frame_id", type: "string", isArray: false },
            ],
          },
        ],
        [
          "geometry_msgs/Pose",
          {
            definitions: [
              { name: "position", type: "geometry_msgs/Point", isArray: false, isComplex: true },
              {
                name: "orientation",
                type: "geometry_msgs/Quaternion",
                isArray: false,
                isComplex: true,
              },
            ],
          },
        ],
        [
          "geometry_msgs/Point",
          {
            definitions: [
              { name: "x", type: "float64", isArray: false },
              { name: "y", type: "float64", isArray: false },
              { name: "z", type: "float64", isArray: false },
            ],
          },
        ],
        [
          "geometry_msgs/Quaternion",
          {
            definitions: [
              { name: "x", type: "float64", isArray: false },
              { name: "y", type: "float64", isArray: false },
              { name: "z", type: "float64", isArray: false },
              { name: "w", type: "float64", isArray: false },
            ],
          },
        ],
      ]),
    });

    return () => {
      if (advertiseIdRef.current && typeof advertiseIdRef.current === "string") {
        context.unadvertise?.(advertiseIdRef.current);
        advertiseIdRef.current = undefined;
      }
    };
  }, [config.cameraPoseTopic, context]);

  // Publish camera pose periodically
  useEffect(() => {
    const interval = setInterval(() => {
      if (!cameraRef.current || !controlsRef.current) return;

      const camera = cameraRef.current;
      const controls = controlsRef.current;

      // Get camera position in world frame
      const position = camera.position;
      const target = controls.target; // Should be (0, 0, 0)

      // Compute camera orientation in world frame
      // World frame: Z-up (robotics convention)
      // Camera frame: IMPORTANT - Camera looks down -Z axis (standard convention!)
      //               +X = right, +Y = up, +Z = backward (away from scene)
      //
      // We need a quaternion representing: camera-frame → world-frame transformation

      // Step 1: Compute view direction (from camera towards target)
      const viewDirection = new THREE.Vector3();
      viewDirection.subVectors(target, position).normalize();
      // This is where camera's -Z axis points in world coordinates

      // Step 2: World Z-up vector
      const worldUp = new THREE.Vector3(0, 0, 1);

      // Step 3: Camera +X axis (right) = view direction × world up
      const cameraX = new THREE.Vector3();
      cameraX.crossVectors(viewDirection, worldUp).normalize();

      // Step 4: Camera +Y axis (up) = right × view direction
      const cameraY = new THREE.Vector3();
      cameraY.crossVectors(cameraX, viewDirection).normalize();

      // Step 5: Camera +Z axis = opposite of view direction (camera looks down -Z)
      const cameraZ = new THREE.Vector3();
      cameraZ.copy(viewDirection).negate();

      // Step 6: Build rotation matrix [camera-frame axes as columns]
      // This matrix transforms from camera frame to world frame
      const rotationMatrix = new THREE.Matrix4();
      rotationMatrix.makeBasis(cameraX, cameraY, cameraZ);

      // Extract quaternion (camera-to-world rotation)
      const quaternion = new THREE.Quaternion();
      quaternion.setFromRotationMatrix(rotationMatrix);

      // Debug: log the camera axes occasionally
      if (Math.random() < 0.05) {
        addDebugLog(`Cam frame in world: X=${cameraX.toArray().map(v => v.toFixed(2))} Y=${cameraY.toArray().map(v => v.toFixed(2))} Z=${cameraZ.toArray().map(v => v.toFixed(2))}`);
      }

      // Update the camera marker to show where the camera is
      if (cameraMarkerRef.current) {
        cameraMarkerRef.current.position.copy(position);

        // Orient the marker to point towards the target
        cameraMarkerRef.current.lookAt(target);

        // Make marker visible
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

      // Store current pose for UI display
      setCurrentPose({
        position: position.clone(),
        quaternion: quaternion.clone(),
      });

      const now = Date.now();
      const poseMsg: PoseStamped = {
        header: {
          stamp: {
            sec: Math.floor(now / 1000),
            nsec: (now % 1000) * 1000000,
          },
          frame_id: "world",
        },
        pose: {
          position: {
            x: position.x,
            y: position.y,
            z: position.z,
          },
          orientation: {
            x: quaternion.x,
            y: quaternion.y,
            z: quaternion.z,
            w: quaternion.w,
          },
        },
      };

      context.publish?.(config.cameraPoseTopic, poseMsg);
    }, 1000 / config.publishRate);

    return () => clearInterval(interval);
  }, [config.cameraPoseTopic, config.publishRate, context, showCameraHelper]);

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

            const msgInfo = {
              topic: message.topic,
              schemaName: message.schemaName,
              messageKeys: Object.keys(pointCloudMsg),
              hasData: !!pointCloudMsg.data,
              hasFields: !!pointCloudMsg.fields,
              pointStride: pointCloudMsg.point_stride,
              frameId: pointCloudMsg.frame_id,
            };

            addDebugLog(`Received message: ${JSON.stringify(msgInfo, null, 2)}`);
            console.log("[PointCloud3DViewer] Received message:", msgInfo);

            // Remove old point cloud
            if (pointCloudRef.current) {
              sceneRef.current.remove(pointCloudRef.current);
              pointCloudRef.current.geometry.dispose();
              (pointCloudRef.current.material as THREE.Material).dispose();
            }

            // Parse foxglove.PointCloud message
            if (pointCloudMsg.data && pointCloudMsg.fields) {
              try {
                addDebugLog(`Parsing: data type=${typeof pointCloudMsg.data}, stride=${pointCloudMsg.point_stride}`);

                let bytes: Uint8Array;

                // Handle both base64 string and direct Uint8Array
                if (typeof pointCloudMsg.data === "string") {
                  // Decode base64 data
                  addDebugLog(`Decoding base64 string...`);
                  const binaryString = atob(pointCloudMsg.data);
                  bytes = new Uint8Array(binaryString.length);
                  for (let i = 0; i < binaryString.length; i++) {
                    bytes[i] = binaryString.charCodeAt(i);
                  }
                } else if (pointCloudMsg.data instanceof Uint8Array) {
                  // Data is already a Uint8Array
                  addDebugLog(`Data is already Uint8Array`);
                  bytes = pointCloudMsg.data;
                } else if (ArrayBuffer.isView(pointCloudMsg.data)) {
                  // Data is some other typed array
                  addDebugLog(`Converting typed array to Uint8Array`);
                  bytes = new Uint8Array(pointCloudMsg.data.buffer);
                } else if (pointCloudMsg.data.buffer && pointCloudMsg.data.byteLength) {
                  // Data has buffer property (likely a typed array)
                  addDebugLog(`Extracting from buffer property`);
                  bytes = new Uint8Array(pointCloudMsg.data.buffer);
                } else {
                  // Log more info about the object
                  const dataInfo = {
                    type: typeof pointCloudMsg.data,
                    constructor: pointCloudMsg.data.constructor?.name,
                    keys: Object.keys(pointCloudMsg.data).slice(0, 10),
                    isArray: Array.isArray(pointCloudMsg.data),
                  };
                  addDebugLog(`✗ Unsupported data: ${JSON.stringify(dataInfo)}`);
                  throw new Error(`Unsupported data type`);
                }

                const pointStride = pointCloudMsg.point_stride;
                const numPoints = Math.floor(bytes.length / pointStride);

                addDebugLog(`Decoded: ${bytes.length} bytes, ${numPoints} points`);

                const positions = new Float32Array(numPoints * 3);
                const colors = new Float32Array(numPoints * 3);

                // Extract positions and colors
                for (let i = 0; i < numPoints; i++) {
                  const offset = i * pointStride;

                  // Position (xyz as float32)
                  const posView = new DataView(bytes.buffer, offset, 12);
                  positions[i * 3 + 0] = posView.getFloat32(0, true);
                  positions[i * 3 + 1] = posView.getFloat32(4, true);
                  positions[i * 3 + 2] = posView.getFloat32(8, true);

                  // Color (rgba as uint8)
                  colors[i * 3 + 0] = (bytes[offset + 12] ?? 255) / 255; // red
                  colors[i * 3 + 1] = (bytes[offset + 13] ?? 255) / 255; // green
                  colors[i * 3 + 2] = (bytes[offset + 14] ?? 255) / 255; // blue
                }

                // Create point cloud
                const geometry = new THREE.BufferGeometry();
                geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
                geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

                const material = new THREE.PointsMaterial({
                  size: 0.05,
                  vertexColors: true,
                });

                const points = new THREE.Points(geometry, material);
                sceneRef.current.add(points);
                pointCloudRef.current = points;
                setPointCount(numPoints);

                addDebugLog(`✓ Rendered ${numPoints} points successfully`);
                console.log(`[PointCloud3DViewer] Rendered ${numPoints} points`);
              } catch (error) {
                addDebugLog(`✗ Parse error: ${error}`);
                console.error("[PointCloud3DViewer] Failed to parse point cloud:", error);
              }
            } else {
              addDebugLog(`✗ Message missing data or fields`);
              console.warn("[PointCloud3DViewer] Message missing 'data' or 'fields'");
            }
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");
    context.subscribe([{ topic: config.pointCloudTopic }]);
  }, [config.pointCloudTopic, context]);

  // Call done callback after render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div
      style={{
        width: "100%",
        height: "100%",
        display: "flex",
        flexDirection: "column",
        backgroundColor: "#1e1e1e",
        color: "#ffffff",
      }}
    >
      <div
        style={{
          padding: "10px",
          backgroundColor: "#2a2a2a",
          borderBottom: "1px solid #444",
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <div style={{ fontSize: "14px", fontWeight: "bold" }}>3D Point Cloud Viewer</div>
          <div style={{ display: "flex", gap: "5px" }}>
            <button
              onClick={() => setShowCameraHelper(!showCameraHelper)}
              style={{
                padding: "4px 8px",
                fontSize: "10px",
                backgroundColor: showCameraHelper ? "#0066cc" : "#444",
                color: "#fff",
                border: "none",
                borderRadius: "3px",
                cursor: "pointer",
              }}
            >
              {showCameraHelper ? "📷 Helper On" : "📷 Helper Off"}
            </button>
            <button
              onClick={() => setShowDebug(!showDebug)}
              style={{
                padding: "4px 8px",
                fontSize: "10px",
                backgroundColor: "#444",
                color: "#fff",
                border: "none",
                borderRadius: "3px",
                cursor: "pointer",
              }}
            >
              {showDebug ? "Hide Debug" : "Show Debug"}
            </button>
          </div>
        </div>
        <div style={{ fontSize: "11px", color: "#888", marginTop: "4px" }}>
          <div>Point Cloud: {config.pointCloudTopic}</div>
          <div>
            {pointCount > 0 ? (
              <span style={{ color: "#4caf50" }}>✓ {pointCount.toLocaleString()} points loaded</span>
            ) : messageCount > 0 ? (
              <span style={{ color: "#ff9800" }}>⚠ {messageCount} messages received (parsing issue?)</span>
            ) : (
              <span style={{ color: "#f44336" }}>⚠ No messages received</span>
            )}
          </div>
          <div>Camera Pose: {config.cameraPoseTopic} ({config.publishRate} Hz)</div>
          {currentPose && (
            <div style={{ marginTop: "4px", fontSize: "10px", fontFamily: "monospace", color: "#aaa" }}>
              Pos: [{currentPose.position.x.toFixed(2)}, {currentPose.position.y.toFixed(2)}, {currentPose.position.z.toFixed(2)}]
              {" | "}
              Quat: [{currentPose.quaternion.x.toFixed(2)}, {currentPose.quaternion.y.toFixed(2)}, {currentPose.quaternion.z.toFixed(2)}, {currentPose.quaternion.w.toFixed(2)}]
            </div>
          )}
          <div style={{ marginTop: "4px", color: "#aaa" }}>
            🖱️ Left-drag: Rotate | Right-drag: Pan | Scroll: Zoom
          </div>
          <div style={{ marginTop: "2px", fontSize: "10px", color: "#666" }}>
            Coordinate frame: Z-up (Red=X, Green=Y, Blue=Z)
          </div>
        </div>
      </div>
      <div ref={containerRef} style={{ flex: 1, position: "relative" }} />
      {showDebug && (
        <div
          style={{
            height: "200px",
            backgroundColor: "#0d0d0d",
            borderTop: "1px solid #444",
            padding: "8px",
            overflowY: "auto",
            fontFamily: "monospace",
            fontSize: "11px",
          }}
        >
          <div style={{ marginBottom: "8px", color: "#aaa" }}>
            Debug Logs (last 20):
            <button
              onClick={() => setDebugLogs([])}
              style={{
                marginLeft: "10px",
                padding: "2px 6px",
                fontSize: "10px",
                backgroundColor: "#333",
                color: "#fff",
                border: "none",
                borderRadius: "2px",
                cursor: "pointer",
              }}
            >
              Clear
            </button>
          </div>
          {debugLogs.length === 0 ? (
            <div style={{ color: "#666" }}>No logs yet...</div>
          ) : (
            debugLogs.map((log, idx) => (
              <div key={idx} style={{ marginBottom: "4px", color: "#ddd" }}>
                {log}
              </div>
            ))
          )}
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
