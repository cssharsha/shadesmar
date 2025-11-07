import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type Config = {
  cameraPoseTopic: string;
  renderedImageTopic: string;
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

function GaussianSplatRendererPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config] = useState<Config>(() => {
    const partialConfig = context.initialState as Partial<Config>;
    return {
      cameraPoseTopic: partialConfig.cameraPoseTopic ?? "/viewer/camera_pose",
      renderedImageTopic: partialConfig.renderedImageTopic ?? "/rendering/output",
      publishRate: partialConfig.publishRate ?? 5, // Hz
    };
  });

  const [renderState, setRenderState] = useState<Immutable<RenderState> | undefined>();
  const [renderedImage, setRenderedImage] = useState<string | undefined>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const advertiseIdRef = useRef<string | undefined | void>();

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

  // Setup rendering and subscriptions
  useLayoutEffect(() => {
    context.onRender = (state, done) => {
      setRenderDone(() => done);
      setRenderState(state);

      // Process messages from rendered image topic
      if (state.currentFrame) {
        for (const message of state.currentFrame) {
          if (message.topic === config.renderedImageTopic) {
            const compressedImage = message.message as any;

            // Log the message structure for debugging
            console.log("[GaussianSplatRenderer] Received message:", {
              topic: message.topic,
              schemaName: message.schemaName,
              messageKeys: Object.keys(compressedImage),
              format: compressedImage.format,
              hasData: !!compressedImage.data,
              dataType: typeof compressedImage.data,
              dataLength: compressedImage.data?.length,
            });

            // Handle foxglove.CompressedImage format
            if (compressedImage.data) {
              let imageUrl: string;

              // Check if data is already a string (base64) or needs conversion
              if (typeof compressedImage.data === "string") {
                // Data is already base64 string
                const format = compressedImage.format || "jpeg";
                imageUrl = `data:image/${format};base64,${compressedImage.data}`;
              } else if (compressedImage.data instanceof Uint8Array) {
                // Data is a byte array, convert to base64
                const bytes = Array.from(compressedImage.data) as number[];
                const base64 = btoa(String.fromCharCode(...bytes));
                const format = compressedImage.format || "jpeg";
                imageUrl = `data:image/${format};base64,${base64}`;
              } else {
                console.error("[GaussianSplatRenderer] Unsupported data type:", typeof compressedImage.data);
                continue;
              }

              console.log("[GaussianSplatRenderer] Setting image URL, length:", imageUrl.length);
              setRenderedImage(imageUrl);
            } else {
              console.warn("[GaussianSplatRenderer] Message missing data field");
            }
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");
    context.subscribe([{ topic: config.renderedImageTopic }]);
  }, [config.renderedImageTopic, context]);

  // Publish camera pose periodically using a timer
  useEffect(() => {
    let lastLoggedState = false;
    let lastPosition = [0, 0, 0];

    const interval = setInterval(() => {
      if (!renderState) {
        return;
      }

      // Extract camera transform from render state
      const rs = renderState as any;

      // Try to get camera state from different possible locations
      let cameraState = rs.cameraState;

      // Log render state structure for debugging (only first time or when missing)
      if (!lastLoggedState) {
        console.log("[GaussianSplatRenderer] RenderState keys:", Object.keys(renderState));
        console.log("[GaussianSplatRenderer] cameraState:", cameraState);
        lastLoggedState = true;
      }

      if (!cameraState) {
        console.warn("[GaussianSplatRenderer] No cameraState in renderState");
        return;
      }

      // Extract position and orientation from camera state
      // The actual structure depends on Foxglove's API version
      let position = [0, 0, 5];
      let orientation = [0, 0, 0, 1]; // quaternion [x, y, z, w]

      // Try different camera state structures
      if (cameraState.perspective) {
        const persp = cameraState.perspective;

        // The perspective camera typically stores target position
        if (persp.targetOffset) {
          position = persp.targetOffset;
        } else if (persp.target) {
          position = persp.target;
        }

        // Log spherical coordinates for debugging (only when position changes)
        if (persp.phi !== undefined && persp.thetaOffset !== undefined) {
          const posChanged =
            Math.abs((position[0] ?? 0) - (lastPosition[0] ?? 0)) > 0.01 ||
            Math.abs((position[1] ?? 0) - (lastPosition[1] ?? 0)) > 0.01 ||
            Math.abs((position[2] ?? 0) - (lastPosition[2] ?? 0)) > 0.01;

          if (posChanged) {
            console.log("[GaussianSplatRenderer] Camera changed - position:", position,
                       "phi:", persp.phi, "theta:", persp.thetaOffset, "distance:", persp.distance);
            lastPosition = [...position];
          }
        }
      } else if (cameraState.target && cameraState.targetOrientation) {
        position = cameraState.target;
        orientation = cameraState.targetOrientation;
      }

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
            x: position[0] ?? 0,
            y: position[1] ?? 0,
            z: position[2] ?? 5,
          },
          orientation: {
            x: orientation[0] ?? 0,
            y: orientation[1] ?? 0,
            z: orientation[2] ?? 0,
            w: orientation[3] ?? 1,
          },
        },
      };

      context.publish?.(config.cameraPoseTopic, poseMsg);
    }, 1000 / config.publishRate);

    return () => clearInterval(interval);
  }, [renderState, config.cameraPoseTopic, config.publishRate, context]);

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
        padding: "10px",
      }}
    >
      <div style={{ marginBottom: "10px" }}>
        <h3 style={{ margin: "0 0 10px 0", fontSize: "16px" }}>Gaussian Splat Interactive Renderer</h3>
        <div style={{ fontSize: "12px", color: "#888" }}>
          <div>Camera Pose Topic: {config.cameraPoseTopic}</div>
          <div>Rendered Image Topic: {config.renderedImageTopic}</div>
          <div>Publish Rate: {config.publishRate} Hz</div>
        </div>
      </div>

      <div style={{ flex: 1, display: "flex", alignItems: "center", justifyContent: "center", overflow: "hidden" }}>
        {renderedImage ? (
          <img
            src={renderedImage}
            alt="Rendered view"
            style={{
              maxWidth: "100%",
              maxHeight: "100%",
              objectFit: "contain",
            }}
          />
        ) : (
          <div style={{ textAlign: "center", color: "#666" }}>
            <div style={{ fontSize: "48px", marginBottom: "10px" }}>📷</div>
            <div>Waiting for rendered image...</div>
            <div style={{ fontSize: "12px", marginTop: "5px" }}>
              Make sure the Gaussian splatting backend is running
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

export function initGaussianSplatRendererPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GaussianSplatRendererPanel context={context} />);

  return () => {
    root.unmount();
  };
}
