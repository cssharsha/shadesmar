import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type Config = {
  renderedImageTopic: string;
};

function RenderedImagePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config] = useState<Config>(() => {
    const partialConfig = context.initialState as Partial<Config>;
    return {
      renderedImageTopic: partialConfig.renderedImageTopic ?? "/rendering/output",
    };
  });

  const [renderedImage, setRenderedImage] = useState<string | undefined>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [lastUpdateTime, setLastUpdateTime] = useState<number>(0);
  const [debugLogs, setDebugLogs] = useState<string[]>([]);
  const [showDebug, setShowDebug] = useState(false);

  const addDebugLog = (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setDebugLogs((prev) => [...prev.slice(-20), `[${timestamp}] ${message}`]);
  };

  // Setup rendering and subscriptions
  useLayoutEffect(() => {
    context.onRender = (state, done) => {
      setRenderDone(() => done);

      // Process messages from rendered image topic
      if (state.currentFrame) {
        for (const message of state.currentFrame) {
          if (message.topic === config.renderedImageTopic) {
            const compressedImage = message.message as any;

            addDebugLog(
              `Received message: format=${compressedImage.format}, data_length=${compressedImage.data?.length}`,
            );

            // Handle foxglove.CompressedImage format
            if (compressedImage.data) {
              let imageUrl: string;

              // Check if data is already a string (base64) or needs conversion
              if (typeof compressedImage.data === "string") {
                const format = compressedImage.format || "jpeg";
                imageUrl = `data:image/${format};base64,${compressedImage.data}`;
              } else if (compressedImage.data instanceof Uint8Array) {
                const bytes = Array.from(compressedImage.data) as number[];
                const base64 = btoa(String.fromCharCode(...bytes));
                const format = compressedImage.format || "jpeg";
                imageUrl = `data:image/${format};base64,${base64}`;
              } else {
                addDebugLog(`Unsupported data type: ${typeof compressedImage.data}`);
                continue;
              }

              setRenderedImage(imageUrl);
              setLastUpdateTime(Date.now());
            }
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");
    context.subscribe([{ topic: config.renderedImageTopic }]);
  }, [config.renderedImageTopic, context]);

  // Call done callback after render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const formatTimeSince = (timestamp: number) => {
    if (timestamp === 0) return "Never";
    const seconds = Math.floor((Date.now() - timestamp) / 1000);
    if (seconds < 1) return "just now";
    if (seconds < 60) return `${seconds}s ago`;
    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return `${minutes}m ago`;
    const hours = Math.floor(minutes / 60);
    return `${hours}h ago`;
  };

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
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
        }}
      >
        <div>
          <div style={{ fontSize: "14px", fontWeight: "bold" }}>Rendered Image</div>
          <div style={{ fontSize: "11px", color: "#888", marginTop: "4px" }}>
            <div>Topic: {config.renderedImageTopic}</div>
            <div>Last update: {formatTimeSince(lastUpdateTime)}</div>
          </div>
        </div>
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

      <div
        style={{
          flex: 1,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          overflow: "hidden",
          padding: "10px",
          position: "relative",
        }}
      >
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
              Check for messages on topic: {config.renderedImageTopic}
            </div>
          </div>
        )}
      </div>
      {showDebug && (
        <div
          style={{
            height: "150px",
            backgroundColor: "#0d0d0d",
            borderTop: "1px solid #444",
            padding: "8px",
            overflowY: "auto",
            fontFamily: "monospace",
            fontSize: "11px",
          }}
        >
          <div style={{ marginBottom: "8px", color: "#aaa" }}>
            Debug Logs:
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

export function initRenderedImagePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RenderedImagePanel context={context} />);

  return () => {
    root.unmount();
  };
}
