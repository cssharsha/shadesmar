import { ExtensionContext } from "@foxglove/extension";

import { initPointCloud3DViewer } from "./PointCloud3DViewer";
import { initRenderedImagePanel } from "./RenderedImagePanel";

export function activate(extensionContext: ExtensionContext): void {
  // 3D viewer with camera controls that publishes camera poses
  extensionContext.registerPanel({
    name: "3D Point Cloud Viewer",
    initPanel: initPointCloud3DViewer,
  });

  // Image viewer for displaying rendered images
  extensionContext.registerPanel({
    name: "Rendered Image Viewer",
    initPanel: initRenderedImagePanel,
  });
}
