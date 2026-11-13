import React, {
  useEffect,
  useRef,
  useState,
  useMemo,
  useCallback,
  memo,
} from "react";
import { cn } from "@/lib/utils";

import URDFManipulator from "urdf-loader/src/urdf-manipulator-element.js";
import { useUrdf } from "@/hooks/useUrdf";
import { useRealTimeJoints } from "@/hooks/useRealTimeJoints";
import { useApi } from "@/contexts/ApiContext";
import {
  createUrdfViewer,
  fitGridToRobot,
  setupMeshLoader,
  setupJointHighlighting,
  setupModelLoading,
  URDFViewerElement,
} from "@/lib/urdfViewerHelpers";

// Register the URDFManipulator as a custom element if it hasn't been already
if (typeof window !== "undefined" && !customElements.get("urdf-viewer")) {
  customElements.define("urdf-viewer", URDFManipulator);
}
import * as THREE from "three";

// Extend the interface for the URDF viewer element to include background property
interface UrdfViewerElement extends HTMLElement {
  background?: string;
  setJointValue?: (jointName: string, value: number) => void;
}

const UrdfViewer: React.FC<{ virtual?: boolean; sessionError?: string }> = ({ virtual = false, sessionError = "" }) => {
  const { baseUrl, fetchWithHeaders } = useApi();
  const [controlError, setControlError] = useState("");
  const [cartesian, setCartesian] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);
  const [highlightedJoint, setHighlightedJoint] = useState<string | null>(null);
  const { registerUrdfProcessor, alternativeUrdfModels, isDefaultModel } =
    useUrdf();

  const cleanupAnimationRef = useRef<(() => void) | null>(null);
  const viewerRef = useRef<URDFViewerElement | null>(null);
  const hasInitializedRef = useRef<boolean>(false);

  // Real-time joint updates via WebSocket
  const { isConnected: isWebSocketConnected } = useRealTimeJoints({
    viewerRef,
    enabled: isDefaultModel, // Only enable WebSocket for default model
  });

  // Add state for custom URDF path
  const [customUrdfPath, setCustomUrdfPath] = useState<string | null>(null);
  const [urlModifierFunc, setUrlModifierFunc] = useState<
    ((url: string) => string) | null
  >(null);

  const packageRef = useRef<string>("");

  // Implement UrdfProcessor interface for drag and drop
  const urdfProcessor = useMemo(
    () => ({
      loadUrdf: (urdfPath: string) => {
        setCustomUrdfPath(urdfPath);
      },
      setUrlModifierFunc: (func: (url: string) => string) => {
        setUrlModifierFunc(() => func);
      },
      getPackage: () => {
        return packageRef.current;
      },
    }),
    []
  );

  // Register the URDF processor with the global drag and drop context
  useEffect(() => {
    registerUrdfProcessor(urdfProcessor);
  }, [registerUrdfProcessor, urdfProcessor]);

  // Create URL modifier function for default model
  const defaultUrlModifier = useCallback((url: string) => {
    console.log(`🔗 defaultUrlModifier called with: ${url}`);

    // Handle various package:// URL formats for the default SO-101 model
    if (url.startsWith("package://so_arm_description/meshes/")) {
      const modifiedUrl = url.replace(
        "package://so_arm_description/meshes/",
        "/so-101-urdf/meshes/"
      );
      console.log(`🔗 Modified URL (package): ${modifiedUrl}`);
      return modifiedUrl;
    }

    // Handle case where package path might be partially resolved
    if (url.includes("so_arm_description/meshes/")) {
      const modifiedUrl = url.replace(
        /.*so_arm_description\/meshes\//,
        "/so-101-urdf/meshes/"
      );
      console.log(`🔗 Modified URL (partial): ${modifiedUrl}`);
      return modifiedUrl;
    }

    // Handle the specific problematic path pattern we're seeing in logs
    if (url.includes("/so-101-urdf/so_arm_description/meshes/")) {
      const modifiedUrl = url.replace(
        "/so-101-urdf/so_arm_description/meshes/",
        "/so-101-urdf/meshes/"
      );
      console.log(`🔗 Modified URL (problematic path): ${modifiedUrl}`);
      return modifiedUrl;
    }

    // Handle relative paths that might need mesh folder prefix
    if (
      url.endsWith(".stl") &&
      !url.startsWith("/") &&
      !url.startsWith("http")
    ) {
      const modifiedUrl = `/so-101-urdf/meshes/${url}`;
      console.log(`🔗 Modified URL (relative): ${modifiedUrl}`);
      return modifiedUrl;
    }

    console.log(`🔗 Unmodified URL: ${url}`);
    return url;
  }, []);

  // Main effect to create and setup the viewer only once
  useEffect(() => {
    if (!containerRef.current) return;

    // Create and configure the URDF viewer element
    const viewer = createUrdfViewer(containerRef.current, true);
    viewerRef.current = viewer; // Store reference to the viewer
    // URDFManipulator invokes updateJoint only for a mouse drag; live feedback
    // uses setJointValue directly and therefore never sends a motion command.
    const interactive = viewer as URDFViewerElement & {
      dragControls?: { updateJoint: (joint: {name: string}, angle: number) => void; setGrabbed: (grabbed: boolean) => void };
      __draggingJoint?: string | null;
      __cartesianDrag?: boolean;
      world: THREE.Object3D;
    };
    let requestPending = false;
    let holdPending = false;
    let pendingRequest: Promise<void> | null = null;
    let cancelled = false;
    let desired: { joint: string; angle: number } | { delta: number[] } | null = null;
    let cartesianReady = false;
    let cartesianBegin: Promise<void> | null = null;
    let pointerStart: { x: number; y: number } | null = null;
    let lastPointer: { x: number; y: number } | null = null;
    let dragTimer: ReturnType<typeof setInterval> | null = null;
    const sendDesired = () => {
      if (cancelled || requestPending || holdPending || !desired || (cartesian && !cartesianReady)) return;
      const command = desired;
      requestPending = true;
      pendingRequest = fetchWithHeaders(`${baseUrl}/${cartesian ? "virtual/cartesian/move" : "virtual/move"}`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(command),
      }).then(r => r.json()).then(data => {
        if (!data.success && !cancelled) setControlError(previous => previous || data.message || "Ошибка управления");
      }).catch(() => { if (!cancelled) setControlError(previous => previous || "Нет связи с роботом"); })
        .finally(() => { requestPending = false; pendingRequest = null; });
    };
    const onHoverJoint = () => { viewer.controls.enabled = false; };
    const onLeaveJoint = () => { if (!interactive.__draggingJoint) viewer.controls.enabled = true; };
    const onDragStart = (event: Event) => {
      interactive.__draggingJoint = (event as CustomEvent).detail;
      interactive.__cartesianDrag = cartesian;
      desired = null;
      if (dragTimer) clearInterval(dragTimer);
      if (cartesian) {
        pointerStart = lastPointer;
        cartesianReady = false;
        cartesianBegin = fetchWithHeaders(`${baseUrl}/virtual/cartesian/begin`, { method: "POST" })
          .then(r => r.json()).then(data => {
            if (!data.success) { if (!cancelled) setControlError(data.message || "Ошибка кинематики"); return; }
            cartesianReady = true;
          }).catch(() => { if (!cancelled) setControlError("Нет связи с роботом"); });
      }
      dragTimer = setInterval(sendDesired, 60);
    };
    const onDragEnd = () => {
      const joint = interactive.__draggingJoint;
      interactive.__draggingJoint = null;
      interactive.__cartesianDrag = false;
      desired = null;
      if (dragTimer) clearInterval(dragTimer);
      dragTimer = null;
      if (!joint || cancelled) return;
      holdPending = true;
      const outstanding = Promise.all([pendingRequest || Promise.resolve(), cartesianBegin || Promise.resolve()]);
      outstanding.then(() => fetchWithHeaders(`${baseUrl}/${cartesian ? "virtual/cartesian/release" : "virtual/release"}`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(cartesian ? {} : { joint }), keepalive: true,
      })).then(r => r.json()).then(data => {
        if (!data.success && !cancelled) setControlError(previous => previous || data.message || "Ошибка остановки");
      }).catch(() => { if (!cancelled) setControlError(previous => previous || "Нет связи с роботом"); })
        .finally(() => { holdPending = false; });
    };
    const releaseDrag = () => {
      desired = null;
      if (dragTimer) clearInterval(dragTimer);
      dragTimer = null;
      interactive.dragControls?.setGrabbed(false);
      interactive.__draggingJoint = null;
    };
    const recordPointer = (event: PointerEvent) => {
      lastPointer = { x: event.clientX, y: event.clientY };
      if (!cartesian || !interactive.__draggingJoint || !pointerStart) return;
      const right = new THREE.Vector3(1, 0, 0).applyQuaternion(viewer.camera.quaternion);
      const up = new THREE.Vector3(0, 1, 0).applyQuaternion(viewer.camera.quaternion);
      const delta = right.multiplyScalar((event.clientX - pointerStart.x) * 0.00075)
        .add(up.multiplyScalar(-(event.clientY - pointerStart.y) * 0.00075));
      const frame = new THREE.Quaternion();
      interactive.world.getWorldQuaternion(frame);
      delta.applyQuaternion(frame.invert());
      if (delta.length() > 0.06) delta.setLength(0.06);
      desired = { delta: delta.toArray() };
      sendDesired();
    };
    if (virtual && interactive.dragControls) {
      window.addEventListener("pointerdown", recordPointer, true);
      window.addEventListener("pointermove", recordPointer, true);
      window.addEventListener("mouseup", releaseDrag);
      window.addEventListener("pointerup", releaseDrag);
      window.addEventListener("pointercancel", releaseDrag);
      window.addEventListener("blur", releaseDrag);
      interactive.addEventListener("joint-mouseover", onHoverJoint);
      interactive.addEventListener("joint-mouseout", onLeaveJoint);
      interactive.addEventListener("manipulate-start", onDragStart);
      interactive.addEventListener("manipulate-end", onDragEnd);
      interactive.dragControls.updateJoint = (joint, angle) => {
        if (cartesian) return;
        viewer.setJointValue(joint.name, angle);
        if (!interactive.__draggingJoint || !Number.isFinite(angle)) return;
        desired = { joint: joint.name, angle };
        sendDesired();
      };
    }


    // Setup mesh loading function with appropriate URL modifier
    const activeUrlModifier = isDefaultModel
      ? defaultUrlModifier
      : urlModifierFunc;
    setupMeshLoader(viewer, activeUrlModifier);

    // Determine which URDF to load - fixed path to match the actual available file
    const urdfPath = isDefaultModel
      ? "/so-101-urdf/urdf/so101_new_calib.urdf"
      : customUrdfPath || "";

    // Set the package path for the default model
    if (isDefaultModel) {
      packageRef.current = "/"; // Set to root so we can handle full path resolution in URL modifier
    }

    // Setup model loading if a path is available
    let cleanupModelLoading = () => {};
    if (urdfPath) {
      cleanupModelLoading = setupModelLoading(
        viewer,
        urdfPath,
        packageRef.current,
        setCustomUrdfPath,
        alternativeUrdfModels
      );
    }

    // Setup joint highlighting
    const cleanupJointHighlighting = setupJointHighlighting(
      viewer,
      setHighlightedJoint
    );

    // Function to fit the robot to the camera view
    const fitRobotToView = (viewer: URDFViewerElement) => {
      if (!viewer || !viewer.robot) {
        console.log(
          "[RobotViewer] Cannot fit to view: No viewer or robot available"
        );
        return;
      }

      try {
        // Create a bounding box for the robot
        const boundingBox = new THREE.Box3().setFromObject(viewer.robot);

        // Calculate the center of the bounding box
        const center = new THREE.Vector3();
        boundingBox.getCenter(center);

        // Calculate the size of the bounding box
        const size = new THREE.Vector3();
        boundingBox.getSize(size);

        // Get the maximum dimension to ensure the entire robot is visible
        const maxDim = Math.max(size.x, size.y, size.z);

        // Rescale the ground grid to this robot's actual unit scale — it
        // starts as a 1x1 placeholder since it's created before any model
        // is loaded (see createUrdfViewer).
        fitGridToRobot(viewer, center, boundingBox.min.y, maxDim);

        // Position camera to see the center of the model
        viewer.camera.position.copy(center);

        // Move the camera back to see the entire robot
        // Use the model's up direction to determine which axis to move along
        const upVector = new THREE.Vector3();
        if (viewer.up === "+Z" || viewer.up === "Z") {
          upVector.set(1, 1, 1); // Move back in a diagonal
        } else if (viewer.up === "+Y" || viewer.up === "Y") {
          upVector.set(1, 1, 1); // Move back in a diagonal
        } else {
          upVector.set(1, 1, 1); // Default direction
        }

        // Normalize the vector and multiply by the size
        upVector.normalize().multiplyScalar(maxDim * 1.3);
        viewer.camera.position.add(upVector);

        // Make the camera look at the center of the model
        viewer.controls.target.copy(center);

        // Update controls and mark for redraw
        viewer.controls.update();
        viewer.redraw();

        console.log("[RobotViewer] Robot auto-fitted to view");
      } catch (error) {
        console.error("[RobotViewer] Error fitting robot to view:", error);
      }
    };

    // Add event listener for when the robot is loaded to auto-fit to view
    const onRobotLoad = () => {
      fitRobotToView(viewer);
    };

    // Setup animation event handler for the default model or when hasAnimation is true
    const onModelProcessed = () => {
      hasInitializedRef.current = true;
      if ("setJointValue" in viewer) {
        // Clear any existing animation
        if (cleanupAnimationRef.current) {
          cleanupAnimationRef.current();
          cleanupAnimationRef.current = null;
        }
      }
      // Auto-fit the robot to view when the model is processed
      onRobotLoad();
    };

    viewer.addEventListener("urdf-processed", onModelProcessed);

    // Return cleanup function
    return () => {
      if (cleanupAnimationRef.current) {
        cleanupAnimationRef.current();
        cleanupAnimationRef.current = null;
      }
      hasInitializedRef.current = false;
      cancelled = true;
      window.removeEventListener("pointerdown", recordPointer, true);
      window.removeEventListener("pointermove", recordPointer, true);
      window.removeEventListener("mouseup", releaseDrag);
      window.removeEventListener("pointerup", releaseDrag);
      window.removeEventListener("pointercancel", releaseDrag);
      window.removeEventListener("blur", releaseDrag);
      desired = null;
      if (dragTimer) clearInterval(dragTimer);
      interactive.removeEventListener("joint-mouseover", onHoverJoint);
      interactive.removeEventListener("joint-mouseout", onLeaveJoint);
      interactive.removeEventListener("manipulate-start", onDragStart);
      interactive.removeEventListener("manipulate-end", onDragEnd);
      cleanupJointHighlighting();
      cleanupModelLoading();
      viewer.removeEventListener("urdf-processed", onModelProcessed);
    };
  }, [
    isDefaultModel,
    customUrdfPath,
    urlModifierFunc,
    defaultUrlModifier,
    alternativeUrdfModels,
    virtual, cartesian, baseUrl, fetchWithHeaders,
  ]);

  return (
    <div
      className={cn(
        "w-full h-full transition-all duration-300 ease-in-out relative",
        "bg-gradient-to-br from-gray-900 to-gray-800"
      )}
    >
      <div ref={containerRef} className="w-full h-full" />

      {virtual && <div className="absolute bottom-4 left-4 z-10 flex gap-2 bg-black/80 rounded p-2">
        <button type="button" className={`px-3 py-1 rounded ${!cartesian ? "bg-blue-700" : "bg-gray-700"} text-white`} onClick={() => setCartesian(false)}>Суставы</button>
        <button type="button" className={`px-3 py-1 rounded ${cartesian ? "bg-blue-700" : "bg-gray-700"} text-white`} onClick={() => setCartesian(true)}>Вся рука</button>
      </div>}
      {sessionError && <div className="absolute inset-0 z-20 bg-black/60 flex items-center justify-center p-6"><div className="bg-red-900 text-white p-5 rounded-lg max-w-md">Управление остановлено: {sessionError}. Вернитесь стрелкой и запустите Teleoperation снова.</div></div>}
      {virtual && <div className="absolute top-4 left-4 z-10 bg-black/80 text-white px-3 py-2 rounded text-sm">{cartesian ? "Тяните модель: конец руки переместится в плоскости экрана" : "Тяните отдельное звено мышью"} · W: закрыть · E: открыть</div>}
      {controlError && <div className="absolute bottom-4 left-4 z-10 bg-red-900 text-white px-3 py-2 rounded text-sm">{controlError}</div>}
      {/* Joint highlight indicator */}
      {highlightedJoint && (
        <div className="absolute bottom-4 right-4 bg-black/70 text-white px-3 py-2 rounded-md text-sm font-mono z-10">
          Joint: {highlightedJoint}
        </div>
      )}

      {/* WebSocket connection status */}
      {isDefaultModel && (
        <div className="absolute top-4 right-4 z-10">
          <div
            className={`flex items-center gap-2 px-3 py-2 rounded-md text-sm font-mono ${
              isWebSocketConnected
                ? "bg-green-900/70 text-green-300"
                : "bg-red-900/70 text-red-300"
            }`}
          >
            <div
              className={`w-2 h-2 rounded-full ${
                isWebSocketConnected ? "bg-green-400" : "bg-red-400"
              }`}
            />
            {isWebSocketConnected ? "Live Robot Data" : "Disconnected"}
          </div>
        </div>
      )}
    </div>
  );
};

export default memo(UrdfViewer);
