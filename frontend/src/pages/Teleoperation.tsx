import React, { useCallback, useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";
import VisualizerPanel from "@/components/control/VisualizerPanel";
import TeleopCameraPanel from "@/components/control/TeleopCameraPanel";
import { useToast } from "@/hooks/use-toast";
import { useApi } from "@/contexts/ApiContext";

const TeleoperationPage = () => {
  const navigate = useNavigate();
  const { toast } = useToast();
  const { baseUrl, fetchWithHeaders } = useApi();

  // Stop teleoperation exactly once, however the user leaves, so the back
  // button, an in-app link, and the unmount safety net can't double-stop or
  // double-toast.
  const virtual = new URLSearchParams(window.location.search).get("virtual") === "1";
  const stopPath = virtual ? "virtual/stop" : "stop-teleoperation";
  const [sessionError, setSessionError] = useState("");
  const stoppedRef = useRef(false);
  const stopTeleoperation = useCallback(async () => {
    if (stoppedRef.current) return;
    stoppedRef.current = true;
    try {
      const res = await fetchWithHeaders(`${baseUrl}/${stopPath}`, {
        method: "POST",
      });
      const data = await res.json();
      if (data?.success) {
        toast({
          title: "Teleoperation stopped",
          description: "The arm was disconnected cleanly.",
        });
      }
    } catch {
      /* best-effort */
    }
  }, [baseUrl, fetchWithHeaders, toast, stopPath]);

  // Cover every exit path so a session can't keep running and block the next
  // start with "already active":
  //   - the back button awaits stopTeleoperation() then navigates (below);
  //   - any other in-app navigation unmounts this component → stop via cleanup;
  //   - a browser-level leave (URL change, reload, tab close) never runs React
  //     cleanup, so `pagehide` fires a keepalive stop that survives the unload
  //     and stashes a flag the next page reads to confirm the clean disconnect.
  //     It uses a bare fetch (no JSON Content-Type) so the request stays a CORS
  //     "simple request" and isn't dropped to a preflight mid-unload.
  useEffect(() => {
    const handlePageHide = () => {
      try {
        sessionStorage.setItem("lelab:teleop-stopped", "1");
      } catch {
        /* sessionStorage may be unavailable; the stop below still runs */
      }
      fetch(`${baseUrl}/${stopPath}`, {
        method: "POST",
        keepalive: true,
      }).catch(() => {});
    };
    window.addEventListener("pagehide", handlePageHide);

    return () => {
      window.removeEventListener("pagehide", handlePageHide);
      stopTeleoperation();
    };
  }, [baseUrl, stopTeleoperation, stopPath]);

  useEffect(() => {
    if (!virtual) return;
    const timer = setInterval(() => {
      fetchWithHeaders(`${baseUrl}/virtual/status`).then(r => r.json()).then(data => {
        if (!data.success) setSessionError(previous => previous || data.message || "Управление отключено");
      }).catch(() => setSessionError(previous => previous || "Нет связи с LeLab"));
    }, 1000);
    return () => clearInterval(timer);
  }, [virtual, baseUrl, fetchWithHeaders]);

  useEffect(() => {
    if (!virtual || sessionError) return;
    let held: "KeyW" | "KeyE" | null = null;
    let timer: ReturnType<typeof setInterval> | null = null;
    let pending: Promise<void> | null = null;
    let stopping = false;
    const send = () => {
      if (!held || pending || stopping) return;
      const angle = held === "KeyW" ? 0 : 100 * Math.PI / 180;
      pending = fetchWithHeaders(`${baseUrl}/virtual/move`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ joint: "Jaw", angle }),
      }).then(r => r.json()).then(data => {
        if (!data.success) setSessionError(previous => previous || data.message || "Ошибка захвата");
      }).catch(() => setSessionError(previous => previous || "Нет связи с LeLab"))
        .finally(() => { pending = null; });
    };
    const release = () => {
      if (!held) return;
      held = null;
      if (timer) clearInterval(timer);
      timer = null;
      stopping = true;
      (pending || Promise.resolve()).then(() => fetchWithHeaders(`${baseUrl}/virtual/release`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ joint: "Jaw" }), keepalive: true,
      })).catch(() => {}).finally(() => { stopping = false; });
    };
    const down = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      if (event.repeat || event.ctrlKey || event.altKey || event.metaKey || stopping || held ||
          target?.isContentEditable || ["INPUT", "TEXTAREA", "SELECT"].includes(target?.tagName || "")) return;
      if (event.code !== "KeyW" && event.code !== "KeyE") return;
      event.preventDefault(); held = event.code; send(); timer = setInterval(send, 100);
    };
    const up = (event: KeyboardEvent) => { if (event.code === held) release(); };
    window.addEventListener("keydown", down); window.addEventListener("keyup", up); window.addEventListener("blur", release);
    return () => { release(); window.removeEventListener("keydown", down); window.removeEventListener("keyup", up); window.removeEventListener("blur", release); };
  }, [virtual, sessionError, baseUrl, fetchWithHeaders]);

  const handleGoBack = async () => {
    await stopTeleoperation();
    navigate("/");
  };

  return (
    <div className="min-h-screen bg-black flex items-center justify-center p-2 sm:p-4">
      <div className="w-full h-[95vh] flex">
        <VisualizerPanel
          onGoBack={handleGoBack}
          virtual={virtual}
          sessionError={sessionError}
          className="lg:w-full"
          rightSlot={<TeleopCameraPanel />}
        />
      </div>
    </div>
  );
};

export default TeleoperationPage;
