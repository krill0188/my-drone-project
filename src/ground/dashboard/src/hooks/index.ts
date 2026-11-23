// Custom hooks for the dashboard

import { useEffect, useRef, useCallback, useState } from 'react';
import { useSwarmStore, useVehiclesArray, useSelectedVehicle, useFilteredSafetyEvents, useRecentAIDetections, useMeshQoSArray } from '@/store/swarmStore';
import type { MapViewport, DroneState, SafetyEvent, AIDetection, MeshQoSMetrics } from '@/types';

// WebSocket hook
export function useWebSocket(url: string, onMessage: (message: any) => void) {
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) return;

    try {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        setIsConnected(true);
        setError(null);
      };

      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          onMessage(message);
        } catch (err) {
          console.error('Failed to parse WS message:', err);
        }
      };

      ws.onclose = () => {
        setIsConnected(false);
        // Reconnect after 3 seconds
        reconnectTimeoutRef.current = setTimeout(connect, 3000);
      };

      ws.onerror = (err) => {
        setError('WebSocket error');
        console.error('WebSocket error:', err);
      };
    } catch (err) {
      setError('Failed to create WebSocket connection');
    }
  }, [url, onMessage]);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [connect]);

  const send = useCallback((message: any) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    }
  }, []);

  return { isConnected, error, send };
}

// Vehicle hooks
export function useVehicles() {
  return useVehiclesArray();
}

export function useSelectedVehicle() {
  return useSelectedVehicle();
}

export function useFlyingVehicles() {
  return useSwarmStore((state) => 
    Array.from(state.vehicles.values()).filter(v => v.is_flying)
  );
}

export function useArmedVehicles() {
  return useSwarmStore((state) => 
    Array.from(state.vehicles.values()).filter(v => v.arming_state === 'ARMED')
  );
}

// Formation hooks
export function useFormation() {
  return useSwarmStore((state) => state.formation);
}

// Mesh QoS hooks
export function useMeshQoS() {
  return useMeshQoSArray();
}

export function useMeshQoSForNode(nodeId: number) {
  return useSwarmStore((state) => state.meshQoS.get(nodeId));
}

// Safety Event hooks
export function useSafetyEvents() {
  return useFilteredSafetyEvents();
}

export function useUnacknowledgedSafetyEvents() {
  return useSwarmStore((state) => 
    state.safetyEvents.filter(e => !e.acknowledged)
  );
}

export function useCriticalSafetyEvents() {
  return useSwarmStore((state) => 
    state.safetyEvents.filter(e => e.severity === 'CRITICAL')
  );
}

// AI Detection hooks
export function useAIDetections(limit = 50) {
  return useRecentAIDetections(limit);
}

export function useAIDetectionsByClass() {
  const detections = useRecentAIDetections(200);
  const byClass = new Map<string, number>();
  
  detections.forEach(d => {
    byClass.set(d.class_name, (byClass.get(d.class_name) || 0) + 1);
  });
  
  return byClass;
}

// Map hooks
export function useMapViewport() {
  const viewport = useSwarmStore((state) => state.mapViewport);
  const setViewport = useSwarmStore((state) => state.setMapViewport);
  return [viewport, setViewport] as const;
}

// Connection hooks
export function useConnection() {
  const isConnected = useSwarmStore((state) => state.isConnected);
  const connectionError = useSwarmStore((state) => state.connectionError);
  const setConnected = useSwarmStore((state) => state.setConnected);
  const setConnectionError = useSwarmStore((state) => state.setConnectionError);
  
  return { isConnected, connectionError, setConnected, setConnectionError };
}

// UI State hooks
export function useActiveTab() {
  const activeTab = useSwarmStore((state) => state.activeTab);
  const setActiveTab = useSwarmStore((state) => state.setActiveTab);
  return [activeTab, setActiveTab] as const;
}

export function useSidebar() {
  const sidebarOpen = useSwarmStore((state) => state.sidebarOpen);
  const toggleSidebar = useSwarmStore((state) => state.toggleSidebar);
  return [sidebarOpen, toggleSidebar] as const;
}

export function useRightPanel() {
  const rightPanelOpen = useSwarmStore((state) => state.rightPanelOpen);
  const toggleRightPanel = useSwarmStore((state) => state.toggleRightPanel);
  return [rightPanelOpen, toggleRightPanel] as const;
}

// Debounced value hook
export function useDebouncedValue<T>(value: T, delay: number): T {
  const [debouncedValue, setDebouncedValue] = useState(value);

  useEffect(() => {
    const timer = setTimeout(() => setDebouncedValue(value), delay);
    return () => clearTimeout(timer);
  }, [value, delay]);

  return debouncedValue;
}

// Local storage hook
export function useLocalStorage<T>(key: string, initialValue: T) {
  const [storedValue, setStoredValue] = useState<T>(() => {
    try {
      const item = window.localStorage.getItem(key);
      return item ? JSON.parse(item) : initialValue;
    } catch {
      return initialValue;
    }
  });

  const setValue = useCallback((value: T | ((val: T) => T)) => {
    try {
      const valueToStore = value instanceof Function ? value(storedValue) : value;
      setStoredValue(valueToStore);
      window.localStorage.setItem(key, JSON.stringify(valueToStore));
    } catch (error) {
      console.error(`Error setting localStorage key "${key}":`, error);
    }
  }, [key, storedValue]);

  return [storedValue, setValue] as const;
}

// Window size hook
export function useWindowSize() {
  const [size, setSize] = useState({
    width: typeof window !== 'undefined' ? window.innerWidth : 0,
    height: typeof window !== 'undefined' ? window.innerHeight : 0,
  });

  useEffect(() => {
    const handleResize = () => {
      setSize({
        width: window.innerWidth,
        height: window.innerHeight,
      });
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  return size;
}

// Media query hook
export function useMediaQuery(query: string): boolean {
  const [matches, setMatches] = useState(false);

  useEffect(() => {
    const media = window.matchMedia(query);
    if (media.matches !== matches) {
      setMatches(media.matches);
    }
    
    const listener = (event: MediaQueryListEvent) => setMatches(event.matches);
    media.addEventListener('change', listener);
    
    return () => media.removeEventListener('change', listener);
  }, [matches, query]);

  return matches;
}

// Keyboard shortcut hook
export function useKeyboardShortcut(
  key: string,
  callback: () => void,
  options: { ctrl?: boolean; shift?: boolean; alt?: boolean; meta?: boolean } = {}
) {
  useEffect(() => {
    const handler = (event: KeyboardEvent) => {
      if (
        event.key.toLowerCase() === key.toLowerCase() &&
        event.ctrlKey === !!options.ctrl &&
        event.shiftKey === !!options.shift &&
        event.altKey === !!options.alt &&
        event.metaKey === !!options.meta
      ) {
        event.preventDefault();
        callback();
      }
    };

    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [key, callback, options.ctrl, options.shift, options.alt, options.meta]);
}

// Copy to clipboard hook
export function useCopyToClipboard() {
  const [copied, setCopied] = useState(false);

  const copy = useCallback(async (text: string) => {
    try {
      await navigator.clipboard.writeText(text);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
      return true;
    } catch (err) {
      console.error('Failed to copy:', err);
      return false;
    }
  }, []);

  return { copied, copy };
}