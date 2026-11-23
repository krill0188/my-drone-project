import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';
import type {
  DroneState,
  SwarmFormationState,
  MeshQoSMetrics,
  SafetyEvent,
  SwarmSummary,
  AIDetection,
  AnyWSMessage,
  MapViewport,
  SafetyEventFilter,
} from '@/types';

interface SwarmStore {
  // Vehicles
  vehicles: Map<number, DroneState>;
  selectedVehicleId: number | null;
  
  // Formation
  formation: SwarmFormationState | null;
  
  // Mesh QoS
  meshQoS: Map<number, MeshQoSMetrics>;
  
  // Safety Events
  safetyEvents: SafetyEvent[];
  safetyEventFilter: SafetyEventFilter;
  
  // AI Detections
  aiDetections: AIDetection[];
  
  // Summary
  summary: SwarmSummary | null;
  
  // Map
  mapViewport: MapViewport;
  
  // Connection
  isConnected: boolean;
  connectionError: string | null;
  
  // UI State
  activeTab: 'live' | 'replay' | 'analytics';
  sidebarOpen: boolean;
  rightPanelOpen: boolean;
  
  // Actions - Vehicles
  setVehicle: (vehicle: DroneState) => void;
  setVehicles: (vehicles: DroneState[]) => void;
  removeVehicle: (id: number) => void;
  selectVehicle: (id: number | null) => void;
  getVehicle: (id: number) => DroneState | undefined;
  getAllVehicles: () => DroneState[];
  
  // Actions - Formation
  setFormation: (formation: SwarmFormationState) => void;
  
  // Actions - Mesh QoS
  setMeshQoS: (qos: MeshQoSMetrics) => void;
  setMeshQoSMap: (qosMap: Map<number, MeshQoSMetrics>) => void;
  
  // Actions - Safety Events
  addSafetyEvent: (event: SafetyEvent) => void;
  acknowledgeSafetyEvent: (timestamp: string, ackBy: number) => void;
  setSafetyEventFilter: (filter: Partial<SafetyEventFilter>) => void;
  clearSafetyEvents: () => void;
  
  // Actions - AI
  addAIDetection: (detection: AIDetection) => void;
  clearAIDetections: () => void;
  
  // Actions - Summary
  setSummary: (summary: SwarmSummary) => void;
  
  // Actions - Map
  setMapViewport: (viewport: Partial<MapViewport>) => void;
  
  // Actions - Connection
  setConnected: (connected: boolean) => void;
  setConnectionError: (error: string | null) => void;
  
  // Actions - UI
  setActiveTab: (tab: 'live' | 'replay' | 'analytics') => void;
  toggleSidebar: () => void;
  toggleRightPanel: () => void;
  
  // WebSocket handler
  handleWSMessage: (message: AnyWSMessage) => void;
}

const defaultMapViewport: MapViewport = {
  center: [37.7749, -122.4194],
  zoom: 14,
  bearing: 0,
  pitch: 0,
};

export const useSwarmStore = create<SwarmStore>()(
  subscribeWithSelector((set, get) => ({
    // State
    vehicles: new Map(),
    selectedVehicleId: null,
    formation: null,
    meshQoS: new Map(),
    safetyEvents: [],
    safetyEventFilter: {},
    aiDetections: [],
    summary: null,
    mapViewport: defaultMapViewport,
    isConnected: false,
    connectionError: null,
    activeTab: 'live',
    sidebarOpen: true,
    rightPanelOpen: false,
    
    // Vehicle actions
    setVehicle: (vehicle) => set((state) => {
      const newVehicles = new Map(state.vehicles);
      newVehicles.set(vehicle.id, vehicle);
      return { vehicles: newVehicles };
    }),
    
    setVehicles: (vehicles) => set({
      vehicles: new Map(vehicles.map(v => [v.id, v])),
    }),
    
    removeVehicle: (id) => set((state) => {
      const newVehicles = new Map(state.vehicles);
      newVehicles.delete(id);
      return { 
        vehicles: newVehicles,
        selectedVehicleId: state.selectedVehicleId === id ? null : state.selectedVehicleId,
      };
    }),
    
    selectVehicle: (id) => set({ selectedVehicleId: id }),
    
    getVehicle: (id) => get().vehicles.get(id),
    
    getAllVehicles: () => Array.from(get().vehicles.values()),
    
    // Formation actions
    setFormation: (formation) => set({ formation }),
    
    // Mesh QoS actions
    setMeshQoS: (qos) => set((state) => {
      const newMeshQoS = new Map(state.meshQoS);
      newMeshQoS.set(qos.node_id, qos);
      return { meshQoS: newMeshQoS };
    }),
    
    setMeshQoSMap: (qosMap) => set({ meshQoS: qosMap }),
    
    // Safety Event actions
    addSafetyEvent: (event) => set((state) => ({
      safetyEvents: [event, ...state.safetyEvents].slice(0, 1000),
    })),
    
    acknowledgeSafetyEvent: (timestamp, ackBy) => set((state) => ({
      safetyEvents: state.safetyEvents.map(event => 
        event.timestamp === timestamp && !event.acknowledged
          ? { ...event, acknowledged: true, acknowledged_by: ackBy, acknowledged_time: new Date().toISOString() }
          : event
      ),
    })),
    
    setSafetyEventFilter: (filter) => set((state) => ({
      safetyEventFilter: { ...state.safetyEventFilter, ...filter },
    })),
    
    clearSafetyEvents: () => set({ safetyEvents: [] }),
    
    // AI actions
    addAIDetection: (detection) => set((state) => ({
      aiDetections: [detection, ...state.aiDetections].slice(0, 500),
    })),
    
    clearAIDetections: () => set({ aiDetections: [] }),
    
    // Summary actions
    setSummary: (summary) => set({ summary }),
    
    // Map actions
    setMapViewport: (viewport) => set((state) => ({
      mapViewport: { ...state.mapViewport, ...viewport },
    })),
    
    // Connection actions
    setConnected: (connected) => set({ isConnected: connected }),
    setConnectionError: (error) => set({ connectionError: error }),
    
    // UI actions
    setActiveTab: (tab) => set({ activeTab: tab }),
    toggleSidebar: () => set((state) => ({ sidebarOpen: !state.sidebarOpen })),
    toggleRightPanel: () => set((state) => ({ rightPanelOpen: !state.rightPanelOpen })),
    
    // WebSocket handler
    handleWSMessage: (message) => {
      const state = get();
      
      switch (message.type) {
        case 'vehicle_update':
          state.setVehicle(message.payload);
          break;
        case 'formation_update':
          state.setFormation(message.payload);
          break;
        case 'mesh_qos_update':
          state.setMeshQoS(message.payload);
          break;
        case 'safety_event':
          state.addSafetyEvent(message.payload);
          break;
        case 'swarm_summary':
          state.setSummary(message.payload);
          break;
      }
    },
  }))
);

// Selectors for derived state
export const useSelectedVehicle = () => useSwarmStore(
  (state) => state.selectedVehicleId ? state.vehicles.get(state.selectedVehicleId) : null
);

export const useVehiclesArray = () => useSwarmStore(
  (state) => Array.from(state.vehicles.values())
);

export const useFlyingVehicles = () => useSwarmStore(
  (state) => Array.from(state.vehicles.values()).filter(v => v.is_flying)
);

export const useArmedVehicles = () => useSwarmStore(
  (state) => Array.from(state.vehicles.values()).filter(v => v.arming_state === 'ARMED')
);

export const useFilteredSafetyEvents = () => useSwarmStore(
  (state) => {
    const { safetyEvents, safetyEventFilter } = state;
    return safetyEvents.filter(event => {
      if (safetyEventFilter.severity?.length && !safetyEventFilter.severity.includes(event.severity)) return false;
      if (safetyEventFilter.source_ids?.length && !safetyEventFilter.source_ids.includes(event.source_id)) return false;
      if (safetyEventFilter.event_types?.length && !safetyEventFilter.event_types.includes(event.event_type)) return false;
      if (safetyEventFilter.acknowledged !== undefined && event.acknowledged !== safetyEventFilter.acknowledged) return false;
      if (safetyEventFilter.date_range) {
        const eventTime = new Date(event.timestamp).getTime();
        const [start, end] = safetyEventFilter.date_range.map(d => new Date(d).getTime());
        if (eventTime < start || eventTime > end) return false;
      }
      return true;
    });
  }
);

export const useRecentAIDetections = (limit = 50) => useSwarmStore(
  (state) => state.aiDetections.slice(0, limit)
);

export const useMeshQoSArray = () => useSwarmStore(
  (state) => Array.from(state.meshQoS.values())
);