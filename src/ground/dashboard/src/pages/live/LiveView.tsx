// Live view page - Real-time swarm monitoring

import React from 'react';
import { useSwarmStore, useVehiclesArray, useSelectedVehicle, useFormation, useMeshQoSArray, useFilteredSafetyEvents, useUnacknowledgedSafetyEvents, useCriticalSafetyEvents, useRecentAIDetections, useConnection, useActiveTab, useSidebar, useRightPanel, useMapViewport } from '@/hooks';
import { DroneMap } from '@/components/map';
import { BatteryChart, LinkQualityChart, FormationErrorCDFChart, DetectionDistributionChart, RadarChart } from '@/components/charts';
import { Card, CardHeader, CardContent, Badge, Button, Avatar, Table, Modal, Spinner, Divider, Tooltip } from '@/components/common';
import { MapPin, Drone, Battery, Signal, AlertTriangle, Shield, Eye, EyeOff, Settings, RefreshCw, AlertCircle, CheckCircle, XCircle, MapPin as MapPinIcon, RotateCcw, Trash2, Download, Upload, Search, Filter, ChevronLeft, ChevronRight, Menu, X, Bell, BellOff, Layers, Grid, Map, Zap, Wifi, Satellite, Home, Play, Pause, Stop, FastForward, Rewind, SkipBack, SkipForward, Volume2, VolumeX, Fullscreen, Minimize, Maximize, RotateCw as Refresh } from 'lucide-react';
import { cn, formatPercentage, formatDistance, formatSpeed, formatAltitude, formatTimeAgo, getSeverityBadgeColor, getBatteryColor, getLinkQualityColor, getRSSIColor } from '@/utils/helpers';
import type { DroneState, SwarmFormationState, SafetyEvent, AIDetection } from '@/types';

const statusColors = {
  ARMED: 'bg-success-100 text-success-700',
  DISARMED: 'bg-gray-100 text-gray-700',
  FLYING: 'bg-primary-100 text-primary-700',
  HOVERING: 'bg-blue-100 text-blue-700',
  RTL: 'bg-warning-100 text-warning-700',
  LANDING: 'bg-orange-100 text-orange-700',
  EMERGENCY: 'bg-danger-100 text-danger-700',
};

export const LiveView: React.FC = () => {
  const vehicles = useVehiclesArray();
  const selectedVehicle = useSelectedVehicle();
  const formation = useFormation();
  const meshQoS = useMeshQoSArray();
  const safetyEvents = useFilteredSafetyEvents();
  const unacknowledgedEvents = useUnacknowledgedSafetyEvents();
  const criticalEvents = useCriticalSafetyEvents();
  const aiDetections = useRecentAIDetections(50);
  const { isConnected, connectionError, setConnected } = useConnection();
  const [activeTab, setActiveTab] = useActiveTab();
  const [sidebarOpen, toggleSidebar] = useSidebar();
  const [rightPanelOpen, toggleRightPanel] = useRightPanel();
  const [mapViewport, setMapViewport] = useMapViewport();
  const [showFormationPaths, setShowFormationPaths] = React.useState(true);
  const [showCoverage, setShowCoverage] = React.useState(false);
  const [selectedSafetyEvent, setSelectedSafetyEvent] = React.useState<SafetyEvent | null>(null);
  const [showVehicleDetails, setShowVehicleDetails] = React.useState(false);

  const flyingVehicles = vehicles.filter(v => v.is_flying);
  const armedVehicles = vehicles.filter(v => v.arming_state === 'ARMED');

  const handleVehicleClick = (vehicle: DroneState) => {
    useSwarmStore.getState().selectVehicle(vehicle.id);
    setShowVehicleDetails(true);
  };

  const handleAcknowledgeEvent = (event: SafetyEvent) => {
    useSwarmStore.getState().acknowledgeSafetyEvent(event.timestamp, 255); // 255 = GCS
  };

  return (
    <div className="h-screen w-full flex bg-gray-50">
      {/* Sidebar */}
      <aside className={cn(
        'fixed inset-y-0 left-0 z-40 bg-white border-r border-gray-200 transition-all duration-300',
        sidebarOpen ? 'w-72' : 'w-20'
      )}>
        <div className="flex flex-col h-full">
          {/* Header */}
          <div className="flex items-center justify-between p-4 border-b border-gray-200">
            {sidebarOpen && (
              <div className="flex items-center gap-2">
                <div className="w-8 h-8 rounded-lg bg-primary-600 flex items-center justify-center">
                  <Drone className="w-5 h-5 text-white" />
                </div>
                <span className="font-semibold text-gray-900">SwarmOps</span>
              </div>
            )}
            <button
              onClick={toggleSidebar}
              className="p-2 rounded-lg hover:bg-gray-100 text-gray-500 hover:text-gray-700"
              aria-label={sidebarOpen ? 'Collapse sidebar' : 'Expand sidebar'}
            >
              {sidebarOpen ? <ChevronLeft className="w-5 h-5" /> : <ChevronRight className="w-5 h-5" />}
            </button>
          </div>

          {/* Connection Status */}
          <div className="p-4 border-b border-gray-200">
            <div className="flex items-center gap-2">
              <div className={cn(
                'w-2 h-2 rounded-full',
                isConnected ? 'bg-success-500' : 'bg-danger-500'
              )} />
              <span className={cn(
                'text-sm font-medium',
                isConnected ? 'text-success-700' : 'text-danger-700'
              )}>
                {isConnected ? 'Connected' : 'Disconnected'}
              </span>
            </div>
            {connectionError && (
              <p className="mt-1 text-xs text-danger-600">{connectionError}</p>
            )}
            <button
              onClick={() => setConnected(!isConnected)}
              className="mt-2 w-full text-sm text-primary-600 hover:text-primary-700 flex items-center gap-1"
            >
              <Refresh className="w-4 h-4" />
              {isConnected ? 'Disconnect' : 'Reconnect'}
            </button>
          </div>

          {/* Fleet Summary */}
          <div className="p-4 border-b border-gray-200">
            {sidebarOpen ? (
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-500">Total</span>
                  <Badge variant="info">{vehicles.length}</Badge>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-500">Flying</span>
                  <Badge variant="success">{flyingVehicles.length}</Badge>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-500">Armed</span>
                  <Badge variant="warning">{armedVehicles.length}</Badge>
                </div>
                <Divider />
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-500">Formation</span>
                  <Badge variant={formation?.formation_locked ? 'success' : 'warning'}>
                    {formation?.formation_type || 'None'}
                  </Badge>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-500">Avg Error</span>
                  <span className="text-sm font-medium text-gray-900">
                    {formation?.avg_formation_error?.toFixed(1) || 0}m
                  </span>
                </div>
                <Divider />
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-500">Safety Events</span>
                  <Badge variant={criticalEvents.length > 0 ? 'danger' : 'success'}>
                    {unacknowledgedEvents.length}
                  </Badge>
                </div>
              </div>
            ) : (
              <div className="grid grid-cols-2 gap-2">
                <div className="text-center p-2 bg-gray-50 rounded-lg">
                  <div className="text-lg font-bold">{vehicles.length}</div>
                  <div className="text-xs text-gray-500">Total</div>
                </div>
                <div className="text-center p-2 bg-gray-50 rounded-lg">
                  <div className="text-lg font-bold text-success-600">{flyingVehicles.length}</div>
                  <div className="text-xs text-gray-500">Flying</div>
                </div>
              </div>
            )}
          </div>

          {/* Vehicle List */}
          <div className="flex-1 overflow-y-auto p-4">
            {sidebarOpen ? (
              <div className="space-y-2">
                <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2">Vehicles</h3>
                {vehicles.map(vehicle => (
                  <button
                    key={vehicle.id}
                    onClick={() => handleVehicleClick(vehicle)}
                    className={cn(
                      'w-full p-3 rounded-lg text-left transition-colors',
                      'hover:bg-gray-100',
                      useSwarmStore.getState().selectedVehicleId === vehicle.id
                        ? 'bg-primary-50 border-l-4 border-primary-500'
                        : ''
                    )}
                  >
                    <div className="flex items-center gap-2">
                      <Avatar
                        name={vehicle.callsign}
                        size="sm"
                        status={vehicle.is_flying ? 'online' : vehicle.arming_state === 'ARMED' ? 'busy' : 'offline'}
                      />
                      <div className="flex-1 min-w-0">
                        <div className="flex items-center gap-1">
                          <span className="text-sm font-medium text-gray-900 truncate">{vehicle.callsign}</span>
                          {vehicle.is_leader && <Badge variant="warning" size="xs">LEAD</Badge>}
                        </div>
                        <div className="flex items-center gap-2 text-xs text-gray-500">
                          <span className="flex items-center gap-1">
                            <Battery className="w-3 h-3" />
                            {vehicle.battery_remaining}%
                          </span>
                          <span className="flex items-center gap-1">
                            <Signal className="w-3 h-3" />
                            {Math.round(vehicle.link_quality * 100)}%
                          </span>
                        </div>
                      </div>
                      <div className={cn(
                        'w-2 h-2 rounded-full',
                        vehicle.is_flying ? 'bg-success-500' :
                        vehicle.arming_state === 'ARMED' ? 'bg-warning-500' : 'bg-gray-300'
                      )} />
                    </div>
                  </button>
                ))}
              </div>
            ) : (
              <div className="grid grid-cols-2 gap-2">
                {vehicles.map(vehicle => (
                  <button
                    key={vehicle.id}
                    onClick={() => handleVehicleClick(vehicle)}
                    className={cn(
                      'p-2 rounded-lg transition-colors',
                      'hover:bg-gray-100',
                      useSwarmStore.getState().selectedVehicleId === vehicle.id
                        ? 'bg-primary-50' : ''
                    )}
                    title={vehicle.callsign}
                  >
                    <div className="flex items-center justify-center gap-1">
                      <Avatar name={vehicle.callsign} size="xs" />
                      <span className="text-xs font-medium truncate">{vehicle.callsign}</span>
                    </div>
                  </button>
                ))}
              </div>
            )}
          </div>

          {/* Quick Actions */}
          <div className="p-4 border-t border-gray-200">
            {sidebarOpen ? (
              <div className="space-y-2">
                <Button variant="danger" className="w-full" onClick={() => useSwarmStore.getState().sendEstop(0)}>
                  <AlertTriangle className="w-4 h-4 mr-2" />
                  EMERGENCY STOP
                </Button>
                <Button variant="warning" className="w-full" onClick={() => useSwarmStore.getState().sendRTL(0)}>
                  <Home className="w-4 h-4 mr-2" />
                  RTL ALL
                </Button>
                <Button variant="secondary" className="w-full" onClick={() => useSwarmStore.getState().sendHold(0)}>
                  <Pause className="w-4 h-4 mr-2" />
                  HOLD ALL
                </Button>
              </div>
            ) : (
              <div className="grid grid-cols-3 gap-1">
                <Button variant="danger" size="sm" onClick={() => useSwarmStore.getState().sendEstop(0)} title="Emergency Stop">
                  <AlertTriangle className="w-4 h-4" />
                </Button>
                <Button variant="warning" size="sm" onClick={() => useSwarmStore.getState().sendRTL(0)} title="RTL All">
                  <Home className="w-4 h-4" />
                </Button>
                <Button variant="secondary" size="sm" onClick={() => useSwarmStore.getState().sendHold(0)} title="Hold All">
                  <Pause className="w-4 h-4" />
                </Button>
              </div>
            )}
          </div>
        </div>
      </aside>

      {/* Main Content */}
      <main className={cn('flex-1 flex flex-col', sidebarOpen ? 'ml-72' : 'ml-20')}>
        {/* Top Bar */}
        <header className="sticky top-0 z-30 bg-white border-b border-gray-200">
          <div className="flex items-center justify-between h-16 px-6">
            <div className="flex items-center gap-4">
              <h1 className="text-xl font-semibold text-gray-900">Live View</h1>
              <div className="flex items-center gap-2">
                <Badge variant={isConnected ? 'success' : 'danger'} size="sm">
                  {isConnected ? 'LIVE' : 'OFFLINE'}
                </Badge>
              </div>
            </div>

            <div className="flex items-center gap-4">
              {/* Map Layers */}
              <div className="flex items-center gap-2 border border-gray-200 rounded-lg p-1">
                <Tooltip content="Formation Paths">
                  <label className="flex items-center gap-1 cursor-pointer">
                    <input
                      type="checkbox"
                      checked={showFormationPaths}
                      onChange={e => setShowFormationPaths(e.target.checked)}
                      className="rounded border-gray-300 text-primary-600 focus:ring-primary-500"
                    />
                    <span className="text-sm text-gray-700">Formation</span>
                  </label>
                </Tooltip>
                <Tooltip content="Coverage Heatmap">
                  <label className="flex items-center gap-1 cursor-pointer">
                    <input
                      type="checkbox"
                      checked={showCoverage}
                      onChange={e => setShowCoverage(e.target.checked)}
                      className="rounded border-gray-300 text-primary-600 focus:ring-primary-500"
                    />
                    <span className="text-sm text-gray-700">Coverage</span>
                  </label>
                </Tooltip>
              </div>

              <Divider className="h-6" />

              {/* Time */}
              <div className="flex items-center gap-2 text-sm text-gray-500">
                <Clock className="w-4 h-4" />
                <span id="live-clock">--:--:--</span>
              </div>

              {/* Connection Indicator */}
              <div className="flex items-center gap-2">
                <div className={cn(
                  'w-2 h-2 rounded-full',
                  isConnected ? 'bg-success-500 animate-pulse' : 'bg-danger-500'
                )} />
              </div>

              {/* User Menu */}
              <div className="relative">
                <button className="flex items-center gap-2 p-2 rounded-lg hover:bg-gray-100">
                  <Avatar name="Operator" size="sm" status="online" />
                  <ChevronDown className="w-4 h-4 text-gray-500" />
                </button>
              </div>
            </div>
          </div>
        </header>

        {/* Map View */}
        <div className="flex-1 relative overflow-hidden">
          <DroneMap
            vehicles={vehicles}
            formation={formation}
            selectedVehicleId={useSwarmStore.getState().selectedVehicleId}
            onVehicleClick={handleVehicleClick}
            viewport={mapViewport}
            onViewportChange={setMapViewport}
            showFormationPaths={showFormationPaths}
            showCoverage={showCoverage}
            className="absolute inset-0"
          />

          {/* Map Controls Overlay */}
          <div className="absolute top-4 right-4 z-20 flex flex-col gap-2">
            <Tooltip content="Center on Formation">
              <Button variant="ghost" size="sm" onClick={() => {
                if (formation && formation.leader_id) {
                  const leader = vehicles.find(v => v.id === formation.leader_id);
                  if (leader) setMapViewport({ center: [leader.position.lon, leader.position.lat], zoom: 16 });
                }
              }}>
                <MapPinIcon className="w-5 h-5" />
              </Button>
            </Tooltip>
            <Tooltip content="Center on Selected">
              <Button variant="ghost" size="sm" onClick={() => {
                const selected = vehicles.find(v => v.id === useSwarmStore.getState().selectedVehicleId);
                if (selected) setMapViewport({ center: [selected.position.lon, selected.position.lat], zoom: 18 });
              }}>
                <MapPin className="w-5 h-5" />
              </Button>
            </Tooltip>
            <Tooltip content="Reset View">
              <Button variant="ghost" size="sm" onClick={() => setMapViewport({ center: [-122.4194, 37.7749], zoom: 14 })}>
                <RotateCcw className="w-5 h-5" />
              </Button>
            </Tooltip>
            <Tooltip content="Fullscreen">
              <Button variant="ghost" size="sm" onClick={() => {
                const el = document.querySelector('.map-container') as HTMLElement;
                el?.requestFullscreen?.();
              }}>
                <Fullscreen className="w-5 h-5" />
              </Button>
            </Tooltip>
          </div>

          {/* Bottom Status Bar */}
          <div className="absolute bottom-0 left-0 right-0 bg-white/90 backdrop-blur-sm border-t border-gray-200 p-3">
            <div className="max-w-7xl mx-auto flex flex-wrap items-center justify-between gap-4">
              <div className="flex items-center gap-4 text-sm">
                <span className="flex items-center gap-1">
                  <MapPin className="w-4 h-4 text-gray-400" />
                  <span className="font-mono">
                    {mapViewport.center[1].toFixed(6)}, {mapViewport.center[0].toFixed(6)}
                  </span>
                </span>
                <span className="flex items-center gap-1">
                  <Zap className="w-4 h-4 text-gray-400" />
                  <span>Zoom: {mapViewport.zoom.toFixed(1)}</span>
                </span>
                <span className="flex items-center gap-1">
                  <RotateCcw className="w-4 h-4 text-gray-400" />
                  <span>Bearing: {mapViewport.bearing.toFixed(0)}°</span>
                </span>
              </div>
              <div className="flex items-center gap-4 text-sm">
                <span className={cn('flex items-center gap-1', formation?.formation_locked ? 'text-success-600' : 'text-warning-600')}>
                  <Shield className="w-4 h-4" />
                  {formation?.formation_type || 'NO FORMATION'}
                </span>
                <span className="flex items-center gap-1">
                  <AlertTriangle className="w-4 h-4 text-gray-400" />
                  <span>Avg Error: {formation?.avg_formation_error?.toFixed(1) || 0}m</span>
                </span>
                <span className="flex items-center gap-1">
                  <Bell className={cn('w-4 h-4', unacknowledgedEvents.length > 0 ? 'text-danger-500' : 'text-gray-400')} />
                  <span className={cn(unacknowledgedEvents.length > 0 ? 'text-danger-600 font-medium' : '')}>
                    {unacknowledgedEvents.length} alerts
                  </span>
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* Right Panel - Vehicle Details / Safety Events */}
        {rightPanelOpen && (
          <aside className={cn(
            'fixed inset-y-0 right-0 z-40 bg-white border-l border-gray-200 transition-all duration-300 shadow-xl',
            'w-96'
          )}>
            <div className="flex flex-col h-full">
              <div className="flex items-center justify-between p-4 border-b border-gray-200">
                <h3 className="font-semibold text-gray-900">
                  {showVehicleDetails && selectedVehicle ? `Vehicle: ${selectedVehicle.callsign}` : 'Safety Events'}
                </h3>
                <button onClick={() => { setShowVehicleDetails(false); toggleRightPanel(); }} className="p-2 rounded-lg hover:bg-gray-100">
                  <X className="w-5 h-5 text-gray-500" />
                </button>
              </div>

              <div className="flex-1 overflow-y-auto p-4">
                {showVehicleDetails && selectedVehicle ? (
                  <div className="space-y-4">
                    {/* Vehicle Header */}
                    <Card>
                      <CardContent className="p-4">
                        <div className="flex items-center justify-between mb-4">
                          <div>
                            <Avatar name={selectedVehicle.callsign} size="lg" status={selectedVehicle.is_flying ? 'online' : 'offline'} />
                            <div className="mt-2">
                              <h4 className="text-lg font-semibold">{selectedVehicle.callsign}</h4>
                              <div className="flex items-center gap-2 text-sm text-gray-500">
                                <Badge variant={statusColors[selectedVehicle.arming_state as keyof typeof statusColors] || 'default'}>
                                  {selectedVehicle.arming_state}
                                </Badge>
                                {selectedVehicle.is_leader && <Badge variant="warning">LEADER</Badge>}
                                <span className="text-gray-400">ID: #{selectedVehicle.id}</span>
                              </div>
                            </div>
                          </div>
                          <div className="flex items-center gap-4">
                            <div className="flex items-center gap-1 text-sm">
                              <Battery className="w-4 h-4" />
                              <span className={cn('font-medium', getBatteryColor(selectedVehicle.battery_remaining))}>
                                {selectedVehicle.battery_remaining}%
                              </span>
                            </div>
                            <div className="flex items-center gap-1 text-sm">
                              <Signal className="w-4 h-4" />
                              <span className={cn('font-medium', getLinkQualityColor(selectedVehicle.link_quality))}>
                                {Math.round(selectedVehicle.link_quality * 100)}%
                              </span>
                            </div>
                            <div className="flex items-center gap-1 text-sm">
                              <Wifi className="w-4 h-4" />
                              <span className={cn('font-medium', getRSSIColor(selectedVehicle.rssi_dbm))}>
                                {selectedVehicle.rssi_dbm}dBm
                              </span>
                            </div>
                          </div>
                        </CardContent>
                    </Card>

                    {/* Position & Attitude */}
                    <div className="grid grid-cols-2 gap-3">
                      <Card>
                        <CardHeader>Position</CardHeader>
                        <CardContent>
                          <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                              <span className="text-gray-500">Lat/Lon</span>
                              <span className="font-mono font-medium">
                                {selectedVehicle.position.lat.toFixed(6)}, {selectedVehicle.position.lon.toFixed(6)}
                              </span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Altitude</span>
                              <span className="font-medium">{formatAltitude(selectedVehicle.position.alt || 0)}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Ground Speed</span>
                              <span className="font-medium">{formatSpeed(selectedVehicle.ground_speed)}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Vertical Speed</span>
                              <span className="font-medium">{selectedVehicle.vertical_speed >= 0 ? '+' : ''}{selectedVehicle.vertical_speed.toFixed(1)} m/s</span>
                            </div>
                          </div>
                        </CardContent>
                      </Card>
                      <Card>
                        <CardHeader>Attitude</CardHeader>
                        <CardContent>
                          <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                              <span className="text-gray-500">Roll</span>
                              <span className="font-medium">{selectedVehicle.attitude.roll.toFixed(1)}°</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Pitch</span>
                              <span className="font-medium">{selectedVehicle.attitude.pitch.toFixed(1)}°</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Yaw</span>
                              <span className="font-medium">{selectedVehicle.attitude.yaw.toFixed(1)}°</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Heading</span>
                              <span className="font-medium">{selectedVehicle.heading.toFixed(1)}°</span>
                            </div>
                          </div>
                        </CardContent>
                      </Card>
                    </div>

                    {/* Battery Details */}
                    <Card>
                      <CardHeader>Battery</CardHeader>
                      <CardContent>
                        <div className="grid grid-cols-2 gap-3">
                          <div>
                            <div className="text-sm text-gray-500">Voltage</div>
                            <div className="font-medium">{selectedVehicle.battery_voltage.toFixed(1)}V</div>
                          </div>
                          <div>
                            <div className="text-sm text-gray-500">Current</div>
                            <div className="font-medium">{selectedVehicle.battery_current.toFixed(1)}A</div>
                          </div>
                          <div>
                            <div className="text-sm text-gray-500">Consumed</div>
                            <div className="font-medium">{selectedVehicle.battery_consumed.toFixed(0)}mAh</div>
                          </div>
                          <div>
                            <div className="text-sm text-gray-500">Cells</div>
                            <div className="font-medium">{selectedVehicle.cell_count}S</div>
                          </div>
                        </div>
                        {selectedVehicle.cell_voltages.length > 0 && (
                          <div className="mt-3">
                            <div className="text-sm text-gray-500 mb-1">Cell Voltages</div>
                            <div className="flex gap-1 overflow-x-auto pb-1">
                              {selectedVehicle.cell_voltages.map((v, i) => (
                                <div key={i} className="w-16 px-2 py-1 text-xs text-center bg-gray-50 rounded">
                                  C{i + 1}: {v.toFixed(2)}V
                                </div>
                              ))}
                            </div>
                          </div>
                        </CardContent>
                      </Card>

                    {/* GPS & Estimator */}
                    <div className="grid grid-cols-2 gap-3">
                      <Card>
                        <CardHeader>GPS</CardHeader>
                        <CardContent>
                          <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                              <span className="text-gray-500">Satellites</span>
                              <span className="font-medium">{selectedVehicle.gps_satellites}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">HDOP</span>
                              <span className="font-medium">{selectedVehicle.gps_hdop.toFixed(1)}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Fix Type</span>
                              <span className="font-medium">{selectedVehicle.gps_fix_type}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Status</span>
                              <Badge variant={selectedVehicle.gps_healthy ? 'success' : 'danger'}>
                                {selectedVehicle.gps_healthy ? 'Healthy' : 'Degraded'}
                              </Badge>
                            </div>
                          </div>
                        </CardContent>
                      </Card>
                      <Card>
                        <CardHeader>Estimator</CardHeader>
                        <CardContent>
                          <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                              <span className="text-gray-500">Innovation</span>
                              <span className="font-medium">{selectedVehicle.ekf_innovation.toFixed(3)}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-sm text-gray-500">Resets</span>
                              <span className="font-medium">{selectedVehicle.ekf_reset_count}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-500">Status</span>
                              <Badge variant={selectedVehicle.ekf_healthy ? 'success' : 'danger'}>
                                {selectedVehicle.ekf_healthy ? 'Healthy' : 'Degraded'}
                              </Badge>
                            </div>
                          </div>
                        </CardContent>
                      </Card>
                    </div>

                    {/* Communication */}
                    <Card>
                      <CardHeader>Communication</CardHeader>
                      <CardContent>
                        <div className="grid grid-cols-2 gap-3">
                          <div>
                            <div className="text-sm text-gray-500">Link Quality</div>
                            <div className="flex items-center gap-2">
                              <div className="font-medium" style={{ color: getLinkQualityColor(selectedVehicle.link_quality) }}>
                                {Math.round(selectedVehicle.link_quality * 100)}%
                              </div>
                              <div className="w-24 h-2 bg-gray-200 rounded-full overflow-hidden">
                                <div className="h-full bg-primary-500 rounded-full transition-all" style={{ width: `${selectedVehicle.link_quality * 100}%` }} />
                              </div>
                            </div>
                          </div>
                          <div>
                            <div className="text-sm text-gray-500">RSSI</div>
                            <div className="font-medium" style={{ color: getRSSIColor(selectedVehicle.rssi_dbm) }}>
                              {selectedVehicle.rssi_dbm}dBm
                            </div>
                          </div>
                          <div>
                            <div className="text-sm text-gray-500">SNR</div>
                            <div className="font-medium">{selectedVehicle.snr_db.toFixed(1)}dB</div>
                          </div>
                          <div>
                            <div className="text-sm text-gray-500">Latency</div>
                            <div className="font-medium">{selectedVehicle.latency_ms.toFixed(0)}ms</div>
                          </div>
                        </div>
                      </CardContent>
                    </Card>

                  </div>
                ) : (
                  <div className="space-y-4">
                    {/* Safety Events Summary */}
                    <div className="grid grid-cols-3 gap-3">
                      <Card variant="outlined">
                        <CardContent className="p-4 text-center">
                          <div className="text-3xl font-bold text-danger-600">{criticalEvents.length}</div>
                          <div className="text-sm text-gray-500">Critical</div>
                        </CardContent>
                      </Card>
                      <Card variant="outlined">
                        <CardContent className="p-4 text-center">
                          <div className="text-3xl font-bold text-warning-600">
                            {safetyEvents.filter(e => e.severity === 'ERROR').length}
                          </div>
                          <div className="text-sm text-gray-500">Errors</div>
                        </CardContent>
                      </Card>
                      <Card variant="outlined">
                        <CardContent className="p-4 text-center">
                          <div className="text-3xl font-bold text-primary-600">{unacknowledgedEvents.length}</div>
                          <div className="text-sm text-gray-500">Unacked</div>
                        </CardContent>
                      </Card>
                    </div>

                    {/* Safety Events Table */}
                    <Card>
                      <CardHeader className="flex items-center justify-between">
                        <h3>Recent Safety Events</h3>
                        <Badge variant={unacknowledgedEvents.length > 0 ? 'danger' : 'success'}>
                          {unacknowledgedEvents.length} Unacked
                        </Badge>
                      </CardHeader>
                      <CardContent>
                        {safetyEvents.length === 0 ? (
                          <div className="text-center py-8 text-gray-500">No safety events</div>
                        ) : (
                          <Table
                            data={safetyEvents.slice(0, 50)}
                            columns={[
                              { key: 'timestamp', header: 'Time', render: (e: SafetyEvent) => formatTimeAgo(e.timestamp) },
                              { key: 'source_id', header: 'Source', render: (e: SafetyEvent) => `#${e.source_id}` },
                              { key: 'event_type', header: 'Type' },
                              { key: 'severity', header: 'Severity', render: (e: SafetyEvent) => (
                                <Badge variant={getSeverityBadgeColor(e.severity)}>{e.severity}</Badge>
                              )},
                              { key: 'description', header: 'Description' },
                              { key: 'acknowledged', header: 'Status', render: (e: SafetyEvent) => (
                                <div className="flex items-center gap-2">
                                  <Badge variant={e.acknowledged ? 'success' : 'danger'} size="sm">
                                    {e.acknowledged ? 'Acked' : 'Pending'}
                                  </Badge>
                                  {!e.acknowledged && (
                                    <Button size="sm" variant="secondary" onClick={() => handleAcknowledgeEvent(e)}>
                                      Ack
                                    </Button>
                                  )}
                                </div>
                              )},
                            ]}
                            keyExtractor={(e: SafetyEvent) => e.timestamp}
                            onRowClick={(e: SafetyEvent) => setSelectedSafetyEvent(e)}
                          />
                        </CardContent>
                      </Card>
                    </Card>
                  </div>
                )}
              </div>
            </div>
          </aside>
        )}

        {/* Vehicle Details Modal */}
        {showVehicleDetails && selectedVehicle && !rightPanelOpen && (
          <Modal
            isOpen={true}
            onClose={() => setShowVehicleDetails(false)}
            title={`${selectedVehicle.callsign} Details`}
            size="xl"
          >
            <div className="max-h-[70vh] overflow-y-auto">
              {/* Same content as right panel but in modal */}
              <p className="text-center text-gray-500 py-8">Use right panel for full details</div>
            </div>
          </Modal>
        )}

        {/* Safety Event Detail Modal */}
        {selectedSafetyEvent && (
          <Modal
            isOpen={true}
            onClose={() => setSelectedSafetyEvent(null)}
            title="Safety Event Details"
            size="lg"
          >
            <div className="space-y-4">
              <div className="flex items-center gap-2">
                <Badge variant={getSeverityBadgeColor(selectedSafetyEvent.severity)}>
                  {selectedSafetyEvent.severity}
                </Badge>
                <Badge variant={selectedSafetyEvent.acknowledged ? 'success' : 'danger'}>
                  {selectedSafetyEvent.acknowledged ? 'Acknowledged' : 'Pending'}
                </Badge>
              </div>
              <div className="grid grid-cols-2 gap-4 text-sm">
                <div><span className="text-gray-500">Time</span><div className="font-medium">{formatTimestamp(selectedSafetyEvent.timestamp)}</div></div>
                <div><span className="text-gray-500">Source</span><div className="font-medium">Drone #{selectedSafetyEvent.source_id}</div></div>
                <div><span className="text-gray-500">Type</span><div className="font-medium">{selectedSafetyEvent.event_type}</div></div>
                <div><span className="text-gray-500">Severity</span><div className="font-medium">{selectedSafetyEvent.severity}</div></div>
              </div>
              <div>
                <span className="text-gray-500 text-sm">Description</span>
                <p className="mt-1">{selectedSafetyEvent.description}</p>
              </div>
              {selectedSafetyEvent.details && Object.keys(selectedSafetyEvent.details).length > 0 && (
                <div>
                  <span className="text-gray-500 text-sm">Details</span>
                  <pre className="mt-1 p-2 bg-gray-100 rounded text-xs overflow-auto max-h-40">
                    {JSON.stringify(selectedSafetyEvent.details, null, 2)}
                  </pre>
                </div>
              )}
              {!selectedSafetyEvent.acknowledged && (
                <Button variant="danger" className="w-full" onClick={() => handleAcknowledgeEvent(selectedSafetyEvent!)}>Acknowledge Event</Button>
              )}
            </div>
          </Modal>
        )}

        {/* Clock Update */}
        <script dangerouslySetInnerHTML={{
          __html: `
            function updateClock() {
              const now = new Date();
              const timeString = now.toLocaleTimeString();
              const el = document.getElementById('live-clock');
              if (el) el.textContent = timeString;
            }
            setInterval(updateClock, 1000);
            updateClock();
          `
        }} />
      </main>
    </div>
  );
};

// Need to import useSwarmStore for direct access
import { useSwarmStore } from '@/store/swarmStore';
import { Clock } from 'lucide-react';