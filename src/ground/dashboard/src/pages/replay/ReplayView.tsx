// Replay page - Flight replay and analysis

import React, { useState, useEffect, useCallback } from 'react';
import { useSwarmStore, useVehiclesArray } from '@/hooks';
import { DroneMap } from '@/components/map';
import { TimeSeriesChart, BatteryChart, LinkQualityChart, FormationErrorCDFChart } from '@/components/charts';
import { Card, CardHeader, CardContent, Button, Badge, Select, Input, Slider, Tooltip } from '@/components/common';
import { Play, Pause, Stop, FastForward, Rewind, SkipBack, SkipForward, RotateCcw, Download, Upload, Calendar, Clock, MapPin, Layers, Filter, ChevronLeft, ChevronRight, Minimize, Maximize, RotateCcw as Refresh } from 'lucide-react';
import { cn, formatTimeAgo, formatTimestamp, formatDuration, formatDistance, formatSpeed, formatAltitude } from '@/utils/helpers';
import type { DroneState, TimeSeriesData } from '@/types';

interface ReplayDataPoint {
  timestamp: number;
  vehicles: Record<number, {
    position: { lat: number; lon: number; alt: number };
    attitude: { roll: number; pitch: number; yaw: number };
    velocity: { vx: number; vy: number; vz: number };
    battery: number;
    mode: string;
  }>;
  formation?: any;
  events: any[];
}

export const ReplayView: React.FC = () => {
  const vehicles = useVehiclesArray();
  const [replayData, setReplayData] = useState<ReplayDataPoint[]>([]);
  const [currentTimeIndex, setCurrentTimeIndex] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [selectedLog, setSelectedLog] = useState<string>('');
  const [dateRange, setDateRange] = useState<[Date, Date]>([new Date(Date.now() - 86400000), new Date()]);
  const [showFormation, setShowFormation] = useState(true);
  const [showTrajectories, setShowTrajectories] = useState(true);
  const [selectedVehicleId, setSelectedVehicleId] = useState<number | null>(null);
  const [mapViewport, setMapViewport] = useState({ center: [-122.4194, 37.7749], zoom: 14, bearing: 0, pitch: 0 });
  const [timeSeriesData, setTimeSeriesData] = useState<TimeSeriesData[]>([]);

  // Load available logs
  const availableLogs = useSwarmStore((state) => state.listAvailableLogs?.() || []);

  // Playback loop
  useEffect(() => {
    if (!isPlaying || !replayData.length) return;

    const interval = setInterval(() => {
      setCurrentTimeIndex(prev => {
        const next = prev + Math.ceil(playbackSpeed);
        if (next >= replayData.length) {
          setIsPlaying(false);
          return replayData.length - 1;
        }
        return next;
      });
    }, 100);

    return () => clearInterval(interval);
  }, [isPlaying, playbackSpeed, replayData.length]);

  // Update time series data when time index changes
  useEffect(() => {
    if (!replayData.length) return;

    const currentData = replayData[currentTimeIndex];
    if (!currentData) return;

    // Build time series for charts
    const batterySeries: TimeSeriesData[] = [];
    const linkQualitySeries: TimeSeriesData[] = [];
    const altitudeSeries: TimeSeriesData[] = [];

    Object.entries(currentData.vehicles).forEach(([vid, v]) => {
      batterySeries.push({
        label: `Drone ${vid}`,
        data: [{ timestamp: currentData.timestamp, value: v.battery }],
        color: `hsl(${vid * 60}, 70%, 50%)`,
      });
    });

    setTimeSeriesData([...batterySeries, ...linkQualitySeries, ...altitudeSeries]);
  }, [currentTimeIndex, replayData]);

  const handleLoadLog = useCallback(async (logId: string) => {
    // In real implementation, this would load the log file and parse it
    setSelectedLog(logId);
    // Mock data for demonstration
    const mockData: ReplayDataPoint[] = [];
    const startTime = Date.now() - 3600000; // 1 hour ago
    for (let i = 0; i < 3600; i += 10) { // Every 10 seconds
      const vehiclesData: Record<number, any> = {};
      vehicles.forEach((v, idx) => {
        const t = i / 3600 * Math.PI * 4;
        vehiclesData[v.id] = {
          position: {
            lat: 37.7749 + Math.sin(t + idx) * 0.001,
            lon: -122.4194 + Math.cos(t + idx) * 0.001,
            alt: 50 + Math.sin(t * 2 + idx) * 10,
          },
          attitude: { roll: Math.sin(t + idx) * 5, pitch: Math.cos(t + idx) * 5, yaw: t * 10 + idx * 30 },
          velocity: { vx: Math.cos(t) * 5, vy: Math.sin(t) * 5, vz: 0 },
          battery: 100 - i / 36,
          mode: 'AUTO',
        };
      });
      mockData.push({
        timestamp: startTime + i * 1000,
        vehicles: vehiclesData,
        events: [],
      });
    }
    setReplayData(mockData);
    setCurrentTimeIndex(0);
  }, [vehicles]);

  const handlePlayPause = () => setIsPlaying(!isPlaying);
  const handleStop = () => { setIsPlaying(false); setCurrentTimeIndex(0); };
  const handleSkipForward = () => setCurrentTimeIndex(prev => Math.min(prev + 300, replayData.length - 1));
  const handleSkipBack = () => setCurrentTimeIndex(prev => Math.max(prev - 300, 0));
  const handleSpeedChange = (speed: number) => setPlaybackSpeed(speed);

  const currentData = replayData[currentTimeIndex];
  const progress = replayData.length ? (currentTimeIndex / (replayData.length - 1)) * 100 : 0;
  const currentTime = currentData ? new Date(currentData.timestamp) : null;

  return (
    <div className="h-screen w-full flex flex-col bg-gray-50">
      {/* Top Toolbar */}
      <header className="bg-white border-b border-gray-200 p-4">
        <div className="max-w-full mx-auto flex flex-wrap items-center justify-between gap-4">
          <div className="flex items-center gap-4">
            <h1 className="text-xl font-semibold text-gray-900">Flight Replay</h1>
            
            <Select
              value={selectedLog}
              onChange={handleLoadLog}
              options={availableLogs.map(log => ({ value: log.log_id, label: `${log.callsign} - ${formatTimestamp(log.flight_start)} (${log.duration_s.toFixed(0)}s)` }))}
              placeholder="Select flight log..."
              className="w-80"
            />
            
            <div className="flex items-center gap-2">
              <Calendar className="w-4 h-4 text-gray-400" />
              <Input
                type="date"
                value={dateRange[0].toISOString().split('T')[0]}
                onChange={e => setDateRange([new Date(e.target.value), dateRange[1]])}
                className="w-40"
              />
              <span className="text-gray-400">to</span>
              <Input
                type="date"
                value={dateRange[1].toISOString().split('T')[0]}
                onChange={e => setDateRange([dateRange[0], new Date(e.target.value)])}
                className="w-40"
              />
            </div>
          </div>

          <div className="flex items-center gap-4">
            {/* Playback Controls */}
            <div className="flex items-center gap-2 bg-gray-50 rounded-lg p-2">
              <Tooltip content="Skip Back (30s)">
                <Button variant="ghost" size="sm" onClick={handleSkipBack} disabled={!replayData.length}>
                  <SkipBack className="w-4 h-4" />
                </Button>
              </Tooltip>
              <Tooltip content="Rewind (2x)">
                <Button variant="ghost" size="sm" onClick={() => handleSpeedChange(0.5)} disabled={!replayData.length}>
                  <Rewind className="w-4 h-4" />
                </Button>
              </Tooltip>
              <Tooltip content={isPlaying ? 'Pause' : 'Play'}>
                <Button 
                  variant="primary" 
                  size="sm" 
                  onClick={handlePlayPause} 
                  disabled={!replayData.length}
                  className="w-10 h-10 rounded-full"
                >
                  {isPlaying ? <Pause className="w-5 h-5" /> : <Play className="w-5 h-5" />}
                </Button>
              </Tooltip>
              <Tooltip content="Fast Forward (2x)">
                <Button variant="ghost" size="sm" onClick={() => handleSpeedChange(2)} disabled={!replayData.length}>
                  <FastForward className="w-4 h-4" />
                </Button>
              </Tooltip>
              <Tooltip content="Skip Forward (30s)">
                <Button variant="ghost" size="sm" onClick={handleSkipForward} disabled={!replayData.length}>
                  <SkipForward className="w-4 h-4" />
                </Button>
              </Tooltip>
              <Tooltip content="Stop">
                <Button variant="ghost" size="sm" onClick={handleStop} disabled={!replayData.length}>
                  <Stop className="w-4 h-4" />
                </Button>
              </Tooltip>
            </div>

            <div className="flex items-center gap-2 ml-4">
              <span className="text-sm text-gray-500">Speed:</span>
              <Select
                value={playbackSpeed.toString()}
                onChange={e => handleSpeedChange(parseFloat(e.target.value))}
                options={[
                  { value: '0.25', label: '0.25x' },
                  { value: '0.5', label: '0.5x' },
                  { value: '1', label: '1x' },
                  { value: '2', label: '2x' },
                  { value: '4', label: '4x' },
                  { value: '8', label: '8x' },
                ]}
                className="w-24"
              />
            </div>

            {/* Timeline */}
            <div className="flex-1 mx-4 flex items-center">
              <div className="w-full h-2 bg-gray-200 rounded-full relative" onClick={e => {
                if (!replayData.length) return;
                const rect = (e.currentTarget as HTMLDivElement).getBoundingClientRect();
                const clickPos = (e.clientX - rect.left) / rect.width;
                setCurrentTimeIndex(Math.floor(clickPos * (replayData.length - 1)));
              }}>
                <div 
                  className="h-full bg-primary-600 rounded-full transition-all duration-100"
                  style={{ width: `${progress}%` }}
                />
                <div 
                  className="absolute top-1/2 w-3 h-3 bg-white border-2 border-primary-600 rounded-full -translate-x-1/2 -translate-y-1/2 shadow-lg"
                  style={{ left: `${progress}%` }}
                />
              </div>
              <span className="text-sm text-gray-500 ml-2 w-20 text-right">
                {currentTime ? formatTimestamp(currentTime.timestamp) : '--:--:--'}
              </span>
            </div>

            {/* Time Display */}
            <div className="flex items-center gap-2 text-sm text-gray-500">
              <Clock className="w-4 h-4" />
              <span>{currentTime ? formatTimestamp(currentTime.timestamp) : '--:--:--'}</span>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <div className="flex-1 flex overflow-hidden">
        {/* Left Panel - Vehicle List & Controls */}
        <aside className="w-64 bg-white border-r border-gray-200 flex flex-col">
          <div className="p-4 border-b border-gray-200">
            <h3 className="font-semibold text-gray-900 mb-3">Vehicles</h3>
            <div className="space-y-2">
              {vehicles.map(v => (
                <label key={v.id} className="flex items-center gap-2 cursor-pointer p-2 rounded-lg hover:bg-gray-50">
                  <input
                    type="checkbox"
                    checked={true}
                    onChange={() => {}}
                    className="rounded border-gray-300 text-primary-600 focus:ring-primary-500"
                  />
                  <Avatar name={v.callsign} size="sm" status={v.is_flying ? 'online' : 'offline'} />
                  <div className="flex-1 min-w-0">
                    <div className="text-sm font-medium truncate">{v.callsign}</div>
                    <div className="text-xs text-gray-500">{v.flight_mode}</div>
                  </div>
                </label>
              ))}
            </div>
          </div>

          <div className="p-4 border-b border-gray-200">
            <h3 className="font-semibold text-gray-900 mb-3">Layers</h3>
            <div className="space-y-2">
              <label className="flex items-center gap-2">
                <input type="checkbox" checked={showFormation} onChange={e => setShowFormation(e.target.checked)} className="rounded border-gray-300 text-primary-600" />
                <span className="text-sm">Formation Paths</span>
              </label>
              <label className="flex items-center gap-2">
                <input type="checkbox" checked={showTrajectories} onChange={e => setShowTrajectories(e.target.checked)} className="rounded border-gray-300 text-primary-600" />
                <span className="text-sm">Trajectories</span>
              </label>
            </div>
          </div>

          <div className="p-4 border-b border-gray-200">
            <h3 className="font-semibold text-gray-900 mb-3">Playback Speed</h3>
            <Slider
              value={playbackSpeed}
              onChange={handleSpeedChange}
              min={0.25}
              max={8}
              step={0.25}
              marks={[
                { value: 0.25, label: '0.25x' },
                { value: 0.5, label: '0.5x' },
                { value: 1, label: '1x' },
                { value: 2, label: '2x' },
                { value: 4, label: '4x' },
                { value: 8, label: '8x' },
              ]}
            />
            <p className="text-center text-sm text-gray-500 mt-1">{playbackSpeed}x</p>
          </div>

          <div className="p-4">
            <Button variant="secondary" className="w-full" onClick={() => {}}>
              <Download className="w-4 h-4 mr-2" />
              Export Replay Data
            </Button>
          </div>
        </aside>

        {/* Main Map View */}
        <div className="flex-1 relative">
          <DroneMap
            vehicles={vehicles}
            viewport={mapViewport}
            onViewportChange={setMapViewport}
            showFormationPaths={showFormation}
            className="absolute inset-0"
          />

          {/* Current Vehicle Info Overlay */}
          {currentData && (
            <div className="absolute top-4 left-4 right-4 z-20 flex flex-wrap gap-2">
              {Object.entries(currentData.vehicles).map(([vid, v]) => (
                <Card key={vid} className="flex-1 min-w-[200px] max-w-[300px]">
                  <CardContent className="p-3">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <Avatar name={`Drone ${vid}`} size="sm" />
                        <div>
                          <div className="font-medium">Drone {vid}</div>
                          <div className="text-xs text-gray-500">{v.mode}</div>
                        </div>
                      </div>
                      <div className="text-right text-xs text-gray-500">
                        <div>Batt: {v.battery.toFixed(0)}%</div>
                        <div>Alt: {v.position.alt.toFixed(0)}m</div>
                      </div>
                    </div>
                  </Card>
              ))}
            </div>
          )}

          {/* Bottom Timeline Charts */}
          <div className="absolute bottom-0 left-0 right-0 h-64 bg-white border-t border-gray-200 p-2 overflow-x-auto">
            <div className="flex gap-4 min-w-max" style={{ minWidth: '1200px' }}>
              <Card className="w-96">
                <CardHeader>Battery History</CardHeader>
                <CardContent>
                  <BatteryChart data={[]} height={150} />
                </CardContent>
              </Card>
              <Card className="w-96">
                <CardHeader>Link Quality</CardHeader>
                <CardContent>
                  <LinkQualityChart data={[]} height={150} />
                </CardContent>
              </Card>
              <Card className="w-96">
                <CardHeader>Formation Error</CardHeader>
                <CardContent>
                  <FormationErrorCDFChart cdfX={[]} cdfY={[]} height={150} />
                </CardContent>
              </Card>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

import { Slider } from '@/components/common';
import { Avatar } from '@/components/common';