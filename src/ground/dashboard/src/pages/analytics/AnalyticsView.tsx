// Analytics page - Post-flight analysis and fleet analytics

import React, { useState, useEffect, useMemo } from 'react';
import { useSwarmStore, useVehiclesArray } from '@/hooks';
import { TimeSeriesChart, BatteryChart, LinkQualityChart, FormationErrorCDFChart, DetectionDistributionChart, RadarChart } from '@/components/charts';
import { Card, CardHeader, CardContent, Badge, Button, Select, Input, Table, Tabs, TabList, Tab, TabPanel, Tooltip, Avatar } from '@/components/common';
import { 
  TrendingUp, TrendingDown, Activity, BarChart3, PieChart, 
  Download, Upload, Filter, Calendar, Clock, AlertTriangle, 
  CheckCircle, XCircle, Eye, EyeOff, Settings, RefreshCw,
  Zap, Signal, Battery, Shield, MapPin, Users, 
  ArrowUpRight, ArrowDownRight, Minus, Plus,
  FileText, Image, Video, Cpu, HardDrive, Network,
  ChevronLeft, ChevronRight
} from 'lucide-react';
import { cn, formatPercentage, formatDuration, formatDistance, formatSpeed, formatAltitude, formatTimeAgo, getSeverityBadgeColor, generateChartColors } from '@/utils/helpers';
import type { DroneState, TimeSeriesData, FlightMetrics, AnalysisReport } from '@/types';

interface FleetMetrics {
  totalFlights: number;
  totalFlightTime: number;
  totalDistance: number;
  avgFormationError: number;
  totalDetections: number;
  totalSafetyEvents: number;
  criticalEvents: number;
  avgBatteryEfficiency: number;
  avgLinkQuality: number;
  fleetAvailability: number;
}

export const AnalyticsView: React.FC = () => {
  const vehicles = useVehiclesArray();
  const [selectedReport, setSelectedReport] = useState<AnalysisReport | null>(null);
  const [reports, setReports] = useState<AnalysisReport[]>([]);
  const [dateRange, setDateRange] = useState<[Date, Date]>([new Date(Date.now() - 30 * 86400000), new Date()]);
  const [activeTab, setActiveTab] = useState<'fleet' | 'vehicle' | 'formation' | 'ai' | 'qos' | 'reports'>('fleet');
  const [selectedVehicleId, setSelectedVehicleId] = useState<number | null>(null);
  const [showReportModal, setShowReportModal] = useState(false);
  const [generatingReport, setGeneratingReport] = useState(false);

  // Mock data for demonstration
  const fleetMetrics = useMemo<FleetMetrics>(() => ({
    totalFlights: 156,
    totalFlightTime: 487.5,
    totalDistance: 12450.3,
    avgFormationError: 1.23,
    totalDetections: 8942,
    totalSafetyEvents: 234,
    criticalEvents: 12,
    avgBatteryEfficiency: 87.3,
    avgLinkQuality: 91.7,
    fleetAvailability: 94.2,
  }), []);

  const vehicleComparisonData = useMemo(() => {
    return vehicles.map(v => ({
      label: v.callsign,
      data: [
        v.battery_remaining,
        v.link_quality * 100,
        Math.max(0, 100 - v.ekf_innovation * 100),
        v.gps_satellites / 20 * 100,
        v.is_flying ? 100 : 0,
      ],
      color: `hsl(${v.id * 60}, 70%, 50%)`,
    }));
  }, [vehicles]);

  const radarLabels = ['Battery', 'Link Quality', 'Estimator', 'GPS', 'Availability'];

  const detectionDistribution = useMemo(() => ({
    'Person': 2341,
    'Vehicle': 1876,
    'Power Line': 1234,
    'Building': 987,
    'Tree': 654,
    'Drone': 432,
    'Bird': 321,
    'Other': 1110,
  }), []);

  const fleetBatteryTrend = useMemo<TimeSeriesData[]>(() => ([
    { label: 'Fleet Avg Battery', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 85 + Math.random() * 10 })), color: '#3b82f6' },
    { label: 'Min Battery', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 40 + Math.random() * 20 })), color: '#ef4444' },
  ]), []);

  const formationErrorTrend = useMemo<TimeSeriesData[]>(() => ([
    { label: 'Mean Error', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 1.0 + Math.random() * 1.5 })), color: '#3b82f6' },
    { label: 'P95 Error', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 2.5 + Math.random() * 2 })), color: '#f59e0b' },
    { label: 'Max Error', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 5.0 + Math.random() * 5 })), color: '#ef4444' },
  ]), []);

  const linkQualityTrend = useMemo<TimeSeriesData[]>(() => ([
    { label: 'Avg Link Quality', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 85 + Math.random() * 15 })), color: '#22c55e' },
    { label: 'Min Link Quality', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 60 + Math.random() * 20 })), color: '#f59e0b' },
  ]), []);

  const detectionTrend = useMemo<TimeSeriesData[]>(() => ([
    { label: 'Total Detections', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 200 + Math.floor(Math.random() * 100) })), color: '#8b5cf6' },
    { label: 'Replans', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: Math.floor(Math.random() * 5) })), color: '#f59e0b' },
  ]), []);

  const generateReport = async (type: string) => {
    setGeneratingReport(true);
    await new Promise(r => setTimeout(r, 2000));
    const newReport = {
      id: `report-${Date.now()}`,
      flight_log_id: 'fleet-summary',
      generated_at: new Date().toISOString(),
      status: 'completed',
      format: 'html',
      template: type,
      file_path: `/reports/${type}-${Date.now()}.html`,
      file_size: Math.floor(Math.random() * 1000000) + 500000,
      summary: `${type.charAt(0).toUpperCase() + type.slice(1)} report generated for fleet`,
    };
    setReports(prev => [newReport, ...prev]);
    setGeneratingReport(false);
  };

  return (
    <div className="h-screen w-full flex flex-col bg-gray-50">
      {/* Header */}
      <header className="bg-white border-b border-gray-200 p-4">
        <div className="max-w-full mx-auto flex flex-wrap items-center justify-between gap-4">
          <div className="flex items-center gap-4">
            <h1 className="text-xl font-semibold text-gray-900">Analytics Dashboard</h1>
            <div className="flex items-center gap-2">
              <Calendar className="w-4 h-4 text-gray-400" />
              <Select
                value={dateRange[0].toISOString().split('T')[0] + '_' + dateRange[1].toISOString().split('T')[0]}
                onChange={e => {
                  const [start, end] = e.target.value.split('_');
                  setDateRange([new Date(start), new Date(end)]);
                }}
                options={[
                  { value: 'today', label: 'Today' },
                  { value: 'week', label: 'Last 7 Days' },
                  { value: 'month', label: 'Last 30 Days' },
                  { value: 'quarter', label: 'Last 90 Days' },
                  { value: 'year', label: 'Last Year' },
                ]}
                className="w-48"
              />
            </div>
          </div>

          <div className="flex items-center gap-4">
            <Button variant="secondary" onClick={() => setShowReportModal(true)}>
              <FileText className="w-4 h-4 mr-2" />
              Generate Report
            </Button>
            <Button variant="secondary">
              <Download className="w-4 h-4 mr-2" />
              Export Data
            </Button>
            <Button variant="primary" onClick={() => setGeneratingReport(true)}>
              <RefreshCw className="w-4 h-4 mr-2" />
              Refresh
            </Button>
          </div>
        </div>
      </header>

      <div className="flex-1 flex overflow-hidden">
        {/* Sidebar */}
        <aside className="w-64 bg-white border-r border-gray-200 flex flex-col">
          <div className="p-4 border-b border-gray-200">
            <h3 className="font-semibold text-gray-900 mb-3">Analytics Modules</h3>
            <nav className="space-y-1">
              {[
                { id: 'fleet', label: 'Fleet Overview', icon: Users },
                { id: 'vehicle', label: 'Vehicle Comparison', icon: Activity },
                { id: 'formation', label: 'Formation Analysis', icon: Network },
                { id: 'ai', label: 'AI & Perception', icon: Cpu },
                { id: 'qos', label: 'QoS & Comms', icon: Signal },
                { id: 'reports', label: 'Reports', icon: FileText },
              ].map(item => (
                <button
                  key={item.id}
                  onClick={() => setActiveTab(item.id as any)}
                  className={cn(
                    'w-full flex items-center gap-3 px-3 py-2 rounded-lg text-left transition-colors',
                    activeTab === item.id
                      ? 'bg-primary-50 text-primary-700 font-medium'
                      : 'text-gray-600 hover:bg-gray-50'
                  )}
                >
                  <item.icon className="w-5 h-5" />
                  <span>{item.label}</span>
                </button>
              ))}
            </nav>
          </div>

          <div className="p-4 border-b border-gray-200">
            <h3 className="font-semibold text-gray-900 mb-3">Quick Filters</h3>
            <div className="space-y-2">
              <Select
                placeholder="All Vehicles"
                options={[
                  { value: 'all', label: 'All Vehicles' },
                  ...Array.from({ length: 10 }, (_, i) => ({ value: String(i + 1), label: `Drone ${i + 1}` })),
                ]}
                className="w-full"
              />
              <Input
                type="date"
                placeholder="Start Date"
                className="text-sm"
              />
              <Input
                type="date"
                placeholder="End Date"
                className="text-sm"
              />
            </div>
          </div>

          <div className="flex-1"></div>

          <div className="p-4 border-t border-gray-200">
            <Button variant="secondary" className="w-full" onClick={() => {}}>
              <Download className="w-4 h-4 mr-2" />
              Export CSV
            </Button>
            <Button variant="outline" className="w-full mt-2" onClick={() => {}}>
              <Upload className="w-4 h-4 mr-2" />
              Import Data
            </Button>
          </div>
        </aside>

        {/* Main Content */}
        <main className="flex-1 flex flex-col overflow-hidden">
          {/* Fleet Overview Tab */}
          {activeTab === 'fleet' && (
            <div className="flex-1 overflow-y-auto p-6">
              {/* KPI Cards */}
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 xl:grid-cols-6 gap-4 mb-6">
                <MetricCard
                  icon={Users}
                  label="Total Flights"
                  value={fleetMetrics.totalFlights.toLocaleString()}
                  trend="+12%"
                  trendUp
                  color="primary"
                />
                <MetricCard
                  icon={Clock}
                  label="Flight Hours"
                  value={fleetMetrics.totalFlightTime.toFixed(1)}h
                  trend="+8.2%"
                  trendUp
                  color="success"
                />
                <MetricCard
                  icon={MapPin}
                  label="Distance (km)"
                  value={fleetMetrics.totalDistance.toLocaleString()}
                  trend="+15.3%"
                  trendUp
                  color="warning"
                />
                <MetricCard
                  icon={Network}
                  label="Formation Error"
                  value={fleetMetrics.avgFormationError.toFixed(2)}m
                  trend="-5.2%"
                  trendUp
                  color="info"
                />
                <MetricCard
                  icon={Cpu}
                  label="AI Detections"
                  value={fleetMetrics.totalDetections.toLocaleString()}
                  trend="+23%"
                  trendUp
                  color="purple"
                />
                <MetricCard
                  icon={Shield}
                  label="Safety Events"
                  value={fleetMetrics.totalSafetyEvents}
                  critical={fleetMetrics.criticalEvents}
                  trend="-18%"
                  trendUp
                  color="danger"
                />
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-6">
                <MetricCard
                  icon={Battery}
                  label="Battery Efficiency"
                  value={`${fleetMetrics.avgBatteryEfficiency}%`}
                  trend="+2.1%"
                  trendUp
                  color="success"
                  fullWidth
                />
                <MetricCard
                  icon={Signal}
                  label="Link Quality"
                  value={`${fleetMetrics.avgLinkQuality}%`}
                  trend="+1.8%"
                  trendUp
                  color="primary"
                  fullWidth
                />
                <MetricCard
                  icon={Zap}
                  label="Fleet Availability"
                  value={`${fleetMetrics.fleetAvailability}%`}
                  trend="+0.5%"
                  trendUp
                  color="warning"
                  fullWidth
                />
              </div>

              {/* Charts Row 1 */}
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card className="lg:col-span-2">
                  <CardHeader className="flex items-center justify-between">
                    <h3>Fleet Battery Trend (30 Days)</h3>
                    <Badge variant="info">Daily Avg</Badge>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={fleetBatteryTrend}
                      unit="%"
                      height={300}
                      threshold={[
                        { value: 30, color: '#ef4444', label: 'Critical' },
                        { value: 50, color: '#f59e0b', label: 'Warning' },
                      ]}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Fleet Availability</h3>
                    <Badge variant="success">94.2%</Badge>
                  </CardHeader>
                  <CardContent>
                    <div className="h-64 flex items-center justify-center">
                      <div className="text-center">
                        <div className="text-5xl font-bold text-success-600">94.2%</div>
                        <div className="text-sm text-gray-500">Fleet Availability</div>
                        <div className="mt-2 text-xs text-success-600">+0.5% vs last month</div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </div>

              {/* Charts Row 2 */}
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card>
                  <CardHeader>
                    <h3>Formation Error Trend</h3>
                    <Badge variant="warning">P95: 3.2m</Badge>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={formationErrorTrend}
                      unit="m"
                      height={300}
                      threshold={[
                        { value: 2, color: '#f59e0b', label: 'Degraded' },
                        { value: 5, color: '#ef4444', label: 'Lost' },
                      ]}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Link Quality Trend</h3>
                    <Badge variant="success">Avg: 91.7%</Badge>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={linkQualityTrend}
                      unit="%"
                      height={300}
                      threshold={[
                        { value: 70, color: '#22c55e', label: 'Good' },
                        { value: 50, color: '#ef4444', label: 'Poor' },
                      ]}
                    />
                  </CardContent>
                </Card>
              </div>

              {/* Charts Row 3 */}
              <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-6">
                <Card>
                  <CardHeader>
                    <h3>Detection Distribution</h3>
                    <Badge variant="purple">8,942 Total</Badge>
                  </CardHeader>
                  <CardContent>
                    <DetectionDistributionChart
                      detections={{
                        'Person': 2341, 'Vehicle': 1876, 'Power Line': 1234,
                        'Building': 987, 'Tree': 654, 'Drone': 432, 'Bird': 321, 'Other': 1110
                      }}
                      height={300}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>AI Activity Trend</h3>
                    <Badge variant="purple">Replans: 47</Badge>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={detectionTrend}
                      height={300}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Vehicle Health Radar</h3>
                    <Badge variant="info">10 Vehicles</Badge>
                  </CardHeader>
                  <CardContent>
                    <RadarChart
                      vehicles={vehicleComparisonData}
                      labels={radarLabels}
                      height={300}
                    />
                  </CardContent>
                </Card>
              </div>
            </div>
          )}

          {/* Vehicle Comparison Tab */}
          {activeTab === 'vehicle' && (
            <div className="flex-1 overflow-y-auto p-6">
              <div className="mb-6 flex items-center justify-between">
                <h2 className="text-xl font-semibold">Vehicle Comparison</h2>
                <Select
                  placeholder="Select Vehicles"
                  options={vehicles.map(v => ({ value: String(v.id), label: v.callsign }))}
                  className="w-64"
                />
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card className="lg:col-span-2">
                  <CardHeader>
                    <h3>Multi-Metric Radar Comparison</h3>
                    <p className="text-sm text-gray-500">Normalized metrics across all vehicles</p>
                  </CardHeader>
                  <CardContent>
                    <RadarChart
                      vehicles={vehicleComparisonData}
                      labels={radarLabels}
                      height={400}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Battery Comparison</h3>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {vehicles.map(v => (
                        <div key={v.id} className="flex items-center gap-3">
                          <Avatar name={v.callsign} size="sm" />
                          <div className="flex-1">
                            <div className="flex justify-between text-sm">
                              <span>{v.callsign}</span>
                              <span className={cn('font-medium', getBatteryColor(v.battery_remaining))}>
                                {v.battery_remaining}%
                              </span>
                            </div>
                            <div className="w-full h-2 bg-gray-200 rounded-full overflow-hidden">
                              <div 
                                className="h-full bg-gradient-to-r from-danger-500 via-warning-500 to-success-500 rounded-full transition-all"
                                style={{ width: `${v.battery_remaining}%` }}
                              />
                            </div>
                          </div>
                        ))}
                      </div>
                    </CardContent>
                  </Card>
                </div>

                <Card>
                  <CardHeader>
                    <h3>Link Quality Comparison</h3>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {vehicles.map(v => (
                        <div key={v.id} className="flex items-center gap-3">
                          <Avatar name={v.callsign} size="sm" />
                          <div className="flex-1">
                            <div className="flex justify-between text-sm">
                              <span>{v.callsign}</span>
                              <span className={cn('font-medium', getLinkQualityColor(v.link_quality))}>
                                {Math.round(v.link_quality * 100)}%
                              </span>
                            </div>
                            <div className="w-full h-2 bg-gray-200 rounded-full overflow-hidden">
                              <div 
                                className="h-full bg-gradient-to-r from-danger-500 via-warning-500 to-success-500 rounded-full transition-all"
                                style={{ width: `${v.link_quality * 100}%` }}
                              />
                            </div>
                          </div>
                        </div>
                      ))}
                    </CardContent>
                  </Card>
                </div>
              </div>

              {/* Vehicle Details Table */}
              <Card>
                <CardHeader>
                  <h3>Vehicle Metrics Table</h3>
                </CardHeader>
                <CardContent>
                  <Table
                    data={vehicles}
                    columns={[
                      { key: 'id', header: 'ID', render: (v: DroneState) => `#${v.id}` },
                      { key: 'callsign', header: 'Callsign', render: (v: DroneState) => (
                        <div className="flex items-center gap-2">
                          <Avatar name={v.callsign} size="xs" status={v.is_flying ? 'online' : 'offline'} />
                          <span className="font-medium">{v.callsign}</span>
                        </div>
                      )},
                      { key: 'flight_mode', header: 'Mode', render: (v: DroneState) => (
                        <Badge variant="info">{v.flight_mode}</Badge>
                      )},
                      { key: 'battery_remaining', header: 'Battery', render: (v: DroneState) => (
                        <span className={cn('font-medium', getBatteryColor(v.battery_remaining))}>{v.battery_remaining}%</span>
                      )},
                      { key: 'link_quality', header: 'Link', render: (v: DroneState) => (
                        <span className={cn('font-medium', getLinkQualityColor(v.link_quality))}>{Math.round(v.link_quality * 100)}%</span>
                      )},
                      { key: 'gps_satellites', header: 'GPS Sats', render: (v: DroneState) => String(v.gps_satellites) },
                      { key: 'ekf_innovation', header: 'EKF Innov', render: (v: DroneState) => v.ekf_innovation.toFixed(3) },
                      { key: 'inference_time_ms', header: 'AI Time', render: (v: DroneState) => `${v.inference_time_ms.toFixed(1)}ms` },
                      { key: 'formation_error', header: 'Form Err', render: (v: DroneState) => v.formation_error ? `${v.formation_error.toFixed(2)}m` : 'N/A' },
                    ]}
                    keyExtractor={(v: DroneState) => String(v.id)}
                  />
                </CardContent>
              </Card>
            </div>
          )}

          {/* Formation Analysis Tab */}
          {activeTab === 'formation' && (
            <div className="flex-1 overflow-y-auto p-6">
              <h2 className="text-xl font-semibold mb-6">Formation Analysis</h2>
              
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card className="lg:col-span-2">
                  <CardHeader>
                    <h3>Formation Error CDF</h3>
                    <Badge variant="warning">P95: 3.2m</Badge>
                  </CardHeader>
                  <CardContent>
                    <FormationErrorCDFChart
                      cdfX={[0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 6, 7, 8, 9, 10]}
                      cdfY={[0.05, 0.15, 0.3, 0.45, 0.6, 0.72, 0.8, 0.87, 0.91, 0.94, 0.96, 0.97, 0.98, 0.99, 1.0]}
                      mean={2.1}
                      p95={3.2}
                      p99={6.8}
                      height={350}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Formation Statistics</h3>
                  </CardHeader>
                  <CardContent>
                    <div className="grid grid-cols-2 gap-4">
                      <StatItem label="Mean Error" value="2.14m" />
                      <StatItem label="Median Error" value="1.82m" />
                      <StatItem label="Std Deviation" value="1.45m" />
                      <StatItem label="Max Error" value="8.92m" />
                      <StatItem label="P95 Error" value="3.21m" />
                      <StatItem label="P99 Error" value="6.84m" />
                      <StatItem label="In Tolerance (<2m)" value="58.3%" color="success" />
                      <StatItem label="Degraded (2-5m)" value="32.1%" color="warning" />
                      <StatItem label="Lost (>5m)" value="9.6%" color="danger" />
                    </div>
                  </CardContent>
                </Card>
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card className="lg:col-span-2">
                  <CardHeader>
                    <h3>Error Timeline</h3>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={[
                        { label: 'Position Error', data: Array.from({ length: 100 }, (_, i) => ({ timestamp: Date.now() - (99 - i) * 60000, value: 1.5 + Math.random() * 2 })), color: '#ef4444' },
                        { label: 'Velocity Error', data: Array.from({ length: 100 }, (_, i) => ({ timestamp: Date.now() - (99 - i) * 60000, value: 0.2 + Math.random() * 0.5 })), color: '#3b82f6' },
                        { label: 'Attitude Error', data: Array.from({ length: 100 }, (_, i) => ({ timestamp: Date.now() - (99 - i) * 60000, value: 0.5 + Math.random() * 1 })), color: '#f59e0b' },
                      ]}
                      height={300}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Per-Vehicle Formation Error</h3>
                  </CardHeader>
                  <CardContent>
                    <Table
                      data={vehicles.map(v => ({
                        id: v.id,
                        callsign: v.callsign,
                        mean_error: v.formation_error || 0,
                        max_error: (v.formation_error || 0) * 2.5,
                        in_tolerance: Math.max(0, 100 - (v.formation_error || 0) * 20),
                      }))}
                      columns={[
                        { key: 'callsign', header: 'Vehicle', render: (v: any) => <span className="font-medium">{v.callsign}</span> },
                        { key: 'mean_error', header: 'Mean (m)', render: (v: any) => v.mean_error.toFixed(2) },
                        { key: 'max_error', header: 'Max (m)', render: (v: any) => v.max_error.toFixed(2) },
                        { key: 'in_tolerance', header: 'In Tol. %', render: (v: any) => (
                          <span className={cn('font-medium', v.in_tolerance > 80 ? 'text-success-600' : v.in_tolerance > 50 ? 'text-warning-600' : 'text-danger-600')}>
                            {v.in_tolerance.toFixed(1)}%
                          </span>
                        )},
                      ]}
                      keyExtractor={(v: any) => String(v.id)}
                    />
                  </CardContent>
                </Card>
              </div>
            </div>
          )}

          {/* AI & Perception Tab */}
          {activeTab === 'ai' && (
            <div className="flex-1 overflow-y-auto p-6">
              <h2 className="text-xl font-semibold mb-6">AI & Perception Analytics</h2>

              <div className="grid grid-cols-1 lg:grid-cols-4 gap-4 mb-6">
                <MetricCard icon={Cpu} label="Total Inferences" value="127,842" trend="+15%" trendUp color="purple" />
                <MetricCard icon={Eye} label="Total Detections" value="8,941" trend="+23%" trendUp color="primary" />
                <MetricCard icon={Zap} label="Avg Inference Time" value="34.2ms" trend="-12%" trendUp color="success" />
                <MetricCard icon={AlertTriangle} label="Faults Detected" value="47" critical={3} trend="-25%" trendUp color="danger" />
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card className="lg:col-span-2">
                  <CardHeader>
                    <h3>Detection Distribution</h3>
                    <Badge variant="purple">8,941 Total</Badge>
                  </CardHeader>
                  <CardContent>
                    <DetectionDistributionChart
                      detections={{
                        'Person': 2341, 'Vehicle': 1876, 'Power Line': 1234,
                        'Building': 987, 'Tree': 654, 'Drone': 432, 'Bird': 321, 'Other': 1110
                      }}
                      height={350}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Inference Time Trend</h3>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={[
                        { label: 'Avg Inference', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 30 + Math.random() * 20 })), color: '#8b5cf6' },
                        { label: 'Max Inference', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 50 + Math.random() * 50 })), color: '#ef4444' },
                      ]}
                      unit="ms"
                      height={300}
                      threshold={[
                        { value: 50, color: '#f59e0b', label: 'RT Threshold' },
                        { value: 100, color: '#ef4444', label: 'Over Budget' },
                      ]}
                    />
                  </CardContent>
                </Card>
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card>
                  <CardHeader>
                    <h3>Detection Confidence Distribution</h3>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={[
                        { label: 'High (>0.8)', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 60 + Math.random() * 20 })), color: '#22c55e' },
                        { label: 'Medium (0.5-0.8)', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 20 + Math.random() * 15 })), color: '#f59e0b' },
                        { label: 'Low (<0.5)', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 5 + Math.random() * 10 })), color: '#ef4444' },
                      ]}
                      unit="%"
                      height={300}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Replans & Faults Trend</h3>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={detectionTrend}
                      height={300}
                    />
                  </CardContent>
                </Card>
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card>
                  <CardHeader>
                    <h3>Top Detection Classes</h3>
                  </CardHeader>
                  <CardContent>
                    <Table
                      data={[
                        { class: 'Person', count: 2341, avgConf: 0.87, avgTime: 32.1 },
                        { class: 'Vehicle', count: 1876, avgConf: 0.82, avgTime: 28.5 },
                        { class: 'Power Line', count: 1234, avgConf: 0.91, avgTime: 35.2 },
                        { class: 'Building', count: 987, avgConf: 0.79, avgTime: 25.8 },
                        { class: 'Tree', count: 654, avgConf: 0.75, avgTime: 22.3 },
                        { class: 'Drone', count: 432, avgConf: 0.88, avgTime: 38.7 },
                        { class: 'Bird', count: 321, avgConf: 0.68, avgTime: 19.5 },
                        { class: 'Other', count: 1110, avgConf: 0.71, avgTime: 24.1 },
                      ]}
                      columns={[
                        { key: 'class', header: 'Class', render: (d: any) => <span className="font-medium">{d.class}</span> },
                        { key: 'count', header: 'Count', render: (d: any) => d.count.toLocaleString() },
                        { key: 'avgConf', header: 'Avg Confidence', render: (d: any) => (d.avgConf * 100).toFixed(1) + '%' },
                        { key: 'avgTime', header: 'Avg Time (ms)', render: (d: any) => d.avgTime.toFixed(1) },
                      ]}
                      keyExtractor={(d: any) => d.class}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>AI Faults & Replans</h3>
                  </CardHeader>
                  <CardContent>
                    <Table
                      data={[
                        { type: 'GPS Spoofing', count: 12, severity: 'Critical', lastSeen: '2h ago' },
                        { type: 'Sensor Drift', count: 8, severity: 'High', lastSeen: '5h ago' },
                        { type: 'Comm Loss', count: 15, severity: 'Medium', lastSeen: '1h ago' },
                        { type: 'Battery Anomaly', count: 7, severity: 'High', lastSeen: '3h ago' },
                        { type: 'Formation Break', count: 5, severity: 'Critical', lastSeen: '4h ago' },
                      ]}
                      columns={[
                        { key: 'type', header: 'Fault Type', render: (f: any) => <span className="font-medium">{f.type}</span> },
                        { key: 'count', header: 'Count', render: (f: any) => f.count },
                        { key: 'severity', header: 'Severity', render: (f: any) => <Badge variant={f.severity === 'Critical' ? 'danger' : f.severity === 'High' ? 'warning' : 'info'}>{f.severity}</Badge> },
                        { key: 'lastSeen', header: 'Last Seen', render: (f: any) => f.lastSeen },
                      ]}
                      keyExtractor={(f: any) => f.type}
                    />
                  </CardContent>
                </Card>
              </div>
            </div>
          )}

          {/* QoS & Comms Tab */}
          {activeTab === 'qos' && (
            <div className="flex-1 overflow-y-auto p-6">
              <h2 className="text-xl font-semibold mb-6">QoS & Communications Analysis</h2>

              <div className="grid grid-cols-1 lg:grid-cols-4 gap-4 mb-6">
                <MetricCard icon={Signal} label="Avg Link Quality" value="91.7%" trend="+2.1%" trendUp color="primary" />
                <MetricCard icon={Zap} label="Avg Latency" value="23.4ms" trend="-12%" trendUp color="success" />
                <MetricCard icon={Network} label="Packet Loss" value="0.34%" trend="-45%" trendUp color="success" />
                <MetricCard icon={AlertTriangle} label="Critical Links" value="3" critical={3} trend="+50%" color="danger" />
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card className="lg:col-span-2">
                  <CardHeader>
                    <h3>Link Quality Timeline</h3>
                    <Badge variant="success">Avg: 91.7%</Badge>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={linkQualityTrend}
                      unit="%"
                      height={300}
                      threshold={[
                        { value: 70, color: '#22c55e', label: 'Good' },
                        { value: 50, color: '#ef4444', label: 'Poor' },
                      ]}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>Latency Distribution</h3>
                  </CardHeader>
                  <CardContent>
                    <TimeSeriesChart
                      data={[
                        { label: 'Avg Latency', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 20 + Math.random() * 15 })), color: '#3b82f6' },
                        { label: 'P95 Latency', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 40 + Math.random() * 30 })), color: '#f59e0b' },
                        { label: 'Max Latency', data: Array.from({ length: 30 }, (_, i) => ({ timestamp: Date.now() - (29 - i) * 86400000, value: 80 + Math.random() * 50 })), color: '#ef4444' },
                      ]}
                      unit="ms"
                      height={300}
                    />
                  </CardContent>
                </Card>
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                <Card>
                  <CardHeader>
                    <h3>Per-Link Quality Matrix</h3>
                  </CardHeader>
                  <CardContent>
                    <Table
                      data={[
                        { from: 1, to: 2, quality: 0.95, rssi: -45, latency: 8, loss: 0.001, status: 'Excellent' },
                        { from: 1, to: 3, quality: 0.87, rssi: -52, latency: 12, loss: 0.003, status: 'Good' },
                        { from: 1, to: 4, quality: 0.72, rssi: -65, latency: 25, loss: 0.02, status: 'Fair' },
                        { from: 2, to: 3, quality: 0.91, rssi: -48, latency: 10, loss: 0.002, status: 'Excellent' },
                        { from: 2, to: 4, quality: 0.65, rssi: -70, latency: 35, loss: 0.05, status: 'Poor' },
                        { from: 3, to: 4, quality: 0.88, rssi: -50, latency: 11, loss: 0.004, status: 'Good' },
                      ]}
                      columns={[
                        { key: 'from', header: 'From', render: (l: any) => `Drone ${l.from}` },
                        { key: 'to', header: 'To', render: (l: any) => `Drone ${l.to}` },
                        { key: 'quality', header: 'Quality', render: (l: any) => (
                          <span className={cn('font-medium', l.quality > 0.8 ? 'text-success-600' : l.quality > 0.7 ? 'text-warning-600' : 'text-danger-600')}>
                            {(l.quality * 100).toFixed(1)}%
                          </span>
                        )},
                        { key: 'rssi', header: 'RSSI (dBm)', render: (l: any) => l.rssi },
                        { key: 'latency', header: 'Latency (ms)', render: (l: any) => l.latency },
                        { key: 'loss', header: 'Loss %', render: (l: any) => (l.loss * 100).toFixed(2) + '%' },
                        { key: 'status', header: 'Status', render: (l: any) => <Badge variant={l.status === 'Excellent' ? 'success' : l.status === 'Good' ? 'info' : l.status === 'Fair' ? 'warning' : 'danger'}>{l.status}</Badge> },
                      ]}
                      keyExtractor={(l: any) => `${l.from}-${l.to}`}
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <h3>QoS Recommendations</h3>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      <div className="p-3 bg-warning-50 border-l-4 border-warning-500 rounded-r-lg">
                        <div className="flex items-start gap-2">
                          <AlertTriangle className="w-5 h-5 text-warning-600 mt-0.5" />
                          <div>
                            <p className="font-medium text-warning-800">Link 2→4 Degraded</p>
                            <p className="text-sm text-warning-700">RSSI -70dBm, 5% packet loss. Consider repositioning or adding relay.</p>
                          </div>
                        </div>
                      </div>
                      <div className="p-3 bg-warning-50 border-l-4 border-warning-500 rounded-r-lg">
                        <div className="flex items-start gap-2">
                          <AlertTriangle className="w-5 h-5 text-warning-600 mt-0.5" />
                          <div>
                            <p className="font-medium text-warning-800">High Latency on 1→4</p>
                            <p className="text-sm text-warning-700">25ms latency exceeds 20ms threshold. Check for interference.</p>
                          </div>
                        </div>
                      </div>
                      <div className="p-3 bg-success-50 border-l-4 border-success-500 rounded-r-lg">
                        <div className="flex items-start gap-2">
                          <CheckCircle className="w-5 h-5 text-success-600 mt-0.5" />
                          <div>
                            <p className="font-medium text-success-800">Mesh Topology Healthy</p>
                            <p className="text-sm text-success-700">All other links above 85% quality. Redundancy maintained.</p>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </div>
          )}

          {/* Reports Tab */}
          {activeTab === 'reports' && (
            <div className="flex-1 overflow-y-auto p-6">
              <div className="flex items-center justify-between mb-6">
                <h2 className="text-xl font-semibold">Generated Reports</h2>
                <Button variant="primary" onClick={() => setShowReportModal(true)}>
                  <FileText className="w-4 h-4 mr-2" />
                  Generate New Report
                </Button>
              </div>

              <Card>
                <CardContent>
                  {reports.length === 0 ? (
                    <div className="text-center py-12">
                      <FileText className="w-12 h-12 text-gray-300 mx-auto mb-4" />
                      <h3 className="text-lg font-medium text-gray-900 mb-2">No reports generated</h3>
                      <p className="text-gray-500 mb-6">Generate your first post-flight analysis report</p>
                      <Button variant="primary" onClick={() => setShowReportModal(true)}>
                        <FileText className="w-4 h-4 mr-2" />
                        Generate Report
                      </Button>
                    </div>
                  ) : (
                    <Table
                      data={reports}
                      columns={[
                        { key: 'id', header: 'Report ID', render: (r: AnalysisReport) => <span className="font-mono text-sm">{r.id}</span> },
                        { key: 'flight_log_id', header: 'Flight Log', render: (r: AnalysisReport) => r.flight_log_id },
                        { key: 'generated_at', header: 'Generated', render: (r: AnalysisReport) => formatTimestamp(r.generated_at) },
                        { key: 'status', header: 'Status', render: (r: AnalysisReport) => (
                          <Badge variant={r.status === 'completed' ? 'success' : r.status === 'generating' ? 'info' : 'danger'}>
                            {r.status}
                          </Badge>
                        )},
                        { key: 'format', header: 'Format', render: (r: AnalysisReport) => r.format.toUpperCase() },
                        { key: 'template', header: 'Template', render: (r: AnalysisReport) => r.template },
                        { key: 'file_size', header: 'Size', render: (r: AnalysisReport) => `${(r.file_size / 1024).toFixed(1)} KB` },
                        { key: 'actions', header: 'Actions', render: (r: AnalysisReport) => (
                          <div className="flex items-center gap-2">
                            <Button variant="ghost" size="sm" onClick={() => window.open(r.file_path, '_blank')}>
                              <Eye className="w-4 h-4" />
                            </Button>
                            <Button variant="ghost" size="sm">
                              <Download className="w-4 h-4" />
                            </Button>
                          </div>
                        )},
                      ]}
                      keyExtractor={(r: AnalysisReport) => r.id}
                    />
                  </CardContent>
                </Card>
              )}
            </div>
          )}

          {/* Report Generation Modal */}
          {showReportModal && (
            <Modal
              isOpen={true}
              onClose={() => setShowReportModal(false)}
              title="Generate New Report"
              size="lg"
            >
              <div className="space-y-4">
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Flight Log</label>
                  <Select
                    placeholder="Select flight log..."
                    options={availableLogs.map(log => ({ value: log.log_id, label: `${log.callsign} - ${formatTimestamp(log.flight_start)}` }))}
                  />
                </div>
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Format</label>
                    <Select
                      options={[
                        { value: 'html', label: 'HTML (Interactive)' },
                        { value: 'pdf', label: 'PDF (Printable)' },
                        { value: 'json', label: 'JSON (Data)' },
                      ]}
                    />
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Template</label>
                    <Select
                      options={[
                        { value: 'standard', label: 'Standard' },
                        { value: 'detailed', label: 'Detailed' },
                        { value: 'executive', label: 'Executive Summary' },
                        { value: 'regulatory', label: 'Regulatory Compliance' },
                      ]}
                    />
                  </div>
                </div>
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Include Sections</label>
                  <div className="grid grid-cols-2 gap-2">
                    <label className="flex items-center gap-2">
                      <input type="checkbox" defaultChecked className="rounded border-gray-300 text-primary-600" />
                      <span className="text-sm">Coverage Heatmap</span>
                    </label>
                    <label className="flex items-center gap-2">
                      <input type="checkbox" defaultChecked className="rounded border-gray-300 text-primary-600" />
                      <span className="text-sm">Formation CDF</span>
                    </label>
                    <label className="flex items-center gap-2">
                      <input type="checkbox" defaultChecked className="rounded border-gray-300 text-primary-600" />
                      <span className="text-sm">AI Timeline</span>
                    </label>
                    <label className="flex items-center gap-2">
                      <input type="checkbox" defaultChecked className="rounded border-gray-300 text-primary-600" />
                      <span className="text-sm">QoS Analysis</span>
                    </label>
                  </div>
                </div>
                <div className="flex justify-end gap-2 pt-4">
                  <Button variant="secondary" onClick={() => setShowReportModal(false)}>Cancel</Button>
                  <Button variant="primary" onClick={() => { setShowReportModal(false); generateReport('standard'); }} disabled={generatingReport}>
                    {generatingReport ? 'Generating...' : 'Generate Report'}
                  </Button>
                </div>
              </div>
            </Modal>
          )}
        </main>
      </div>
    </div>
  );
};

// Helper Components
const MetricCard: React.FC<{
  icon: React.ComponentType<{ className?: string }>;
  label: string;
  value: string;
  trend?: string;
  trendUp?: boolean;
  color?: string;
  fullWidth?: boolean;
  critical?: number;
}> = ({ icon: Icon, label, value, trend, trendUp, color = 'primary', fullWidth, critical }) => (
  <Card className={cn(fullWidth && 'lg:col-span-3')}>
    <CardContent className="p-4">
      <div className="flex items-start justify-between">
        <div>
          <p className="text-sm text-gray-500">{label}</p>
          <p className="text-2xl font-bold text-gray-900 mt-1">{value}</p>
          {trend && (
            <div className={cn('flex items-center gap-1 mt-2 text-sm', trendUp ? 'text-success-600' : 'text-danger-600')}>
              {trendUp ? <TrendingUp className="w-4 h-4" /> : <TrendingDown className="w-4 h-4" />}
              <span>{trend}</span>
            </div>
          )}
        </div>
        <div className={cn('p-2 rounded-lg', `bg-${color}-100`)}>
          <Icon className={cn('w-6 h-6', `text-${color}-600`)} />
        </div>
      </div>
      {critical !== undefined && (
        <div className="mt-2 flex items-center gap-1 text-xs text-danger-600">
          <AlertTriangle className="w-3 h-3" />
          <span>{critical} Critical</span>
        </div>
      )}
    </CardContent>
  </Card>
);

const StatItem: React.FC<{ label: string; value: string; color?: string }> = ({ label, value, color }) => (
  <div>
    <p className="text-sm text-gray-500">{label}</p>
    <p className={cn('font-medium', color && `text-${color}-600`)}>{value}</p>
  </div>
);

const radarLabels = ['Battery', 'Link Quality', 'Estimator', 'GPS', 'Availability'];

const availableLogs = [
  { log_id: 'flight-001', callsign: 'SWARM-ALPHA', flight_start: '2025-01-15T10:30:00Z' },
  { log_id: 'flight-002', callsign: 'SWARM-BRAVO', flight_start: '2025-01-14T14:15:00Z' },
  { log_id: 'flight-003', callsign: 'SWARM-CHARLIE', flight_start: '2025-01-13T09:45:00Z' },
];