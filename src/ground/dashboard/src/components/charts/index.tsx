// Chart components using Chart.js and react-chartjs-2

import React, { useMemo, useRef, useEffect } from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  Title,
  Tooltip,
  Legend,
  Filler,
  ChartOptions,
} from 'chart.js';
import {
  LineChart,
  BarChart,
  DoughnutChart,
  RadarChart,
  ScatterChart,
  AreaChart,
} from 'react-chartjs-2';
import { cn } from '@/utils/helpers';
import type { ChartDataPoint, TimeSeriesData } from '@/types';

// Register Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  Title,
  Tooltip,
  Legend,
  Filler
);

// Common chart options
const defaultOptions: ChartOptions = {
  responsive: true,
  maintainAspectRatio: false,
  interaction: {
    mode: 'index',
    intersect: false,
  },
  plugins: {
    legend: {
      display: true,
      position: 'top',
      labels: {
        usePointStyle: true,
        padding: 20,
        font: { size: 12 },
      },
    },
    tooltip: {
      backgroundColor: 'rgba(0, 0, 0, 0.8)',
      padding: 12,
      titleFont: { size: 13 },
      bodyFont: { size: 12 },
      cornerRadius: 8,
      displayColors: true,
    },
  },
  scales: {
    x: {
      grid: {
        color: 'rgba(0, 0, 0, 0.05)',
        drawBorder: false,
      },
      ticks: {
        maxTicksLimit: 10,
        font: { size: 11 },
      },
    },
    y: {
      grid: {
        color: 'rgba(0, 0, 0, 0.05)',
        drawBorder: false,
      },
      ticks: {
        font: { size: 11 },
      },
    },
  },
};

// Color palette
const CHART_COLORS = [
  '#3b82f6', // primary
  '#22c55e', // success
  '#f59e0b', // warning
  '#ef4444', // danger
  '#8b5cf6', // purple
  '#ec4899', // pink
  '#06b6d4', // cyan
  '#84cc16', // lime
];

export function generateChartColors(count: number) {
  const colors = [];
  for (let i = 0; i < count; i++) {
    colors.push(CHART_COLORS[i % CHART_COLORS.length]);
  }
  return colors;
}

// Time Series Line Chart
interface TimeSeriesChartProps {
  data: TimeSeriesData[];
  title?: string;
  unit?: string;
  height?: number;
  showLegend?: boolean;
  showGrid?: boolean;
  smooth?: boolean;
  threshold?: { value: number; color: string; label: string }[];
  className?: string;
}

export const TimeSeriesChart: React.FC<TimeSeriesChartProps> = ({
  data,
  title,
  unit,
  height = 300,
  showLegend = true,
  showGrid = true,
  smooth = true,
  threshold,
  className,
}) => {
  const datasets = useMemo(() => {
    return data.map((series, index) => ({
      label: series.label,
      data: series.data.map(d => ({ x: d.timestamp, y: d.value })),
      borderColor: CHART_COLORS[index % CHART_COLORS.length],
      backgroundColor: `${CHART_COLORS[index % CHART_COLORS.length]}20`,
      borderWidth: 2,
      fill: true,
      tension: smooth ? 0.3 : 0,
      pointRadius: 0,
      pointHoverRadius: 6,
    }));
  }, [data, smooth]);

  const thresholdAnnotations = useMemo(() => {
    if (!threshold || threshold.length === 0) return {};
    
    return {
      annotations: threshold.reduce((acc, t, i) => {
        acc[`threshold${i}`] = {
          type: 'line' as const,
          mode: 'horizontal' as const,
          scale: 'y',
          value: t.value,
          borderColor: t.color,
          borderWidth: 2,
          borderDash: [5, 5],
          label: {
            enabled: true,
            content: t.label,
            position: 'start' as const,
            backgroundColor: t.color,
            color: '#fff',
            font: { size: 10 },
          },
        };
        return acc;
      }, {} as any),
    };
  }, [threshold]);

  const options = useMemo<ChartOptions<'line'>>(() => ({
    ...defaultOptions,
    plugins: {
      ...defaultOptions.plugins,
      legend: { display: showLegend, position: 'top' },
      annotation: thresholdAnnotations,
    },
    scales: {
      ...defaultOptions.scales,
      y: {
        ...defaultOptions.scales!.y,
        title: unit ? { display: true, text: unit } : undefined,
      },
    },
  }), [showLegend, unit, thresholdAnnotations]);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      {title && <h3 className="text-sm font-medium text-gray-700 mb-2">{title}</h3>}
      <div style={{ height: height - (title ? 30 : 0) }}>
        <LineChart data={{ datasets }} options={options} />
      </div>
    </div>
  );
};

// Multi-axis chart for different units
interface MultiAxisChartProps {
  series: {
    label: string;
    data: ChartDataPoint[];
    color: string;
    yAxisID: string;
    unit: string;
  }[];
  title?: string;
  height?: number;
  className?: string;
}

export const MultiAxisChart: React.FC<MultiAxisChartProps> = ({
  series,
  title,
  height = 300,
  className,
}) => {
  const datasets = useMemo(() => {
    return series.map((s, i) => ({
      label: s.label,
      data: s.data.map(d => ({ x: d.timestamp, y: d.value })),
      borderColor: s.color,
      backgroundColor: `${s.color}20`,
      borderWidth: 2,
      fill: false,
      tension: 0.3,
      yAxisID: s.yAxisID,
      pointRadius: 0,
      pointHoverRadius: 6,
    });
  }, [series]);

  const scales = useMemo(() => {
    const scaleConfig: any = {
      x: { ...defaultOptions.scales!.x },
    };
    
    series.forEach(s => {
      scales[s.yAxisID] = {
        type: 'linear' as const,
        position: series.indexOf(s) % 2 === 0 ? 'left' : 'right',
        title: { display: true, text: s.unit },
        grid: { drawOnChartArea: series.indexOf(s) === 0 },
      };
    });
    
    return scales;
  }, [series]);

  const options = useMemo<ChartOptions<'line'>>(() => ({
    ...defaultOptions,
    scales,
  }), [scales]);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      {title && <h3 className="text-sm font-medium text-gray-700 mb-2">{title}</h3>}
      <div style={{ height: height - (title ? 30 : 0) }}>
        <LineChart data={{ datasets }} options={options} />
      </div>
    </div>
  );
};

// Battery History Chart
interface BatteryChartProps {
  data: ChartDataPoint[];
  warningThreshold?: number;
  criticalThreshold?: number;
  height?: number;
  className?: string;
}

export const BatteryChart: React.FC<BatteryChartProps> = ({
  data,
  warningThreshold = 30,
  criticalThreshold = 20,
  height = 200,
  className,
}) => {
  const datasets = useMemo(() => [
    {
      label: 'Battery %',
      data: data.map(d => ({ x: d.timestamp, y: d.value })),
      borderColor: '#3b82f6',
      backgroundColor: 'rgba(59, 130, 246, 0.1)',
      borderWidth: 2,
      fill: true,
      tension: 0.2,
      pointRadius: 0,
      pointHoverRadius: 6,
    },
  ], [data]);

  const options = useMemo<ChartOptions<'line'>>(() => ({
    ...defaultOptions,
    plugins: {
      ...defaultOptions.plugins,
      legend: { display: false },
      annotation: {
        annotations: {
          warningLine: {
            type: 'line' as const,
            mode: 'horizontal' as const,
            scale: 'y',
            value: warningThreshold,
            borderColor: '#f59e0b',
            borderWidth: 2,
            borderDash: [5, 5],
            label: {
              enabled: true,
              content: `Warning (${warningThreshold}%)`,
              position: 'start' as const,
              backgroundColor: '#f59e0b',
              color: '#fff',
              font: { size: 10 },
            },
          },
          criticalLine: {
            type: 'line' as const,
            mode: 'horizontal' as const,
            scale: 'y',
            value: criticalThreshold,
            borderColor: '#ef4444',
            borderWidth: 2,
            borderDash: [5, 5],
            label: {
              enabled: true,
              content: `Critical (${criticalThreshold}%)`,
              position: 'start' as const,
              backgroundColor: '#ef4444',
              color: '#fff',
              font: { size: 10 },
            },
          },
        },
      },
    },
    scales: {
      ...defaultOptions.scales,
      y: {
        ...defaultOptions.scales!.y,
        min: 0,
        max: 100,
        title: { display: true, text: 'Battery %' },
      },
    },
  }), [warningThreshold, criticalThreshold]);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      <div style={{ height }}>
        <LineChart data={{ datasets }} options={options} />
      </div>
      <div className="flex items-center justify-center gap-4 mt-2 text-xs text-gray-500">
        <span className="flex items-center gap-1">
          <span className="w-3 h-3 rounded-full bg-blue-500" />
          Battery
        </span>
        <span className="flex items-center gap-1">
          <span className="w-3 h-3 rounded-full bg-warning-500" />
          Warning
        </span>
        <span className="flex items-center gap-1">
          <span className="w-3 h-3 rounded-full bg-danger-500" />
          Critical
        </span>
      </div>
    </div>
  );
};

// Link Quality Chart
interface LinkQualityChartProps {
  data: ChartDataPoint[];
  height?: number;
  className?: string;
}

export const LinkQualityChart: React.FC<LinkQualityChartProps> = ({
  data,
  height = 200,
  className,
}) => {
  const datasets = useMemo(() => [
    {
      label: 'Link Quality',
      data: data.map(d => ({ x: d.timestamp, y: d.value * 100 })),
      borderColor: '#22c55e',
      backgroundColor: 'rgba(34, 197, 94, 0.1)',
      borderWidth: 2,
      fill: true,
      tension: 0.2,
      pointRadius: 0,
      pointHoverRadius: 6,
    },
  ], [data]);

  const options = useMemo<ChartOptions<'line'>>(() => ({
    ...defaultOptions,
    plugins: {
      ...defaultOptions.plugins,
      legend: { display: false },
      annotation: {
        annotations: {
          goodLine: {
            type: 'line' as const,
            mode: 'horizontal' as const,
            scale: 'y',
            value: 70,
            borderColor: '#22c55e',
            borderWidth: 1,
            borderDash: [5, 5],
          },
          poorLine: {
            type: 'line' as const,
            mode: 'horizontal' as const,
            scale: 'y',
            value: 40,
            borderColor: '#ef4444',
            borderWidth: 1,
            borderDash: [5, 5],
          },
        },
      },
    },
    scales: {
      ...defaultOptions.scales,
      y: {
        ...defaultOptions.scales!.y,
        min: 0,
        max: 100,
        title: { display: true, text: 'Link Quality %' },
      },
    },
  }), []);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      <LineChart data={{ datasets }} options={options} />
    </div>
  );
};

// Formation Error CDF Chart
interface FormationErrorCDFChartProps {
  cdfX: number[];
  cdfY: number[];
  mean?: number;
  p95?: number;
  p99?: number;
  height?: number;
  className?: string;
}

export const FormationErrorCDFChart: React.FC<FormationErrorCDFChartProps> = ({
  cdfX,
  cdfY,
  mean,
  p95,
  p99,
  height = 300,
  className,
}) => {
  const datasets = useMemo(() => [
    {
      label: 'Position Error CDF',
      data: cdfX.map((x, i) => ({ x, y: cdfY[i] })),
      borderColor: '#3b82f6',
      backgroundColor: 'rgba(59, 130, 246, 0.1)',
      borderWidth: 2,
      fill: true,
      tension: 0.1,
      pointRadius: 0,
      pointHoverRadius: 4,
    },
  ], [cdfX, cdfY]);

  const annotations = useMemo(() => {
    const ann: any = {};
    if (mean !== undefined) {
      annotations.meanLine = {
        type: 'line' as const,
        mode: 'vertical' as const,
        scale: 'x',
        value: mean,
        borderColor: '#22c55e',
        borderWidth: 2,
        borderDash: [5, 5],
        label: { enabled: true, content: `Mean: ${mean.toFixed(2)}m`, position: 'start' as const, backgroundColor: '#22c55e', color: '#fff', font: { size: 10 } },
      };
    }
    if (p95 !== undefined) {
      annotations.p95Line = {
        type: 'line' as const,
        mode: 'vertical' as const,
        scale: 'x',
        value: p95,
        borderColor: '#f59e0b',
        borderWidth: 2,
        borderDash: [5, 5],
        label: { enabled: true, content: `P95: ${p95.toFixed(2)}m`, position: 'start' as const, backgroundColor: '#f59e0b', color: '#fff', font: { size: 10 } },
      };
    }
    if (p99 !== undefined) {
      annotations.p99Line = {
        type: 'line' as const,
        mode: 'vertical' as const,
        scale: 'x',
        value: p99,
        borderColor: '#ef4444',
        borderWidth: 2,
        borderDash: [5, 5],
        label: { enabled: true, content: `P99: ${p99.toFixed(2)}m`, position: 'start' as const, backgroundColor: '#ef4444', color: '#fff', font: { size: 10 } },
      };
    }
    return { annotations };
  }, [mean, p95, p99]);

  const options = useMemo<ChartOptions<'line'>>(() => ({
    ...defaultOptions,
    plugins: {
      ...defaultOptions.plugins,
      legend: { display: false },
      annotation: annotations,
    },
    scales: {
      ...defaultOptions.scales,
      x: {
        ...defaultOptions.scales!.x,
        title: { display: true, text: 'Position Error (m)' },
        min: 0,
      },
      y: {
        ...defaultOptions.scales!.y,
        title: { display: true, text: 'Cumulative Probability' },
        min: 0,
        max: 1,
        ticks: { callback: (value) => `${(value as number * 100).toFixed(0)}%` },
      },
    },
  }), [annotations]);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      <LineChart data={{ datasets }} options={options} />
    </div>
  );
};

// Detection Distribution Chart
interface DetectionDistributionChartProps {
  detections: Record<string, number>;
  height?: number;
  className?: string;
}

export const DetectionDistributionChart: React.FC<DetectionDistributionChartProps> = ({
  detections,
  height = 300,
  className,
}) => {
  const labels = Object.keys(detections);
  const values = Object.values(detections);
  const colors = useMemo(() => generateChartColors(labels.length), [labels.length]);

  const datasets = useMemo(() => [
    {
      label: 'Detections',
      data: values,
      backgroundColor: colors.map(c => `${c}80`),
      borderColor: colors,
      borderWidth: 2,
    },
  ], [values, colors]);

  const options = useMemo<ChartOptions<'doughnut'>>(() => ({
    ...defaultOptions,
    plugins: {
      ...defaultOptions.plugins,
      legend: {
        display: true,
        position: 'right',
        labels: { usePointStyle: true, padding: 15, font: { size: 11 } },
      },
    },
    cutout: '60%',
  }), []);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      <DoughnutChart data={{ labels, datasets }} options={options} />
    </div>
  );
}

// Radar Chart for vehicle comparison
interface RadarChartProps {
  vehicles: {
    label: string;
    data: number[];
    color: string;
  }[];
  labels: string[];
  height?: number;
  className?: string;
}

export const RadarChart: React.FC<RadarChartProps> = ({
  vehicles,
  labels,
  height = 300,
  className,
}) => {
  const datasets = useMemo(() => vehicles.map((v, i) => ({
    label: v.label,
    data: v.data,
    borderColor: v.color,
    backgroundColor: `${v.color}40`,
    borderWidth: 2,
    pointBackgroundColor: v.color,
    pointBorderColor: '#fff',
    pointHoverBackgroundColor: '#fff',
    pointHoverBorderColor: v.color,
    pointRadius: 4,
    pointHoverRadius: 6,
  }), [vehicles]);

  const options = useMemo<ChartOptions<'radar'>>(() => ({
    ...defaultOptions,
    plugins: { ...defaultOptions.plugins, legend: { display: true, position: 'top' } },
    scales: {
      r: {
        angleLines: { color: 'rgba(0, 0, 0, 0.1)' },
        grid: { color: 'rgba(0, 0, 0, 0.1)' },
        pointLabels: { font: { size: 11 } },
        suggestedMin: 0,
        suggestedMax: 100,
        ticks: { display: false },
      },
    },
  }), []);

  return (
    <div className={cn('w-full', className)} style={{ height }}>
      <RadarChart data={{ labels, datasets }} options={options} />
    </div>
  );
}

// Real-time updating chart hook
export function useRealTimeChart(maxPoints = 100) {
  const dataRef = useRef<ChartDataPoint[]>([]);
  const [, forceUpdate] = React.useReducer(x => x + 1, 0);

  const addPoint = useCallback((point: ChartDataPoint) => {
    dataRef.current = [...dataRef.current, point].slice(-maxPoints);
    forceUpdate();
  }, [maxPoints]);

  const getData = useCallback(() => dataRef.current, []);

  const clear = useCallback(() => {
    dataRef.current = [];
    forceUpdate();
  }, []);

  return { addPoint, getData, clear };
}

export const ChartComponents = {
  TimeSeriesChart,
  MultiAxisChart,
  BatteryChart,
  LinkQualityChart,
  FormationErrorCDFChart,
  DetectionDistributionChart,
  RadarChart,
  useRealTimeChart,
};