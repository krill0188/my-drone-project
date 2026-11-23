import { clsx, type ClassValue } from 'clsx';
import { twMerge } from 'tailwind-merge';

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

export function formatDuration(seconds: number): string {
  if (seconds < 60) return `${seconds.toFixed(0)}s`;
  if (seconds < 3600) return `${(seconds / 60).toFixed(1)}m`;
  if (seconds < 86400) return `${(seconds / 3600).toFixed(1)}h`;
  return `${(seconds / 86400).toFixed(1)}d`;
}

export function formatDistance(meters: number): string {
  if (meters < 1000) return `${meters.toFixed(0)}m`;
  return `${(meters / 1000).toFixed(1)}km`;
}

export function formatSpeed(mps: number): string {
  return `${(mps * 3.6).toFixed(1)} km/h`;
}

export function formatAltitude(meters: number): string {
  return `${meters.toFixed(1)}m`;
}

export function formatPercentage(value: number, decimals = 1): string {
  return `${(value * 100).toFixed(decimals)}%`;
}

export function formatTimestamp(timestamp: string | number): string {
  const date = typeof timestamp === 'string' ? new Date(timestamp) : new Date(timestamp);
  return date.toLocaleString();
}

export function formatTimeAgo(timestamp: string | number): string {
  const date = typeof timestamp === 'string' ? new Date(timestamp) : new Date(timestamp);
  const now = new Date();
  const diffMs = now.getTime() - date.getTime();
  const diffSecs = Math.floor(diffMs / 1000);
  
  if (diffSecs < 60) return 'just now';
  if (diffSecs < 3600) return `${Math.floor(diffSecs / 60)}m ago`;
  if (diffSecs < 86400) return `${Math.floor(diffSecs / 3600)}h ago`;
  return `${Math.floor(diffSecs / 86400)}d ago`;
}

export function getSeverityColor(severity: string): string {
  switch (severity.toLowerCase()) {
    case 'critical': return 'text-danger-600 bg-danger-50 border-danger-200';
    case 'error': return 'text-danger-600 bg-danger-50 border-danger-200';
    case 'warn': return 'text-warning-600 bg-warning-50 border-warning-200';
    case 'warning': return 'text-warning-600 bg-warning-50 border-warning-200';
    case 'info': return 'text-primary-600 bg-primary-50 border-primary-200';
    default: return 'text-gray-600 bg-gray-50 border-gray-200';
  }
}

export function getSeverityBadgeColor(severity: string): string {
  switch (severity.toLowerCase()) {
    case 'critical': return 'bg-danger-100 text-danger-700';
    case 'error': return 'bg-danger-100 text-danger-700';
    case 'warn': return 'bg-warning-100 text-warning-700';
    case 'warning': return 'bg-warning-100 text-warning-700';
    case 'info': return 'bg-primary-100 text-primary-700';
    default: return 'bg-gray-100 text-gray-700';
  }
}

export function getBatteryColor(percentage: number): string {
  if (percentage > 50) return 'text-success-600';
  if (percentage > 20) return 'text-warning-600';
  return 'text-danger-600';
}

export function getLinkQualityColor(quality: number): string {
  if (quality > 0.7) return 'text-success-600';
  if (quality > 0.4) return 'text-warning-600';
  return 'text-danger-600';
}

export function getRSSIColor(rssi: number): string {
  if (rssi > -70) return 'text-success-600';
  if (rssi > -85) return 'text-warning-600';
  return 'text-danger-600';
}

export function getLatencyColor(latency: number): string {
  if (latency < 50) return 'text-success-600';
  if (latency < 100) return 'text-warning-600';
  return 'text-danger-600';
}

export function generateId(): string {
  return Math.random().toString(36).substring(2, 15) + 
         Math.random().toString(36).substring(2, 15);
}

export function debounce<T extends (...args: any[]) => any>(
  func: T,
  wait: number
): (...args: Parameters<T>) => void {
  let timeout: NodeJS.Timeout | null = null;
  
  return (...args: Parameters<T>) => {
    if (timeout) clearTimeout(timeout);
    timeout = setTimeout(() => func(...args), wait);
  };
}

export function throttle<T extends (...args: any[]) => any>(
  func: T,
  limit: number
): (...args: Parameters<T>) => void {
  let inThrottle = false;
  
  return (...args: Parameters<T>) => {
    if (!inThrottle) {
      func(...args);
      inThrottle = true;
      setTimeout(() => (inThrottle = false), limit);
    }
  };
}

export function haversineDistance(
  lat1: number, lon1: number, 
  lat2: number, lon2: number
): number {
  const R = 6371000; // Earth radius in meters
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = 
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

export function calculateBearing(
  lat1: number, lon1: number, 
  lat2: number, lon2: number
): number {
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const lat1Rad = lat1 * Math.PI / 180;
  const lat2Rad = lat2 * Math.PI / 180;
  
  const y = Math.sin(dLon) * Math.cos(lat2Rad);
  const x = Math.cos(lat1Rad) * Math.sin(lat2Rad) - 
            Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLon);
  
  const bearing = Math.atan2(y, x) * 180 / Math.PI;
  return (bearing + 360) % 360;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

export function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

export function parseMAVLinkMode(mode: number): string {
  const modes: Record<number, string> = {
    0: 'MANUAL',
    1: 'CIRCLE',
    2: 'STABILIZE',
    3: 'TRAINING',
    4: 'ACRO',
    5: 'FLY_BY_WIRE_A',
    6: 'FLY_BY_WIRE_B',
    7: 'CRUISE',
    8: 'AUTOTUNE',
    10: 'AUTO',
    11: 'RTL',
    12: 'LOITER',
    13: 'TAKEOFF',
    14: 'LAND',
    15: 'GUIDED',
    16: 'INITIALISING',
    17: 'QSTABILIZE',
    18: 'QHOVER',
    19: 'QLOITER',
    20: 'QLAND',
    21: 'QRTL',
    22: 'QAUTOTUNE',
    23: 'QACRO',
    24: 'THERMAL',
    25: 'LOITER_ALT',
    26: 'AVOID_ADSB',
    27: 'GUIDED_NOGPS',
    28: 'SMART_RTL',
    29: 'FLOWHOLD',
    30: 'FOLLOW',
    31: 'ZIGZAG',
    32: 'SYSTEMID',
    33: 'AUTOROTATE',
    34: 'AUTO_RTL',
  };
  return modes[mode] || `UNKNOWN(${mode})`;
}

export function parsePX4Mode(mode: string): string {
  // PX4 mode strings are typically human-readable
  return mode;
}