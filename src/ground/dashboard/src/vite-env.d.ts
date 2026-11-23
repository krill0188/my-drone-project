/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_MAPBOX_TOKEN: string;
  readonly VITE_API_URL: string;
  readonly VITE_WS_URL: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}

// Extend Window for Mapbox
interface Window {
  mapboxgl: any;
  mapboxToken: string;
}

// Extend for react-hot-toast
declare module 'react-hot-toast' {
  interface ToastOptions {
    duration?: number;
    style?: React.CSSProperties;
    className?: string;
  }
}

// Deck.gl types
declare module '@deck.gl/react' {
  export interface MapboxOverlayProps {
    viewState: any;
    onViewStateChange: (params: { viewState: any }) => void;
    layers: any[];
    interleaved?: boolean;
  }
  export const MapboxOverlay: React.FC<MapboxOverlayProps>;
}

declare module '@deck.gl/layers' {
  export class ScatterplotLayer {
    constructor(props: any);
  }
  export class PathLayer {
    constructor(props: any);
  }
  export class PolygonLayer {
    constructor(props: any);
  }
  export class IconLayer {
    constructor(props: any);
  }
}

// Mapbox GL types
declare module 'mapbox-gl' {
  export class Map {
    constructor(options: any);
    addControl(control: any, position: string): this;
    addSource(id: string, source: any): this;
    addLayer(layer: any, before?: string): this;
    removeLayer(id: string): this;
    removeSource(id: string): this;
    getLayer(id: string): any;
    getSource(id: string): any;
    on(event: string, layer: string, callback: (e: any) => void): this;
    off(event: string, layer: string, callback: (e: any) => void): this;
    getCenter(): { lng: number; lat: number };
    getZoom(): number;
    getBearing(): number;
    getPitch(): number;
    getCanvas(): HTMLCanvasElement;
    once(event: string, callback: () => void): this;
    on(event: string, callback: () => void): this;
  }

  export class NavigationControl {
    constructor(options?: any);
  }

  export class FullscreenControl {
    constructor(options?: any);
  }

  export class GeolocateControl {
    constructor(options?: any);
  }

  export function accessToken(token: string): void;
}

// react-chartjs-2
declare module 'react-chartjs-2' {
  export const Chart: React.FC<any>;
  export const LineChart: React.FC<any>;
  export const BarChart: React.FC<any>;
  export const DoughnutChart: React.FC<any>;
  export const RadarChart: React.FC<any>;
  export const ScatterChart: React.FC<any>;
  export const AreaChart: React.FC<any>;
  export const PieChart: React.FC<any>;
  export const PolarAreaChart: React.FC<any>;
  export const BubbleChart: React.FC<any>;
}

// Chart.js
declare module 'chart.js' {
  export class Chart {
    constructor(ctx: CanvasRenderingContext2D | HTMLCanvasElement, config: any);
    destroy(): void;
    update(mode?: string): void;
    reset(): void;
    clear(): void;
    toBase64Image(type?: string, encoderOptions?: number): string;
    static register(...components: any[]): void;
  }

  export class CategoryScale {}
  export class LinearScale {}
  export class LogarithmicScale {}
  export class PointElement {}
  export class LineElement {}
  export class BarElement {}
  export class ArcElement {}
  export class Title {}
  export class Tooltip {}
  export class Legend {}
  export class Filler {}
  export class Decimation {}

  export interface ChartOptions<T> {
    responsive?: boolean;
    maintainAspectRatio?: boolean;
    interaction?: any;
    plugins?: any;
    scales?: any;
    elements?: any;
    layout?: any;
    animation?: any;
    locale?: string;
  }

  export interface ChartData<T> {
    labels?: any[];
    datasets: any[];
  }
}

// Zustand
declare module 'zustand' {
  export function create<T>(createState: (set: any, get: any, api: any) => T): any;
  export function subscribeWithSelector<T>(config: any): any;
  export function devtools<T>(config: any): any;
  export function persist<T>(config: any, options: any): any;
}

declare module 'zustand/middleware' {
  export function subscribeWithSelector<T>(config: any): any;
  export function devtools<T>(config: any): any;
  export function persist<T>(config: any, options: any): any;
}

// react-router-dom
declare module 'react-router-dom' {
  export function BrowserRouter(props: any): React.ReactElement;
  export function Routes(props: any): React.ReactElement;
  export function Route(props: any): React.ReactElement;
  export function Navigate(props: any): React.ReactElement;
  export function useNavigate(): any;
  export function useLocation(): any;
  export function useParams(): any;
  export function useSearchParams(): any;
  export function Link(props: any): React.ReactElement;
  export function Outlet(props: any): React.ReactElement;
}

// @tanstack/react-query
declare module '@tanstack/react-query' {
  export function QueryClientProvider(props: any): React.ReactElement;
  export function useQuery(options: any): any;
  export function useMutation(options: any): any;
  export function useQueryClient(): any;
  export class QueryClient {
    constructor(config?: any);
    invalidateQueries(options?: any): Promise<void>;
    setQueryData(key: any, data: any): void;
    getQueryData(key: any): any;
    removeQueries(options?: any): void;
  }
  export function QueryClientProvider(props: { client: QueryClient; children: React.ReactNode }): React.ReactElement;
}

// Lucide React
declare module 'lucide-react' {
  export const Activity: React.FC<any>;
  export const AlertCircle: React.FC<any>;
  export const AlertTriangle: React.FC<any>;
  export const ArrowDownRight: React.FC<any>;
  export const ArrowUpRight: React.FC<any>;
  export const Battery: React.FC<any>;
  export const Bell: React.FC<any>;
  export const BellOff: React.FC<any>;
  export const Calendar: React.FC<any>;
  export const CheckCircle: React.FC<any>;
  export const ChevronLeft: React.FC<any>;
  export const ChevronRight: React.FC<any>;
  export const Clock: React.FC<any>;
  export const Cpu: React.FC<any>;
  export const Download: React.FC<any>;
  export const Drone: React.FC<any>;
  export const Eye: React.FC<any>;
  export const EyeOff: React.FC<any>;
  export const FastForward: React.FC<any>;
  export const FileText: React.FC<any>;
  export const Filter: React.FC<any>;
  export const HardDrive: React.FC<any>;
  export const Home: React.FC<any>;
  export const Image: React.FC<any>;
  export const Layers: React.FC<any>;
  export const MapPin: React.FC<any>;
  export const Maximize: React.FC<any>;
  export const Minimize: React.FC<any>;
  export const Minus: React.FC<any>;
  export const Network: React.FC<any>;
  export const Pause: React.FC<any>;
  export const Play: React.FC<any>;
  export const Plus: React.FC<any>;
  export const RefreshCw: React.FC<any>;
  export const Rewind: React.FC<any>;
  export const RotateCcw: React.FC<any>;
  export const Satellite: React.FC<any>;
  export const Settings: React.FC<any>;
  export const Shield: React.FC<any>;
  export const Signal: React.FC<any>;
  export const SkipBack: React.FC<any>;
  export const SkipForward: React.FC<any>;
  export const Stop: React.FC<any>;
  export const Trash2: React.FC<any>;
  export const Upload: React.FC<any>;
  export const Users: React.FC<any>;
  export const Video: React.FC<any>;
  export const Volume2: React.FC<any>;
  export const VolumeX: React.FC<any>;
  export const Wifi: React.FC<any>;
  export const XCircle: React.FC<any>;
  export const Zap: React.FC<any>;
}

// React Hot Toast
declare module 'react-hot-toast' {
  export const Toaster: React.FC<any>;
  export const toast: any;
  export interface ToastOptions {
    duration?: number;
    style?: React.CSSProperties;
    className?: string;
    icon?: React.ReactNode;
  }
}

// clsx
declare module 'clsx' {
  export function clsx(...inputs: any[]): string;
}

// tailwind-merge
declare module 'tailwind-merge' {
  export function twMerge(...inputs: string[]): string;
}

// date-fns
declare module 'date-fns' {
  export function format(date: Date | number, formatString: string, options?: any): string;
  export function formatDistanceToNow(date: Date | number, options?: any): string;
  export function formatRelativeTime(date: Date | number, baseDate: Date | number, options?: any): string;
  export function addDays(date: Date | number, amount: number): Date;
  export function subDays(date: Date | number, amount: number): Date;
  export function startOfDay(date: Date | number): Date;
  export function endOfDay(date: Date | number): Date;
  export function startOfWeek(date: Date | number, options?: any): Date;
  export function endOfWeek(date: Date | number, options?: any): Date;
  export function startOfMonth(date: Date | number): Date;
  export function endOfMonth(date: Date | number): Date;
  export function startOfYear(date: Date | number): Date;
  export function endOfYear(date: Date | number): Date;
  export function isToday(date: Date | number): boolean;
  export function isYesterday(date: Date | number): boolean;
  export function isTomorrow(date: Date | number): boolean;
  export function isPast(date: Date | number): boolean;
  export function isFuture(date: Date | number): boolean;
  export function differenceInDays(dateLeft: Date | number, dateRight: Date | number): number;
  export function differenceInHours(dateLeft: Date | number, dateRight: Date | number): number;
  export function differenceInMinutes(dateLeft: Date | number, dateRight: Date | number): number;
  export function differenceInSeconds(dateLeft: Date | number, dateRight: Date | number): number;
  export function parseISO(dateString: string): Date;
  export function formatISO(date: Date | number, options?: any): string;
}

// react-query types
declare module '@tanstack/react-query' {
  export interface UseQueryOptions<TData, TError, TData, TQueryKey> {
    queryKey: TQueryKey;
    queryFn: () => Promise<TData>;
    enabled?: boolean;
    staleTime?: number;
    cacheTime?: number;
    refetchOnWindowFocus?: boolean;
    refetchInterval?: number | false;
    retry?: number | boolean;
    retryDelay?: number | ((retryAttempt: number) => number);
    onSuccess?: (data: TData) => void;
    onError?: (error: TError) => void;
    onSettled?: (data: TData | undefined, error: TError | null) => void;
    select?: (data: TData) => TData;
  }

  export interface UseMutationOptions<TData, TError, TVariables, TContext> {
    mutationFn: (variables: TVariables) => Promise<TData>;
    onMutate?: (variables: TVariables) => Promise<TContext | void>;
    onSuccess?: (data: TData, variables: TVariables, context: TContext) => void;
    onError?: (error: TError, variables: TVariables, context: TContext | undefined) => void;
    onSettled?: (data: TData | undefined, error: TError | null, variables: TVariables, context: TContext | undefined) => void;
    retry?: number | boolean;
    retryDelay?: number | ((retryAttempt: number) => number);
  }
}

// TanStack Query v5
declare module '@tanstack/react-query' {
  export interface QueryClientConfig {
    defaultOptions?: {
      queries?: Partial<UseQueryOptions<any, any, any, any>>;
      mutations?: Partial<UseMutationOptions<any, any, any, any>>;
    };
  }
}