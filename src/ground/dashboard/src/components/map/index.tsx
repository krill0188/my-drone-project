// Map components using Mapbox GL JS and Deck.gl

import React, { useEffect, useRef, useState, useCallback } from 'react';
import mapboxgl from 'mapbox-gl';
import { MapboxOverlay } from '@deck.gl/react';
import { ScatterplotLayer, PathLayer, IconLayer, PolygonLayer } from '@deck.gl/layers';
import { cn } from '@/utils/helpers';
import type { DroneState, SwarmFormationState, GeoCoordinate, Vector3 } from '@/types';

mapboxgl.accessToken = import.meta.env.VITE_MAPBOX_TOKEN || '';

// Drone marker icons
const createDroneIcon = (color: string, isLeader: boolean) => {
  const svg = `
    <svg width="32" height="32" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg">
      <circle cx="16" cy="16" r="14" fill="${color}" opacity="0.9"/>
      ${isLeader ? '<polygon points="16,4 28,20 16,28 4,20" fill="gold"/>' : ''}
    </svg>
  `;
  return `data:image/svg+xml;base64,${btoa(svg)}`;
};

interface DroneMapProps {
  vehicles: DroneState[];
  formation?: SwarmFormationState | null;
  selectedVehicleId?: number | null;
  onVehicleClick?: (vehicle: DroneState) => void;
  viewport?: { center: [number, number]; zoom: number; bearing: number; pitch: number };
  onViewportChange?: (viewport: { center: [number, number]; zoom: number; bearing: number; pitch: number }) => void;
  showFormationPaths?: boolean;
  showCoverage?: boolean;
  coverageData?: any;
  className?: string;
}

export const DroneMap: React.FC<DroneMapProps> = ({
  vehicles,
  formation,
  selectedVehicleId,
  onVehicleClick,
  viewport,
  onViewportChange,
  showFormationPaths = true,
  showCoverage = false,
  coverageData,
  className,
}) => {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<mapboxgl.Map | null>(null);
  const [mapLoaded, setMapLoaded] = useState(false);
  const layersRef = useRef<mapboxgl.Layer[]>([]);

  // Initialize map
  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return;

    const map = new mapboxgl.Map({
      container: mapContainerRef.current,
      style: 'mapbox://styles/mapbox/satellite-streets-v12',
      center: viewport?.center || [-122.4194, 37.7749],
      zoom: viewport?.zoom || 14,
      bearing: viewport?.bearing || 0,
      pitch: viewport?.pitch || 0,
      antialias: true,
    });

    map.addControl(new mapboxgl.NavigationControl(), 'top-right');
    map.addControl(new mapboxgl.FullscreenControl(), 'top-right');
    map.addControl(
      new mapboxgl.GeolocateControl({
        positionOptions: { enableHighAccuracy: true },
        trackUserLocation: true,
        showUserHeading: true,
      }),
      'top-right'
    );

    map.on('load', () => {
      setMapLoaded(true);
      
      // Add formation path layer if formation exists
      if (formation && showFormationPaths) {
        addFormationLayers(map, formation);
      }
      
      // Add coverage layer
      if (showCoverage && coverageData) {
        addCoverageLayer(map, coverageData);
      }
    });

    map.on('moveend', () => {
      const center = map.getCenter();
      onViewportChange?.({
        center: [center.lng, center.lat],
        zoom: map.getZoom(),
        bearing: map.getBearing(),
        pitch: map.getPitch(),
      });
    });

    mapRef.current = map;

    return () => {
      map.remove();
      mapRef.current = null;
      setMapLoaded(false);
    };
  }, []);

  const addFormationLayers = (map: mapboxgl.Map, formation: SwarmFormationState) => {
    // Assigned positions polygon
    const assignedCoords = formation.member_ids.map(id => {
      const pos = formation.assigned_positions[id];
      return pos ? [pos.lon, pos.lat] : null;
    }).filter(Boolean);
    
    if (assignedCoords.length >= 3) {
      map.addSource('formation-assigned', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: {
            type: 'Polygon',
            coordinates: [[...assignedCoords, assignedCoords[0]]],
          },
        },
      });
      
      map.addLayer({
        id: 'formation-assigned',
        type: 'fill',
        source: 'formation-assigned',
        paint: {
          'fill-color': '#3b82f6',
          'fill-opacity': 0.2,
          'fill-outline-color': '#3b82f6',
        },
      });
      layersRef.current.push(map.getLayer('formation-assigned')!);
    }

    // Actual positions polygon
    const actualCoords = formation.member_ids.map(id => {
      const pos = formation.actual_positions[id];
      return pos ? [pos.lon, pos.lat] : null;
    }).filter(Boolean);
    
    if (actualCoords.length >= 3) {
      map.addSource('formation-actual', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: {
            type: 'Polygon',
            coordinates: [[...actualCoords, actualCoords[0]]],
          },
        },
      });
      
      map.addLayer({
        id: 'formation-actual',
        type: 'fill',
        source: 'formation-actual',
        paint: {
          'fill-color': '#22c55e',
          'fill-opacity': 0.15,
          'fill-outline-color': '#22c55e',
        },
      });
      layersRef.current.push(map.getLayer('formation-actual')!);
    }

    // Connection lines between assigned and actual
    const lineFeatures = formation.member_ids.flatMap(id => {
      const assigned = formation.assigned_positions[id];
      const actual = formation.actual_positions[id];
      if (assigned && actual) {
        return {
          type: 'Feature',
          geometry: {
            type: 'LineString',
            coordinates: [
              [assigned.lon, assigned.lat],
              [actual.lon, actual.lat],
            ],
          },
          properties: {
            error: formation.position_errors[id] || 0,
          },
        };
      }
      return [];
    });

    if (lineFeatures.length > 0) {
      map.addSource('formation-errors', {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: lineFeatures },
      });
      
      map.addLayer({
        id: 'formation-errors',
        type: 'line',
        source: 'formation-errors',
        paint: {
          'line-color': [
            'interpolate',
            ['linear'],
            ['get', 'error'],
            0, '#22c55e',
            2, '#f59e0b',
            5, '#ef4444',
          ],
          'line-width': 2,
          'line-opacity': 0.8,
        },
      });
      layersRef.current.push(map.getLayer('formation-errors')!);
    }
  };

  const addCoverageLayer = (map: mapboxgl.Map, coverageData: any) => {
    if (!coverageData.heatmap_data) return;

    // Add coverage as a heatmap layer using Deck.gl overlay would be better
    // For now, skip - would use Deck.gl ScatterplotLayer with heatmap aggregation
  };

  // Update drone markers when vehicles change
  useEffect(() => {
    if (!mapLoaded || !mapRef.current) return;

    const map = mapRef.current!;
    
    // Remove old drone layers
    ['drones', 'drone-labels', 'drone-selected'].forEach(id => {
      if (map.getLayer(id)) map.removeLayer(id);
      if (map.getSource(id)) map.removeSource(id);
    });

    // Add drone markers using GeoJSON
    const droneFeatures = vehicles.map(vehicle => ({
      type: 'Feature' as const,
      geometry: {
        type: 'Point' as const,
        coordinates: [vehicle.position.lon, vehicle.position.lat],
      },
      properties: {
        id: vehicle.id,
        callsign: vehicle.callsign,
        heading: vehicle.heading,
        isLeader: vehicle.is_leader,
        selected: vehicle.id === selectedVehicleId,
        altitude: vehicle.position.alt || 0,
        speed: vehicle.ground_speed,
        battery: vehicle.battery_remaining,
      },
    }));

    map.addSource('drones', {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: droneFeatures },
    });

    // Drone markers
    map.addLayer({
      id: 'drones',
      type: 'symbol',
      source: 'drones',
      layout: {
        'icon-image': 'drone-marker',
        'icon-size': 0.8,
        'icon-rotate': ['get', 'heading'],
        'icon-allow-overlap': true,
        'icon-ignore-placement': true,
      },
    });

    // Labels
    map.addLayer({
      id: 'drone-labels',
      type: 'symbol',
      source: 'drones',
      layout: {
        'text-field': '{callsign}',
        'text-font': ['Open Sans SemiBold', 'Arial Unicode MS Bold'],
        'text-size': 11,
        'text-offset': [0, 1.5],
        'text-anchor': 'top',
        'text-allow-overlap': true,
      },
      paint: {
        'text-color': '#ffffff',
        'text-halo-color': '#000000',
        'text-halo-width': 1.5,
      },
    });

    // Selected drone highlight
    if (selectedVehicleId !== null) {
      const selected = vehicles.find(v => v.id === selectedVehicleId);
      if (selected) {
        map.addSource('drone-selected', {
          type: 'geojson',
          data: {
            type: 'Feature',
            geometry: {
              type: 'Point',
              coordinates: [selected.position.lon, selected.position.lat],
            },
          },
        });
        
        map.addLayer({
          id: 'drone-selected',
          type: 'circle',
          source: 'drone-selected',
          paint: {
            'circle-radius': 20,
            'circle-color': '#3b82f6',
            'circle-opacity': 0.3,
            'circle-stroke-width': 2,
            'circle-stroke-color': '#3b82f6',
          },
        });
      }
    }

    // Add click handlers
    map.on('click', 'drones', (e) => {
      const feature = e.features?.[0];
      if (feature && onVehicleClick) {
        const vehicle = vehicles.find(v => v.id === feature.properties.id);
        if (vehicle) onVehicleClick(vehicle);
      }
    });

    map.on('mouseenter', 'drones', () => {
      map.getCanvas().style.cursor = 'pointer';
    });
    
    map.on('mouseleave', 'drones', () => {
      map.getCanvas().style.cursor = '';
    });

  }, [vehicles, selectedVehicleId, onVehicleClick, mapLoaded, formation, showFormationPaths]);

  return (
    <div 
      ref={mapContainerRef} 
      className={cn('w-full h-full rounded-xl overflow-hidden', className)}
      style={{ width: '100%', height: '100%' }}
    >
      {!mapLoaded && (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-900 z-10">
          <div className="text-center text-white">
            <Spinner size="lg" />
            <p className="mt-2">Loading map...</p>
          </div>
        </div>
      )}
    </div>
  );
};

// Deck.gl overlay for advanced visualizations
interface DeckGLOverlayProps {
  vehicles: DroneState[];
  formation?: SwarmFormationState | null;
  viewport: { center: [number, number]; zoom: number; bearing: number; pitch: number };
  onViewportChange: (viewport: any) => void;
}

export const DeckGLOverlay: React.FC<DeckGLOverlayProps> = ({
  vehicles,
  formation,
  viewport,
  onViewportChange,
}) => {
  const [viewState, setViewState] = useState({
    longitude: viewport.center[0],
    latitude: viewport.center[1],
    zoom: viewport.zoom,
    bearing: viewport.bearing,
    pitch: viewport.pitch,
  });

  // Drone scatterplot layer
  const droneLayer = new ScatterplotLayer({
    id: 'drones',
    data: vehicles,
    getPosition: (d: DroneState) => [d.position.lon, d.position.lat, d.position.alt || 0],
    getRadius: 15,
    getFillColor: (d: DroneState) => 
      d.is_leader ? [255, 215, 0, 200] : 
      d.battery_remaining < 20 ? [239, 68, 68, 200] :
      d.battery_remaining < 50 ? [245, 158, 11, 200] :
      [34, 197, 94, 200],
    getLineColor: [255, 255, 255, 255],
    lineWidthMinPixels: 2,
    pickable: true,
    autoHighlight: true,
    onHover: ({ object, x, y }) => {
      // Tooltip handled by parent
    },
  });

  // Formation path layer
  const formationPathLayer = formation ? new PathLayer({
    id: 'formation-paths',
    data: formation.member_ids.flatMap(id => {
      const assigned = formation.assigned_positions[id];
      const actual = formation.actual_positions[id];
      if (assigned && actual) {
        return {
          path: [
            [assigned.lon, assigned.lat, 0],
            [actual.lon, actual.lat, 0],
          ],
          error: formation.position_errors[id] || 0,
        };
      }
      return [];
    }),
    getPath: (d: any) => d.path,
    getColor: (d: any) => 
      d.error < 2 ? [34, 197, 94, 200] :
      d.error < 5 ? [245, 158, 11, 200] :
      [239, 68, 68, 200],
    getWidth: 3,
    pickable: true,
  }) : null;

  // Coverage heatmap layer
  const coverageLayer = false && new ScatterplotLayer({
    id: 'coverage-heatmap',
    data: [], // Would be coverage grid points
    getPosition: (d: any) => [d.lon, d.lat],
    getRadius: 50,
    getFillColor: (d: any) => [
      255 * (1 - d.coverage),
      255 * d.coverage,
      0,
      150,
    ],
    radiusMinPixels: 5,
    radiusMaxPixels: 50,
  });

  return (
    <MapboxOverlay
      viewState={viewState}
      onViewStateChange={({ viewState }) => {
        setViewState(viewState);
        onViewportChange({
          center: [viewState.longitude, viewState.latitude],
          zoom: viewState.zoom,
          bearing: viewState.bearing,
          pitch: viewState.pitch,
        });
      }}
      layers={[
        droneLayer,
        formationPathLayer,
        coverageLayer,
      ].filter(Boolean) as any}
    />
  );
};

// Coverage heatmap component
interface CoverageHeatmapProps {
  coverageData: any;
  viewport: { center: [number, number]; zoom: number };
}

export const CoverageHeatmap: React.FC<CoverageHeatmapProps> = ({ coverageData, viewport }) => {
  // Would use Deck.gl with a heatmap layer or a custom Mapbox layer
  // For now, return a placeholder
  return (
    <div className="absolute inset-0">
      <canvas className="w-full h-full" />
    </div>
  );
};

export const MapComponents = {
  DroneMap,
  DeckGLOverlay,
  CoverageHeatmap,
};