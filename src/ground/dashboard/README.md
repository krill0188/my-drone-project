# AI Swarm Autonomous Drone System - Web Dashboard

Real-time monitoring and analytics dashboard for the AI Swarm Autonomous Drone System.

## Features

- **Live View** - Real-time swarm monitoring with map visualization
- **Replay** - Flight replay with timeline scrubbing
- **Analytics** - Fleet analytics, formation analysis, AI perception metrics, QoS analysis
- **Reports** - Automated post-flight report generation

## Tech Stack

- **Framework**: React 18 + TypeScript
- **Build Tool**: Vite 5
- **Styling**: Tailwind CSS 3
- **State Management**: Zustand
- **Data Fetching**: TanStack Query (React Query)
- **Charts**: Chart.js + react-chartjs-2
- **Mapping**: Mapbox GL JS + Deck.gl
- **Real-time**: WebSocket
- **Testing**: Vitest

## Project Structure

```
src/
├── components/
│   ├── common/          # Reusable UI components
│   ├── charts/          # Chart components (Chart.js)
│   ├── map/             # Map components (Mapbox/Deck.gl)
│   └── panels/          # Panel components
├── pages/
│   ├── live/            # Live view page
│   ├── replay/          # Replay page
│   └── analytics/       # Analytics page
├── hooks/               # Custom React hooks
├── store/               # Zustand store
├── types/               # TypeScript types
├── utils/               # Utility functions
├── styles/              # Global styles
├── App.tsx              # Main app component
└── main.tsx             # Entry point
```

## Getting Started

### Prerequisites

- Node.js 18+
- npm 9+
- Mapbox Access Token (for map features)

### Installation

```bash
cd src/ground/dashboard
npm install
```

### Environment Variables

Create a `.env` file:

```env
VITE_MAPBOX_TOKEN=your_mapbox_access_token
VITE_API_URL=http://localhost:8080/api
VITE_WS_URL=ws://localhost:8080/ws
```

### Development

```bash
npm run dev
```

Open http://localhost:3000

### Build

```bash
npm run build
```

### Preview Production Build

```bash
npm run preview
```

### Linting

```bash
npm run lint
```

### Testing

```bash
npm run test
```

## Mapbox Setup

1. Create a Mapbox account at https://mapbox.com
2. Create an access token with the following scopes:
   - `styles:read`
   - `styles:write`
   - `tokens:read`
3. Add the token to `.env` as `VITE_MAPBOX_TOKEN`

## WebSocket API

The dashboard connects to a WebSocket server for real-time data:

```javascript
// Connection
const ws = new WebSocket('ws://localhost:8080/ws');

// Message format
{
  "type": "vehicle_update",
  "payload": { /* DroneState */ },
  "timestamp": "2025-01-15T10:30:00.000Z"
}
```

### Message Types

- `vehicle_update` - Drone state update
- `formation_update` - Formation state update
- `mesh_qos_update` - Mesh QoS metrics update
- `safety_event` - Safety event notification
- `swarm_summary` - Fleet summary update

## Customization

### Theme Colors

Edit `tailwind.config.js` to customize the color palette:

```js
theme: {
  extend: {
    colors: {
      primary: { ... },
      danger: { ... },
      success: { ... },
      warning: { ... },
    },
  },
}
```

### Adding New Pages

1. Create page component in `src/pages/`
2. Add route in `App.tsx`
3. Add navigation item in `DashboardLayout`

## Deployment

### Docker

```dockerfile
FROM node:18-alpine AS builder
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
```

### Kubernetes

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: swarm-dashboard
spec:
  replicas: 3
  selector:
    matchLabels:
      app: swarm-dashboard
  template:
    metadata:
      labels:
        app: swarm-dashboard
    spec:
      containers:
      - name: dashboard
        image: swarm-dashboard:latest
        ports:
        - containerPort: 80
---
apiVersion: v1
kind: Service
metadata:
  name: swarm-dashboard
spec:
  selector:
    app: swarm-dashboard
  ports:
  - port: 80
    targetPort: 80
  type: LoadBalancer
```

## Performance Optimization

- Code splitting with React.lazy()
- React Query for caching
- Memoization with React.memo, useMemo, useCallback
- Virtualized lists for large datasets
- Web Workers for heavy computations

## Browser Support

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## License

BSD-3-Clause - See LICENSE file for details.