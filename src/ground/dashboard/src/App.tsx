// Main App component with routing

import React, { Suspense, lazy } from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';
import { Toaster } from 'react-hot-toast';
import { useSwarmStore } from '@/store/swarmStore';
import { cn } from '@/utils/helpers';
import '@/styles/globals.css';

// Lazy load page components
const LiveView = lazy(() => import('@/pages/live/LiveView').then(m => ({ default: m.LiveView })));
const ReplayView = lazy(() => import('@/pages/replay/ReplayView').then(m => ({ default: m.ReplayView })));
const AnalyticsView = lazy(() => import('@/pages/analytics/AnalyticsView').then(m => ({ default: m.AnalyticsView })));

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: 1000 * 30, // 30 seconds
      retry: 1,
      refetchOnWindowFocus: false,
    },
  },
});

// Loading fallback
const PageLoading = () => (
  <div className="flex h-screen w-full items-center justify-center bg-gray-50">
    <div className="text-center">
      <div className="inline-flex items-center justify-center w-12 h-12 rounded-full bg-primary-100 mb-4">
        <div className="w-6 h-6 border-4 border-primary-600 border-t-transparent rounded-full animate-spin" />
      </div>
      <p className="text-gray-500">Loading...</p>
    </div>
  </div>
);

// Main layout with sidebar navigation
const DashboardLayout: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { activeTab, setActiveTab, sidebarOpen, toggleSidebar } = useSwarmStore();

  const navItems = [
    { id: 'live', label: 'Live View', icon: '📡', href: '/live' },
    { id: 'replay', label: 'Replay', icon: '⏮️', href: '/replay' },
    { id: 'analytics', label: 'Analytics', icon: '📊', href: '/analytics' },
  ];

  return (
    <div className="h-screen w-full flex bg-gray-50">
      {/* Sidebar */}
      <aside className={cn(
        'fixed inset-y-0 left-0 z-40 bg-white border-r border-gray-200 transition-all duration-300 flex flex-col',
        sidebarOpen ? 'w-72' : 'w-20'
      )}>
        <div className="flex flex-col h-full">
          {/* Logo & Toggle */}
          <div className="flex items-center justify-between p-4 border-b border-gray-200">
            {sidebarOpen && (
              <div className="flex items-center gap-2">
                <div className="w-8 h-8 rounded-lg bg-primary-600 flex items-center justify-center">
                  <span className="text-white font-bold text-sm">SD</span>
                </div>
                <span className="font-semibold text-gray-900">SwarmOps</span>
              </div>
            )}
            <button
              onClick={toggleSidebar}
              className="p-2 rounded-lg hover:bg-gray-100 text-gray-500 hover:text-gray-700 transition-colors"
              aria-label={sidebarOpen ? 'Collapse sidebar' : 'Expand sidebar'}
            >
              {sidebarOpen ? '◀' : '▶'}
            </button>
          </div>

          {/* Navigation */}
          <nav className="flex-1 p-4 space-y-1">
            {navItems.map(item => (
              <a
                key={item.id}
                href={item.href}
                className={cn(
                  'flex items-center gap-3 px-3 py-2 rounded-lg text-left transition-colors',
                  'hover:bg-gray-50',
                  activeTab === item.id
                    ? 'bg-primary-50 text-primary-700 font-medium'
                    : 'text-gray-600'
                )}
              >
                <span className="text-xl">{item.icon}</span>
                {sidebarOpen && <span className="font-medium">{item.label}</span>}
              </a>
            ))}
          </nav>

          {/* Connection Status */}
          <div className="p-4 border-t border-gray-200">
            <div className="flex items-center gap-2 mb-2">
              <div className={cn(
                'w-2 h-2 rounded-full',
                'bg-success-500'
              )} />
              <span className="text-sm font-medium text-success-700">Connected</span>
            </div>
            <div className="text-xs text-gray-500">
              Last sync: Just now
            </div>
          </div>
        </div>
      </aside>

      {/* Main Content */}
      <main className={cn('flex-1 flex flex-col overflow-hidden', sidebarOpen ? 'ml-72' : 'ml-20')}>
        {children}
      </main>
    </div>
  );
};

// App Routes
const AppRoutes = () => {
  const { setActiveTab } = useSwarmStore();

  return (
    <Routes>
      <Route path="/live" element={
        <DashboardLayout>
          <Suspense fallback={<PageLoading />}>
            <LiveView />
          </Suspense>
        </DashboardLayout>
      } />
      <Route path="/replay" element={
        <DashboardLayout>
          <Suspense fallback={<PageLoading />}>
            <ReplayView />
          </Suspense>
        </DashboardLayout>
      } />
      <Route path="/analytics" element={
        <DashboardLayout>
          <Suspense fallback={<PageLoading />}>
            <AnalyticsView />
          </Suspense>
        </DashboardLayout>
      } />
      <Route path="/" element={<Navigate to="/live" replace />} />
      <Route path="*" element={<Navigate to="/live" replace />} />
    </Routes>
  );
};

// Main App Component
const App = () => {
  return (
    <QueryClientProvider client={queryClient}>
      <BrowserRouter>
        <div className="h-screen w-full">
          <Suspense fallback={<PageLoading />}>
            <AppRoutes />
          </Suspense>
          <Toaster
            position="top-right"
            toastOptions={{
              duration: 4000,
              style: {
                background: '#363636',
                color: '#fff',
                borderRadius: '8px',
                padding: '12px 16px',
              },
              success: {
                iconTheme: {
                  primary: '#22c55e',
                  secondary: '#fff',
                },
              },
              error: {
                iconTheme: {
                  primary: '#ef4444',
                  secondary: '#fff',
                },
              },
            }}
          />
        </BrowserRouter>
      </QueryClientProvider>
    </div>
  );
};

export default App;