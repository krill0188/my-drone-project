#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <cmath>
#include <vector>
#include <algorithm>

class TestCoverageAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testHaversineDistance() {
        auto haversine = [](double lat1, double lon1, double lat2, double lon2) {
            const double R = 6371000;
            double dlat = (lat2 - lat1) * M_PI / 180.0;
            double dlon = (lon2 - lon1) * M_PI / 180.0;
            double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dlon/2)*sin(dlon/2);
            return 2 * R * atan2(sqrt(a), sqrt(1-a));
        };
        
        // Same point
        double d1 = haversine(37.7749, -122.4194, 37.7749, -122.4194);
        QVERIFY(qFuzzyCompare(d1, 0.0));
        
        // ~111km (1 degree latitude)
        double d2 = haversine(37.7749, -122.4194, 38.7749, -122.4194);
        QVERIFY(qFuzzyCompare(d2, 111195.0, 1000.0));
        
        // ~111km * cos(lat) at lat 37.77
        double d3 = haversine(37.7749, -122.4194, 37.7749, -121.4194);
        double expected = 111195.0 * cos(37.7749 * M_PI / 180.0);
        QVERIFY(qFuzzyCompare(d3, expected, 1000.0));
    }
    
    void testFootprintRadius() {
        auto footprint_radius = [](double flight_alt, double sensor_fov_deg) {
            return flight_alt * tan(sensor_fov_deg * M_PI / 360.0);
        };
        
        // 50m altitude, 90 deg FOV
        double r1 = footprint_radius(50, 90);
        QVERIFY(qFuzzyCompare(r1, 50.0, 0.1));  // tan(45) = 1
        
        // 100m altitude, 60 deg FOV
        double r2 = footprint_radius(100, 60);
        QVERIFY(qFuzzyCompare(r2, 100 * tan(M_PI/6), 0.1));  // tan(30) = 1/sqrt(3)
    }
    
    void testGridCellCalculation() {
        auto grid_cell = [](double lat, double lon, double min_lat, double min_lon, double cell_size_m) {
            double lat_m = (lat - min_lat) * 111000;
            double lon_m = (lon - min_lon) * 111000 * cos(min_lat * M_PI / 180.0);
            return std::pair<int, int>(int(lat_m / cell_size_m), int(lon_m / cell_size_m));
        };
        
        // Same as min
        auto c1 = grid_cell(37.7749, -122.4194, 37.7749, -122.4194, 10);
        QCOMPARE(c1.first, 0);
        QCOMPARE(c1.second, 0);
        
        // 0.001 deg lat ~ 111m
        auto c2 = grid_cell(37.7759, -122.4194, 37.7749, -122.4194, 10);
        QCOMPARE(c2.first, 11);
    }
    
    void testCoverageCalculation() {
        // Simulate flight path
        std::vector<std::pair<double, double>> coords = {
            {0, 0}, {10, 0}, {20, 0}, {20, 10}, {10, 10}, {0, 10}, {0, 0}
        };
        
        double grid_resolution = 5.0;
        double footprint_radius = 10.0;
        int footprint_cells = int(footprint_radius / grid_resolution);
        
        int width = 5, height = 5;
        std::vector<std::vector<int>> grid(height, std::vector<int>(width, 0));
        
        for (auto [x, y] : coords) {
            int gx = int(x / grid_resolution);
            int gy = int(y / grid_resolution);
            
            for (int dx = -footprint_cells; dx <= footprint_cells; ++dx) {
                for (int dy = -footprint_cells; dy <= footprint_cells; ++dy) {
                    int nx = gx + dx, ny = gy + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        if (dx*dx + dy*dy <= footprint_cells*footprint_cells) {
                            grid[ny][nx] = 1;
                        }
                    }
                }
            }
            
            int covered = 0;
            for (auto& row : grid) {
                for (auto cell : row) {
                    if (cell) covered++;
                }
            }
            
            int total = width * height;
            double coverage_pct = covered * 100.0 / total;
            
            QVERIFY(coverage_pct > 0);
            QVERIFY(coverage_pct <= 100);
        }
    }
    
    void testLocalCoordinates() {
        // Test with local NED coordinates
        std::vector<std::pair<double, double>> coords = {
            {0, 0}, {100, 0}, {200, 0}, {200, 100}, {100, 100}, {0, 100}
        };
        
        double min_x = 0, max_x = 200;
        double min_y = 0, max_y = 100;
        double grid_res = 10.0;
        
        int width = int((max_x - min_x) / grid_res) + 1;
        int height = int((max_y - min_y) / grid_res) + 1;
        
        QCOMPARE(width, 21);
        QCOMPARE(height, 11);
    }
};

QTEST_MAIN(TestCoverageAnalyzer)
#include "test_coverage_analyzer.moc"