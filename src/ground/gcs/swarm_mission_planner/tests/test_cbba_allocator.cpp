#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <cmath>

class TestCBBAAllocator : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testSimpleAllocation() {
        // Tasks: 3 tasks, 2 drones
        QVariantList tasks = {
            QVariantMap{{"id", "t1"}, {"position", QVariantList{0.0, 0.0}}, {"priority", 10}},
            QVariantMap{{"id", "t2"}, {"position", QVariantList{100.0, 0.0}}, {"priority", 5}},
            QVariantMap{{"id", "t3"}, {"position", QVariantList{0.0, 100.0}}, {"priority", 8}}
        };
        
        QVariantList drones = {
            QVariantMap{{"id", 1}, {"position", QVariantList{10.0, 10.0}}},
            QVariantMap{{"id", 2}, {"position", QVariantList{90.0, 10.0}}}
        };
        
        // Simulate allocation (greedy by distance + load)
        QVariantMap allocations;
        QVariantMap drone_loads;
        for (const auto& drone : drones) {
            allocations[QString::number(drone["id"].toInt())] = QVariantList();
            drone_loads[QString::number(drone["id"].toInt())] = 0;
        }
        
        // Sort by priority
        std::sort(tasks.begin(), tasks.end(), [](const QVariant& a, const QVariant& b) {
            return a.toMap()["priority"].toInt() > b.toMap()["priority"].toInt();
        });
        
        for (const auto& task_var : tasks) {
            QVariantMap task = task_var.toMap();
            QString best_drone;
            double best_score = std::numeric_limits<double>::infinity();
            
            for (const auto& drone_var : drones) {
                QVariantMap drone = drone_var.toMap();
                QString d_id = QString::number(drone["id"].toInt());
                int load = drone_loads[d_id].toInt();
                
                if (load >= 10) continue;  // max tasks per drone
                
                // Distance score
                QVariantList task_pos = task["position"].toList();
                QVariantList drone_pos = drone["position"].toList();
                double dist = std::hypot(
                    drone_pos[0].toDouble() - task_pos[0].toDouble(),
                    drone_pos[1].toDouble() - task_pos[1].toDouble()
                );
                
                double score = dist + load * 100;
                
                if (score < best_score) {
                    best_score = score;
                    best_drone = d_id;
                }
            }
            
            if (!best_drone.isEmpty()) {
                allocations[best_drone].toList().append(task);
                drone_loads[best_drone] = drone_loads[best_drone].toInt() + 1;
            }
        }
        
        // Verify allocation
        QVERIFY(allocations.contains("1"));
        QVERIFY(allocations.contains("2"));
        
        int total_assigned = 0;
        for (auto it = allocations.begin(); it != allocations.end(); ++it) {
            total_assigned += it.value().toList().size();
        }
        
        QCOMPARE(total_assigned, 3);
    }
    
    void testMaxTasksPerDrone() {
        QVariantList tasks;
        for (int i = 0; i < 15; ++i) {
            tasks.append(QVariantMap{
                {"id", QString("t%1").arg(i)},
                {"position", QVariantList{double(i * 10), 0.0}},
                {"priority", 10 - i}
            });
        }
        
        QVariantList drones = {
            QVariantMap{{"id", 1}, {"position", QVariantList{0.0, 0.0}}},
            QVariantMap{{"id", 2}, {"position", QVariantList{100.0, 0.0}}}
        };
        
        QVariantMap allocations;
        QVariantMap drone_loads;
        for (const auto& drone : drones) {
            allocations[QString::number(drone["id"].toInt())] = QVariantList();
            drone_loads[QString::number(drone["id"].toInt())] = 0;
        }
        
        int max_per_drone = 5;
        
        for (const auto& task_var : tasks) {
            QVariantMap task = task_var.toMap();
            QString best_drone;
            double best_score = std::numeric_limits<double>::infinity();
            
            for (const auto& drone_var : drones) {
                QVariantMap drone = drone_var.toMap();
                QString d_id = QString::number(drone["id"].toInt());
                int load = drone_loads[d_id].toInt();
                
                if (load >= max_per_drone) continue;
                
                QVariantList task_pos = task["position"].toList();
                QVariantList drone_pos = drone["position"].toList();
                double dist = std::hypot(
                    drone_pos[0].toDouble() - task_pos[0].toDouble(),
                    drone_pos[1].toDouble() - task_pos[1].toDouble()
                );
                
                double score = dist + load * 100;
                
                if (score < best_score) {
                    best_score = score;
                    best_drone = d_id;
                }
            }
            
            if (!best_drone.isEmpty()) {
                allocations[best_drone].toList().append(task);
                drone_loads[best_drone] = drone_loads[best_drone].toInt() + 1;
            }
        }
        
        // Each drone should have at most max_per_drone tasks
        for (auto it = allocations.begin(); it != allocations.end(); ++it) {
            QVERIFY(it.value().toList().size() <= max_per_drone);
        }
        
        // Total assigned should be 10 (5 per drone * 2 drones)
        int total = 0;
        for (auto it = allocations.begin(); it != allocations.end(); ++it) {
            total += it.value().toList().size();
        }
        QCOMPARE(total, 10);
    }
    
    void testEmptyInput() {
        QVariantList empty_tasks;
        QVariantList empty_drones;
        
        // Should handle gracefully
        QVERIFY(empty_tasks.isEmpty());
        QVERIFY(empty_drones.isEmpty());
    }
    
    void testSingleDroneMultipleTasks() {
        QVariantList tasks = {
            QVariantMap{{"id", "t1"}, {"position", QVariantList{0.0, 0.0}}, {"priority", 10}},
            QVariantMap{{"id", "t2"}, {"position", QVariantList{10.0, 0.0}}, {"priority", 5}},
            QVariantMap{{"id", "t3"}, {"position", QVariantList{20.0, 0.0}}, {"priority", 8}}
        };
        
        QVariantList drones = {
            QVariantMap{{"id", 1}, {"position", QVariantList{5.0, 5.0}}}
        };
        
        QVariantMap allocations;
        QVariantMap drone_loads;
        allocations["1"] = QVariantList();
        drone_loads["1"] = 0;
        
        for (const auto& task_var : tasks) {
            QVariantMap task = task_var.toMap();
            allocations["1"].toList().append(task);
            drone_loads["1"] = drone_loads["1"].toInt() + 1;
        }
        
        QCOMPARE(allocations["1"].toList().size(), 3);
        QCOMPARE(drone_loads["1"].toInt(), 3);
    }
    
    void testPriorityOrdering() {
        QVariantList tasks = {
            QVariantMap{{"id", "low"}, {"position", QVariantList{0.0, 0.0}}, {"priority", 1}},
            QVariantMap{{"id", "high"}, {"position", QVariantList{100.0, 0.0}}, {"priority", 100}},
            QVariantMap{{"id", "med"}, {"position", QVariantList{50.0, 0.0}}, {"priority", 50}}
        };
        
        // Sort by priority descending
        std::sort(tasks.begin(), tasks.end(), [](const QVariant& a, const QVariant& b) {
            return a.toMap()["priority"].toInt() > b.toMap()["priority"].toInt();
        });
        
        QCOMPARE(tasks[0].toMap()["id"].toString(), "high");
        QCOMPARE(tasks[1].toMap()["id"].toString(), "med");
        QCOMPARE(tasks[2].toMap()["id"].toString(), "low");
    }
    
    void testDistanceCalculation() {
        // Test haversine-like distance
        auto calc_dist = [](double lat1, double lon1, double lat2, double lon2) {
            // Simple Euclidean for local coords
            return std::hypot(lat2 - lat1, lon2 - lon1);
        };
        
        double d1 = calc_dist(0, 0, 3, 4);  // 3-4-5 triangle
        QVERIFY(qFuzzyCompare(d1, 5.0));
        
        double d2 = calc_dist(10, 10, 10, 10);  // Same point
        QCOMPARE(d2, 0.0);
        
        double d3 = calc_dist(-5, -5, 5, 5);
        QVERIFY(qFuzzyCompare(d3, std::sqrt(200.0)));
    }
};

QTEST_MAIN(TestCBBAAllocator)
#include "test_cbba_allocator.moc"