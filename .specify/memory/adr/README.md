# Architecture Decision Records (ADR)

This directory contains Architecture Decision Records for the AI Swarm Autonomous Drone System.

## Index

| ADR | Title | Status | Date | Related Research |
|-----|-------|--------|------|------------------|
| [ADR-001](adr-001.md) | PX4 + ROS 2 Hybrid Architecture | Accepted | 2025-11-20 | Research Decision 1 |
| [ADR-002](adr-002.md) | Tiered Mesh Communication (Wi-Fi 6E/5G/LoRa) | Accepted | 2025-11-20 | Research Decision 2 |
| [ADR-003](adr-003.md) | TensorRT + ONNX Runtime Dual Inference Engine | Accepted | 2025-11-20 | Research Decision 3 |
| [ADR-004](adr-004.md) | CBBA Consensus for Task Allocation | Accepted | 2025-11-20 | Research Decision 4 |
| [ADR-005](adr-005.md) | GTSAM-based VIO + UWB Fusion | Accepted | 2025-11-20 | Research Decision 5 |
| [ADR-006](adr-006.md) | ORCA/MPC Hybrid Collision Avoidance | Accepted | 2025-11-20 | Research Decision 6 |
| [ADR-007](adr-007.md) | ULog + rosbag2 Dual Logging with Time Sync | Accepted | 2025-11-20 | Research Decision 7 |
| [ADR-008](adr-008.md) | QGC Plugin + Web Dashboard Hybrid GCS | Accepted | 2025-11-20 | Research Decision 8 |

---

## ADR Template

```markdown
# ADR-XXX: [Title]

## Status
[Proposed | Accepted | Rejected | Deprecated | Superseded]

## Context
[What is the issue that we're seeing that is motivating this decision or change?]

## Decision
[What is the change that we're proposing and/or doing?]

## Consequences
### Positive
- [Benefit 1]
- [Benefit 2]

### Negative
- [Drawback 1]
- [Drawback 2]

### Neutral
- [Neutral observation]

## Related Research
[Link to research.md decision]

## References
- [Reference 1]
- [Reference 2]

## Date
YYYY-MM-DD

## Authors
- [Author Name]
```

---

## ADR Lifecycle

1. **Proposed** - Initial draft under review
2. **Accepted** - Approved and implemented
3. **Rejected** - Not adopted
4. **Deprecated** - No longer recommended
5. **Superseded** - Replaced by newer ADR

---

## Governance

- ADRs are immutable once Accepted
- New ADRs can Supersede old ones
- All architectural decisions must have an ADR
- ADRs are reviewed during Architecture Review meetings