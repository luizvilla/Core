# Safety API dev map

Copy of the visual map on [Map: Safety API as an upstream PR](https://github.com/luizvilla/Core/issues/29), the canonical one. Redrawn whenever a ticket opens, closes, is claimed or goes stale.

**Destination:** the Safety API of ADR 0009 and its safety tests merged into `owntech-foundation/Core-staging` as one pull request, passing staging's SPOC CI and L0-verified on Twist and Ownverter (redrawn on 2026-11-26 by [What goes into the Safety API PR](https://github.com/luizvilla/Core/issues/31)).

```mermaid
flowchart LR
  classDef done fill:#d3f9d8,stroke:#2b8a3e,color:#1b4332
  classDef open fill:#d0ebff,stroke:#1c7ed6,color:#0b3d6e
  classDef blocked fill:#ffe8cc,stroke:#e8590c,color:#5c2b00
  classDef wip fill:#f3d9fa,stroke:#ae3ec9,color:#4a1259
  classDef stale fill:#ffe3e3,stroke:#e03131,color:#5c0a0a,stroke-width:3px
  classDef fog fill:#f1f3f5,stroke:#868e96,color:#495057,stroke-dasharray:5 5
  classDef out fill:#fff,stroke:#adb5bd,color:#868e96,stroke-dasharray:2 2
  classDef dest fill:#343a40,stroke:#000,color:#fff,font-weight:bold

  subgraph Out["Out of scope"]
    OUT["#24 L1 switched model<br/>#25 L2 powered tests<br/>timing step 2 (uSolarVerter)<br/>S1 software check speed-up<br/>TIM6/7 irq_connect_dynamic<br/>staging → Core promotion"]:::out
  end

  subgraph Done["Done"]
    direction TB
    BUILT["✓ Built on safety-hw, #16–#22<br/>CPU supervision · allocation · comparator<br/>ADC watchdog · safe states · Class B · one API"]:::done
    BENCH["✓ Bench and findings<br/>#15 infra · #26 SPOC · #23 L0 runner<br/>#27 timing · #28 Ownverter scaling"]:::done
    DEC["✓ Decided<br/>#30 push · #32 #38 #39 Zephyr 4.4.2<br/>#33 timing declared · #31 one PR on staging"]:::done
  end

  subgraph Comp["Companion PRs on Core-staging"]
    direction TB
    N40["🔨👤 #40 Ownverter<br/>scaling (Core#171)"]:::open
    N41["🔨👤 #41 HRTIM up-down<br/>re-init (Core#172)"]:::open
    N42["🔨👤 #42 TIM6/7<br/>reconnect fix"]:::open
  end

  subgraph PR["Safety API PR on Core-staging, lands last"]
    direction LR
    N34["🔨 #34 Cut<br/>the branch"]:::blocked
    N35["🔨👤 #35 L0<br/>campaign"]:::blocked
    N36["🔨👤 #36 Open<br/>the PR"]:::blocked
    FR["review<br/>follow-ups"]:::fog
    N37["👤 #37 Reviewed<br/>and merged"]:::blocked
    N34 --> N35 --> N36 --> N37
    N36 -.-> FR -.-> N37
  end

  DEST(["🎯 Merged into Core-staging,<br/>SPOC CI green"]):::dest

  Out ~~~ Done
  Done --> Comp --> PR
  N37 --> DEST
```

<sub>Green done · blue takeable · purple in progress · orange blocked · red ⏳ stale (3 days without activity) · grey dashed fog · white dashed out of scope · 🔨 build · 🔎 research · 👤 can't move without Luiz.</sub>
