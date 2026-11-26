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
    N24["#24 L1 switched model"]:::out
    N25["#25 L2 powered tests"]:::out
    OST["staging → Core<br/>promotion"]:::out
    OIRQ["TIM6/7<br/>irq_connect_dynamic"]:::out
    OS1["S1 old software check<br/>speed-up"]:::out
  end

  DEST(["🎯 Safety API merged into<br/>Core-staging, SPOC CI green"]):::dest

  subgraph Built["Built and L0-verified on safety-hw"]
    N16["✓ #16 CPU supervision"]:::done
    N17["✓ #17 Devicetree, allocation,<br/>manifest"]:::done
    N18["✓ #18 Comparator path"]:::done
    N19["✓ #19 ADC-watchdog path"]:::done
    N20["✓ #20 Safe states, outputs,<br/>fault record"]:::done
    N21["✓ #21 Class B self-tests"]:::done
    N22["✓ #22 One API,<br/>development mode"]:::done
  end

  subgraph Bench["Test bench and findings"]
    N15["✓ #15 Test infrastructure"]:::done
    N26["✓ #26 SPOC bench"]:::done
    N23["✓ #23 L0 runner on SPOC<br/>final campaign 11-23"]:::done
    N27["✓ #27 Timing overhead<br/>step 1"]:::done
    N28["✓ #28 Ownverter scaling<br/>(Core#171 upstream)"]:::done
  end

  subgraph Decide["Before the PR"]
    N30["✓ #30 Push safety-hw<br/>to GitHub"]:::done
    N32["✓ 🔎 #32 Zephyr 4.4.2<br/>still holds?"]:::done
    N33["✓ #33 Timing: declared,<br/>no fix"]:::done
    N31["✓ #31 One PR on<br/>Core-staging, tests in"]:::done
    N38["✓ #38 Fault ISR:<br/>IRQ_DIRECT_CONNECT"]:::done
    N39["✓ #39 L0 on<br/>Zephyr 4.4.2"]:::done
  end

  subgraph Comp["Companion PRs on Core-staging (land first)"]
    N40["🔨👤 #40 Ownverter scaling<br/>(Core#171)"]:::open
    N41["🔨👤 #41 HRTIM up-down<br/>re-init (Core#172)"]:::open
    N42["🔨👤 #42 TIM6/7<br/>reconnect fix"]:::open
  end

  subgraph PR["The Safety API PR on Core-staging (lands last)"]
    N34["🔨 #34 Cut the PR branch"]:::blocked
    N35["🔨👤 #35 L0 campaign<br/>on the PR branch"]:::blocked
    N36["🔨👤 #36 Open the PR"]:::blocked
    N37["👤 #37 Reviewed and merged"]:::blocked
    FR["review follow-ups"]:::fog
  end

  Out ~~~ Built
  Out ~~~ Bench
  N16 & N17 & N18 & N19 & N20 & N21 & N22 --> N31
  N23 & N28 --> N35
  N27 --> N33
  N32 --> N31
  N32 --> N38 --> N34
  N38 --> N39 --> N31
  N28 --> N40
  N31 --> N40 & N41 & N42
  N40 & N41 & N42 --> N34
  N31 --> N34
  N33 --> N34
  N34 --> N35 --> N36
  N30 --> N36
  N36 --> N37 --> DEST
  N37 -.-> FR -.-> DEST
```

<sub>Green done · blue takeable · purple in progress · orange blocked · red ⏳ stale (3 days without activity) · grey dashed fog · white dashed out of scope · 🔨 build · 🔎 research · 👤 can't move without Luiz.</sub>
