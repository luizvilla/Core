# Protection table: TWIST v0.9.0

Generated from the shield devicetree by the build (ADR 0009). Do not edit by hand.

| Limit | Low | High | Low bound path | High bound path | Note |
|---|---|---|---|---|---|
| V1_LOW | -10 | 70 | software | software (exception) | legacy board: every HRTIM fault input is taken by higher-priority limits (ADR 0009); software path only |
| V2_LOW | -10 | 70 | software | software (exception) | legacy board: every HRTIM fault input is taken by higher-priority limits (ADR 0009); software path only |
| V3_LOW | -10 | 70 | software | software (exception) | legacy board: every HRTIM fault input is taken by higher-priority limits (ADR 0009); software path only |
| V_HIGH | 10 | 100 | software | ADC1 AWD2->FLT2 |  |
| I1_LOW | -7 | 7 | ADC2 AWD3->FLT6 | COMP3->FLT5 |  |
| I2_LOW | -7 | 7 | software (exception) | COMP1->FLT4 | legacy board: no free watchdog on its default ADC (ADC2) for the low bound (ADR 0009); software path only |
| I3_LOW | -7 | 7 | ADC1 AWD1->FLT1 | COMP6->FLT3 |  |

| Leg | Safe state |
|---|---|
| LEG1 | open |
| LEG2 | open |
| LEG3 | open |

Fault inputs used: FLT1 (I3_LOW low (ADC1 AWD1)), FLT2 (V_HIGH high (ADC1 AWD2)), FLT3 (I3_LOW high (COMP6)), FLT4 (I2_LOW high (COMP1)), FLT5 (I1_LOW high (COMP3)), FLT6 (I1_LOW low (ADC2 AWD3)). Free: none.
