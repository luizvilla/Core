# Protection table: TWIST v1.2.0

Generated from the shield devicetree by the build (ADR 0009). Do not edit by hand.

| Limit | Low | High | Low bound path | High bound path | Note |
|---|---|---|---|---|---|
| V1_LOW | -10 | 70 | software (exception) | software (exception) | legacy board: every HRTIM fault input is taken by higher-priority limits (ADR 0009); software path only |
| V2_LOW | -10 | 70 | software (exception) | software (exception) | legacy board: every HRTIM fault input is taken by higher-priority limits (ADR 0009); software path only |
| V_HIGH | 10 | 100 | ADC2 AWD3->FLT6 | ADC2 AWD3->FLT6 |  |
| I1_LOW | -7 | 7 | ADC1 AWD1->FLT1 | COMP1->FLT4 |  |
| I2_LOW | -7 | 7 | ADC1 AWD2->FLT2 | COMP3->FLT5 |  |
| I_HIGH | -7 | 7 | ADC1 AWD3->FLT3 | ADC1 AWD3->FLT3 |  |

| Leg | Safe state |
|---|---|
| LEG1 | open |
| LEG2 | open |

Fault inputs used: FLT1 (I1_LOW low (ADC1 AWD1)), FLT2 (I2_LOW low (ADC1 AWD2)), FLT3 (I_HIGH high/low (ADC1 AWD3)), FLT4 (I1_LOW high (COMP1)), FLT5 (I2_LOW high (COMP3)), FLT6 (V_HIGH high/low (ADC2 AWD3)). Free: none.
