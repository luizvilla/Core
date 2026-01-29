

#include <stdint.h>
#include <string.h>

#include <thingset.h>
#include "transform.h"

/*
 * Groups / first layer data object IDs
 */
#define ID_ROOT        0x00

/* Measurements */
#define ID_MEAS         0x5
#define ID_MEAS_VLOW    0x50
#define ID_MEAS_VAC     0x51
#define ID_MEAS_ILOW1   0x52
#define ID_MEAS_ILOW2   0x53
#define ID_MEAS_VDC     0x54
#define ID_MEAS_IAC     0x55
#define ID_MEAS_VDC_F   0x56
#define ID_MEAS_VGRID   0x57
#define ID_MEAS_VN      0x58
#define ID_MEAS_IGRID   0x59

/* Power control */
#define ID_CTRL        0x40
#define ID_CTRL_PWR    0x401
#define ID_CTRL_BOOST  0x4011
#define ID_CTRL_INV    0x4012
#define ID_CTRL_REF    0x402
#define ID_CTRL_BVREF  0x4021
#define ID_CTRL_VDREF  0x4022
#define ID_CTRL_VQREF  0x4023
#define ID_CTRL_IDREF  0x4024
#define ID_CTRL_IQREF  0x4025

/*
 * Subset definitions for statements and publish/subscribe
 */

/* UART serial */
#define SUBSET_SER  (1U << 0)
/* CAN bus */
#define SUBSET_CAN  (1U << 1)
/* Control data sent and received via CAN */
#define SUBSET_CTRL (1U << 3)

/* Measure variables (defined in main.cpp) */
extern float32_t Vlow_value;
extern float32_t Vac_value;
extern float32_t Ilow1_value;
extern float32_t Ilow2_value;
extern float32_t Vdc_bus;
extern float32_t Iac_value;
extern float32_t Vdc_bus_filt;
extern float32_t Vgrid_meas;
extern float32_t VN_meas;
extern float32_t Igrid_meas;

/* Power controls (defined in main.cpp) */
extern bool boost_pwm_enable;
extern bool inverter_on;
extern float32_t boost_voltage_reference;
extern dqo_t Vdq_ref;
extern dqo_t Idq_ref;

/* ThingSet object definitions */
TS_ADD_GROUP(ID_MEAS, "Measurements", TS_NO_CALLBACK, ID_ROOT);

TS_ADD_ITEM_FLOAT(ID_MEAS_VLOW, "rVlow_V", &Vlow_value, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_VAC, "rVac_V", &Vac_value, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_ILOW1, "rIlow1_A", &Ilow1_value, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_ILOW2, "rIlow2_A", &Ilow2_value, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_VDC, "rVdc_V", &Vdc_bus, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_IAC, "rIac_A", &Iac_value, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_VDC_F, "rVdcFilt_V", &Vdc_bus_filt, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_VGRID, "rVgrid_V", &Vgrid_meas, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_VN, "rVN_V", &VN_meas, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_MEAS_IGRID, "rIgrid_A", &Igrid_meas, 2,
                  ID_MEAS, TS_ANY_R, SUBSET_SER);

/* Power control group */
TS_ADD_GROUP(ID_CTRL, "Control", TS_NO_CALLBACK, ID_ROOT);
TS_ADD_GROUP(ID_CTRL_PWR, "Power", TS_NO_CALLBACK, ID_CTRL);
TS_ADD_ITEM_BOOL(ID_CTRL_BOOST, "wBoostEnable", &boost_pwm_enable,
                 ID_CTRL_PWR, TS_ANY_RW, SUBSET_SER);
TS_ADD_ITEM_BOOL(ID_CTRL_INV, "wInverterOn", &inverter_on,
                 ID_CTRL_PWR, TS_ANY_RW, SUBSET_SER);

/* Reference setpoints */
TS_ADD_GROUP(ID_CTRL_REF, "Refs", TS_NO_CALLBACK, ID_CTRL);
TS_ADD_ITEM_FLOAT(ID_CTRL_BVREF, "wBoostVRef_V", &boost_voltage_reference, 2,
                  ID_CTRL_REF, TS_ANY_RW, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_CTRL_VDREF, "wVdRef_V", &Vdq_ref.d, 2,
                  ID_CTRL_REF, TS_ANY_RW, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_CTRL_VQREF, "wVqRef_V", &Vdq_ref.q, 2,
                  ID_CTRL_REF, TS_ANY_RW, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_CTRL_IDREF, "wIdRef_A", &Idq_ref.d, 2,
                  ID_CTRL_REF, TS_ANY_RW, SUBSET_SER);
TS_ADD_ITEM_FLOAT(ID_CTRL_IQREF, "wIqRef_A", &Idq_ref.q, 2,
                  ID_CTRL_REF, TS_ANY_RW, SUBSET_SER);
