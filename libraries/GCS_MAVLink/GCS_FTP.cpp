/*
   GCS MAVLink functions related to FTP

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "GCS_config.h"

#if AP_MAVLINK_FTP_ENABLED
#include <unordered_set>
#include <string>

#include <AP_HAL/AP_HAL.h>

#include "GCS.h"

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

struct GCS_MAVLINK::ftp_state GCS_MAVLINK::ftp;

// timeout for session inactivity
#define FTP_SESSION_TIMEOUT 3000

bool GCS_MAVLINK::ftp_init(void) {

    // check if ftp is disabled for memory savings
#if !defined(HAL_BUILD_AP_PERIPH)
    if (AP_BoardConfig::ftp_disabled()) {
        goto failed;
    }
#endif
    // we can simply check if we allocated everything we need

    if (ftp.requests != nullptr) {
        return true;
    }

    ftp.requests = NEW_NOTHROW ObjectBuffer<pending_ftp>(5);
    if (ftp.requests == nullptr || ftp.requests->get_size() == 0) {
        goto failed;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&GCS_MAVLINK::ftp_worker, void),
                                      "FTP", 2560, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        goto failed;
    }

    return true;

failed:
    delete ftp.requests;
    ftp.requests = nullptr;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed to initialize MAVFTP");

    return false;
}


static const std::unordered_set<std::string> params_to_zero = {
    "Param",
    "ACRO_BAL_PITCH",
    "ACRO_BAL_ROLL",
    "ACRO_OPTIONS",
    "ACRO_RP_EXPO",
    "ACRO_RP_RATE",
    "ACRO_RP_RATE_TC",
    "ACRO_THR_MID",
    "ACRO_TRAINER",
    "ACRO_Y_EXPO",
    "ACRO_Y_RATE",
    "ACRO_Y_RATE_TC",
    "AHRS_COMP_BETA",
    "AHRS_EKF_TYPE",
    "AHRS_GPS_GAIN",
    "AHRS_GPS_MINSATS",
    "AHRS_GPS_USE",
    "AHRS_OPTIONS",
    "AHRS_ORIENTATION",
    "AHRS_RP_P",
    "AHRS_TRIM_X",
    "AHRS_TRIM_Y",
    "AHRS_TRIM_Z",
    "AHRS_WIND_MAX",
    "AHRS_YAW_P",
    "ANGLE_MAX",
    "ARMING_ACCTHRESH",
    "ARMING_CHECK",
    "ARMING_MAGTHRESH",
    "ARMING_MIS_ITEMS",
    "ARMING_OPTIONS",
    "ARMING_RUDDER",
    "ATC_ACCEL_P_MAX",
    "ATC_ACCEL_R_MAX",
    "ATC_ACCEL_Y_MAX",
    "ATC_ANG_LIM_TC",
    "ATC_ANG_PIT_P",
    "ATC_ANG_RLL_P",
    "ATC_ANG_YAW_P",
    "ATC_ANGLE_BOOST",
    "ATC_INPUT_TC",
    "ATC_RAT_PIT_D",
    "ATC_RAT_PIT_D_FF",
    "ATC_RAT_PIT_FF",
    "ATC_RAT_PIT_FLTD",
    "ATC_RAT_PIT_FLTE",
    "ATC_RAT_PIT_FLTT",
    "ATC_RAT_PIT_I",
    "ATC_RAT_PIT_IMAX",
    "ATC_RAT_PIT_P",
    "ATC_RAT_PIT_PDMX",
    "ATC_RAT_PIT_SMAX",
    "ATC_RAT_RLL_D",
    "ATC_RAT_RLL_D_FF",
    "ATC_RAT_RLL_FF",
    "ATC_RAT_RLL_FLTD",
    "ATC_RAT_RLL_FLTE",
    "ATC_RAT_RLL_FLTT",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_IMAX",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_PDMX",
    "ATC_RAT_RLL_SMAX",
    "ATC_RAT_YAW_D",
    "ATC_RAT_YAW_D_FF",
    "ATC_RAT_YAW_FF",
    "ATC_RAT_YAW_FLTD",
    "ATC_RAT_YAW_FLTE",
    "ATC_RAT_YAW_FLTT",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_PDMX",
    "ATC_RAT_YAW_SMAX",
    "ATC_RATE_FF_ENAB",
    "ATC_RATE_P_MAX",
    "ATC_RATE_R_MAX",
    "ATC_RATE_Y_MAX",
    "ATC_SLEW_YAW",
    "ATC_THR_G_BOOST",
    "ATC_THR_MIX_MAN",
    "ATC_THR_MIX_MAX",
    "ATC_THR_MIX_MIN",
    "AUTO_OPTIONS",
    "AUTOTUNE_AGGR",
    "AUTOTUNE_AXES",
    "AUTOTUNE_MIN_D",
    "AVOID_ACCEL_MAX",
    "AVOID_ALT_MIN",
    "AVOID_ANGLE_MAX",
    "AVOID_BACKUP_DZ",
    "AVOID_BACKUP_SPD",
    "AVOID_BACKZ_SPD",
    "AVOID_BEHAVE",
    "AVOID_DIST_MAX",
    "AVOID_ENABLE",
    "AVOID_MARGIN",
    "BARO_ALT_OFFSET",
    "BARO_ALTERR_MAX",
    "BARO_EXT_BUS",
    "BARO_FLTR_RNG",
    "BARO_GND_TEMP",
    "BARO_OPTIONS",
    "BARO_PRIMARY",
    "BARO_PROBE_EXT",
    "BARO2_DEVID",
    "BARO2_GND_PRESS",
    "BARO3_DEVID",
    "BARO3_GND_PRESS",
    "BATT_ARM_MAH",
    "BATT_ARM_VOLT",
    "BATT_CAPACITY",
    "BATT_CRT_MAH",
    "BATT_CRT_VOLT",
    "BATT_CURR_PIN",
    "BATT_FS_CRT_ACT",
    "BATT_FS_LOW_ACT",
    "BATT_FS_VOLTSRC",
    "BATT_LOW_MAH",
    "BATT_LOW_TIMER",
    "BATT_LOW_VOLT",
    "BATT_MONITOR",
    "BATT_OPTIONS",
    "BATT_SERIAL_NUM",
    "BATT_VLT_OFFSET",
    "BATT_VOLT_MULT",
    "BATT_VOLT_PIN",
    "BATT2_MONITOR",
    "BATT3_MONITOR",
    "BATT4_MONITOR",
    "BATT5_MONITOR",
    "BATT6_MONITOR",
    "BATT7_MONITOR",
    "BATT8_MONITOR",
    "BATT9_MONITOR",
    "BRD_ALT_CONFIG",
    "BRD_BOOT_DELAY",
    "BRD_OPTIONS",
    "BRD_RTC_TYPES",
    "BRD_RTC_TZ_MIN",
    "BRD_SAFETY_DEFLT",
    "BRD_SAFETY_MASK",
    "BRD_SAFETYOPTION",
    "BRD_SD_SLOWDOWN",
    "CAM_AUTO_ONLY",
    "CAM_MAX_ROLL",
    "CAM1_TYPE",
    "CAM2_TYPE",
    "CIRCLE_OPTIONS",
    "CIRCLE_RADIUS",
    "CIRCLE_RATE",
    "COMPASS_AUTO_ROT",
    "COMPASS_AUTODEC",
    "COMPASS_CAL_FIT",
    "COMPASS_DEV_ID3",
    "COMPASS_DEV_ID4",
    "COMPASS_DEV_ID5",
    "COMPASS_DEV_ID6",
    "COMPASS_DEV_ID7",
    "COMPASS_DEV_ID8",
    "COMPASS_DISBLMSK",
    "COMPASS_ENABLE",
    "COMPASS_EXTERN2",
    "COMPASS_EXTERN3",
    "COMPASS_EXTERNAL",
    "COMPASS_FLTR_RNG",
    "COMPASS_LEARN",
    "COMPASS_OPTIONS",
    "COMPASS_ORIENT",
    "COMPASS_ORIENT2",
    "COMPASS_ORIENT3",
    "COMPASS_PMOT_EN",
    "COMPASS_PRIO3_ID",
    "COMPASS_SCALE",
    "COMPASS_SCALE2",
    "COMPASS_SCALE3",
    "COMPASS_USE",
    "COMPASS_USE2",
    "COMPASS_USE3",
    "CUST_ROT_ENABLE",
    "DEV_OPTIONS",
    "DISARM_DELAY",
    "EK3_ABIAS_P_NSE",
    "EK3_ACC_BIAS_LIM",
    "EK3_ACC_P_NSE",
    "EK3_AFFINITY",
    "EK3_ALT_M_NSE",
    "EK3_BCN_DELAY",
    "EK3_BCN_I_GTE",
    "EK3_BCN_M_NSE",
    "EK3_BETA_MASK",
    "EK3_CHECK_SCALE",
    "EK3_DRAG_BCOEF_X",
    "EK3_DRAG_BCOEF_Y",
    "EK3_DRAG_M_NSE",
    "EK3_DRAG_MCOEF",
    "EK3_EAS_I_GATE",
    "EK3_EAS_M_NSE",
    "EK3_ENABLE",
    "EK3_ERR_THRESH",
    "EK3_FLOW_DELAY",
    "EK3_FLOW_I_GATE",
    "EK3_FLOW_M_NSE",
    "EK3_FLOW_USE",
    "EK3_GBIAS_P_NSE",
    "EK3_GLITCH_RAD",
    "EK3_GND_EFF_DZ",
    "EK3_GPS_CHECK",
    "EK3_GPS_VACC_MAX",
    "EK3_GSF_RST_MAX",
    "EK3_GSF_RUN_MASK",
    "EK3_GSF_USE_MASK",
    "EK3_GYRO_P_NSE",
    "EK3_HGT_DELAY",
    "EK3_HGT_I_GATE",
    "EK3_HRT_FILT",
    "EK3_IMU_MASK",
    "EK3_LOG_LEVEL",
    "EK3_MAG_CAL",
    "EK3_MAG_EF_LIM",
    "EK3_MAG_I_GATE",
    "EK3_MAG_M_NSE",
    "EK3_MAG_MASK",
    "EK3_MAGB_P_NSE",
    "EK3_MAGE_P_NSE",
    "EK3_MAX_FLOW",
    "EK3_NOAID_M_NSE",
    "EK3_OGN_HGT_MASK",
    "EK3_OGNM_TEST_SF",
    "EK3_POS_I_GATE",
    "EK3_POSNE_M_NSE",
    "EK3_PRIMARY",
    "EK3_RNG_I_GATE",
    "EK3_RNG_M_NSE",
    "EK3_RNG_USE_HGT",
    "EK3_RNG_USE_SPD",
    "EK3_SRC_OPTIONS",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "EK3_SRC2_POSXY",
    "EK3_SRC2_POSZ",
    "EK3_SRC2_VELXY",
    "EK3_SRC2_VELZ",
    "EK3_SRC2_YAW",
    "EK3_SRC3_POSXY",
    "EK3_SRC3_POSZ",
    "EK3_SRC3_VELXY",
    "EK3_SRC3_VELZ",
    "EK3_SRC3_YAW",
    "EK3_TAU_OUTPUT",
    "EK3_TERR_GRAD",
    "EK3_VEL_I_GATE",
    "EK3_VELD_M_NSE",
    "EK3_VELNE_M_NSE",
    "EK3_VIS_VERR_MAX",
    "EK3_VIS_VERR_MIN",
    "EK3_WENC_VERR",
    "EK3_WIND_P_NSE",
    "EK3_WIND_PSCALE",
    "EK3_YAW_I_GATE",
    "EK3_YAW_M_NSE",
    "ESC_CALIBRATION",
    "ESC_TLM_MAV_OFS",
    "FENCE_ACTION",
    "FENCE_ALT_MAX",
    "FENCE_ALT_MIN",
    "FENCE_ENABLE",
    "FENCE_MARGIN",
    "FENCE_RADIUS",
    "FENCE_TOTAL",
    "FENCE_TYPE",
    "FLIGHT_OPTIONS",
    "FLTMODE_CH",
    "FLTMODE_GCSBLOCK",
    "FLTMODE1",
    "FLTMODE2",
    "FLTMODE3",
    "FLTMODE4",
    "FLTMODE5",
    "FLTMODE6",
    "FORMAT_VERSION",
    "FRAME_CLASS",
    "FRAME_TYPE",
    "FRSKY_DNLINK_ID",
    "FRSKY_DNLINK1_ID",
    "FRSKY_DNLINK2_ID",
    "FRSKY_OPTIONS",
    "FRSKY_UPLINK_ID",
    "FS_CRASH_CHECK",
    "FS_DR_ENABLE",
    "FS_DR_TIMEOUT",
    "FS_EKF_ACTION",
    "FS_EKF_FILT",
    "FS_EKF_THRESH",
    "FS_GCS_ENABLE",
    "FS_GCS_TIMEOUT",
    "FS_OPTIONS",
    "FS_THR_ENABLE",
    "FS_THR_VALUE",
    "FS_VIBE_ENABLE",
    "GCS_PID_MASK",
    "GND_EFFECT_COMP",
    "GPS_AUTO_CONFIG",
    "GPS_AUTO_SWITCH",
    "GPS_BLEND_MASK",
    "GPS_DELAY_MS",
    "GPS_DELAY_MS2",
    "GPS_DRV_OPTIONS",
    "GPS_GNSS_MODE",
    "GPS_GNSS_MODE2",
    "GPS_HDOP_GOOD",
    "GPS_INJECT_TO",
    "GPS_MIN_DGPS",
    "GPS_MIN_ELEV",
    "GPS_NAVFILTER",
    "GPS_POS1_X",
    "GPS_POS1_Y",
    "GPS_POS1_Z",
    "GPS_POS2_X",
    "GPS_POS2_Y",
    "GPS_POS2_Z",
    "GPS_PRIMARY",
    "GPS_RATE_MS",
    "GPS_RATE_MS2",
    "GPS_RAW_DATA",
    "GPS_SAVE_CFG",
    "GPS_SBAS_MODE",
    "GPS_TYPE",
    "GPS_TYPE2",
    "GUID_OPTIONS",
    "GUID_TIMEOUT",
    "INITIAL_MODE",
    "INS_ACC_BODYFIX",
    "INS_ACC_ID",
    "INS_ACCEL_FILTER",
    "INS_ENABLE_MASK",
    "INS_FAST_SAMPLE",
    "INS_GYR_ID",
    "INS_GYRO_FILTER",
    "INS_GYRO_RATE",
    "INS_HNTC2_ENABLE",
    "INS_HNTCH_ATT",
    "INS_HNTCH_BW",
    "INS_HNTCH_ENABLE",
    "INS_HNTCH_FM_RAT",
    "INS_HNTCH_FREQ",
    "INS_HNTCH_HMNCS",
    "INS_HNTCH_MODE",
    "INS_HNTCH_OPTS",
    "INS_HNTCH_REF",
    "INS_LOG_BAT_CNT",
    "INS_LOG_BAT_LGCT",
    "INS_LOG_BAT_LGIN",
    "INS_LOG_BAT_MASK",
    "INS_LOG_BAT_OPT",
    "INS_POS1_X",
    "INS_POS1_Y",
    "INS_POS1_Z",
    "INS_RAW_LOG_OPT",
    "INS_STILL_THRESH",
    "INS_TRIM_OPTION",
    "INS_USE",
    "LAND_ALT_LOW",
    "LAND_REPOSITION",
    "LAND_SPEED",
    "LAND_SPEED_HIGH",
    "LGR_ENABLE",
    "LOG_BACKEND_TYPE",
    "LOG_BITMASK",
    "LOG_BLK_RATEMAX",
    "LOG_DARM_RATEMAX",
    "LOG_DISARMED",
    "LOG_FILE_BUFSIZE",
    "LOG_FILE_DSRMROT",
    "LOG_FILE_MB_FREE",
    "LOG_FILE_RATEMAX",
    "LOG_FILE_TIMEOUT",
    "LOG_MAV_BUFSIZE",
    "LOG_MAV_RATEMAX",
    "LOG_MAX_FILES",
    "LOG_REPLAY",
    "LOIT_ACC_MAX",
    "LOIT_ANG_MAX",
    "LOIT_BRK_ACCEL",
    "LOIT_BRK_DELAY",
    "LOIT_BRK_JERK",
    "LOIT_SPEED",
    "MIS_OPTIONS",
    "MIS_RESTART",
    "MIS_TOTAL",
    "MOT_BAT_CURR_MAX",
    "MOT_BAT_CURR_TC",
    "MOT_BAT_IDX",
    "MOT_BAT_VOLT_MAX",
    "MOT_BAT_VOLT_MIN",
    "MOT_BOOST_SCALE",
    "MOT_HOVER_LEARN",
    "MOT_OPTIONS",
    "MOT_PWM_MAX",
    "MOT_PWM_MIN",
    "MOT_PWM_TYPE",
    "MOT_SAFE_DISARM",
    "MOT_SAFE_TIME",
    "MOT_SLEW_DN_TIME",
    "MOT_SLEW_UP_TIME",
    "MOT_SPIN_ARM",
    "MOT_SPIN_MAX",
    "MOT_SPIN_MIN",
    "MOT_SPOOL_TIM_DN",
    "MOT_SPOOL_TIME",
    "MOT_THST_EXPO",
    "MOT_THST_HOVER",
    "MOT_YAW_HEADROOM",
    "MSP_OPTIONS",
    "MSP_OSD_NCELLS",
    "NTF_BUZZ_ON_LVL",
    "NTF_BUZZ_PIN",
    "NTF_BUZZ_TYPES",
    "NTF_BUZZ_VOLUME",
    "NTF_LED_BRIGHT",
    "NTF_LED_LEN",
    "NTF_LED_OVERRIDE",
    "NTF_LED_TYPES",
    "OSD_TYPE",
    "PHLD_BRAKE_ANGLE",
    "PHLD_BRAKE_RATE",
    "PILOT_ACCEL_Z",
    "PILOT_SPEED_DN",
    "PILOT_SPEED_UP",
    "PILOT_THR_BHV",
    "PILOT_THR_FILT",
    "PILOT_TKOFF_ALT",
    "PILOT_Y_EXPO",
    "PILOT_Y_RATE",
    "PILOT_Y_RATE_TC",
    "PLDP_DELAY",
    "PLDP_RNG_MIN",
    "PLDP_SPEED_DN",
    "PLDP_THRESH",
    "PSC_ACCZ_D",
    "PSC_ACCZ_D_FF",
    "PSC_ACCZ_FF",
    "PSC_ACCZ_FLTD",
    "PSC_ACCZ_FLTE",
    "PSC_ACCZ_FLTT",
    "PSC_ACCZ_I",
    "PSC_ACCZ_IMAX",
    "PSC_ACCZ_P",
    "PSC_ACCZ_PDMX",
    "PSC_ACCZ_SMAX",
    "PSC_ANGLE_MAX",
    "PSC_JERK_XY",
    "PSC_JERK_Z",
    "PSC_POSXY_P",
    "PSC_POSZ_P",
    "PSC_VELXY_D",
    "PSC_VELXY_FF",
    "PSC_VELXY_FLTD",
    "PSC_VELXY_FLTE",
    "PSC_VELXY_I",
    "PSC_VELXY_IMAX",
    "PSC_VELXY_P",
    "PSC_VELZ_D",
    "PSC_VELZ_FF",
    "PSC_VELZ_FLTD",
    "PSC_VELZ_FLTE",
    "PSC_VELZ_I",
    "PSC_VELZ_IMAX",
    "PSC_VELZ_P",
    "RALLY_INCL_HOME",
    "RALLY_LIMIT_KM",
    "RALLY_TOTAL",
    "RC_FS_TIMEOUT",
    "RC_OPTIONS",
    "RC_OVERRIDE_TIME",
    "RC_PROTOCOLS",
    "RC_SPEED",
    "RC1_DZ",
    "RC1_OPTION",
    "RC1_REVERSED",
    "RC10_DZ",
    "RC10_OPTION",
    "RC10_REVERSED",
    "RC11_DZ",
    "RC11_OPTION",
    "RC11_REVERSED",
    "RC12_DZ",
    "RC12_OPTION",
    "RC12_REVERSED",
    "RC13_DZ",
    "RC13_OPTION",
    "RC13_REVERSED",
    "RC14_DZ",
    "RC14_OPTION",
    "RC14_REVERSED",
    "RC15_DZ",
    "RC15_OPTION",
    "RC15_REVERSED",
    "RC16_DZ",
    "RC16_OPTION",
    "RC16_REVERSED",
    "RC2_DZ",
    "RC2_OPTION",
    "RC2_REVERSED",
    "RC3_DZ",
    "RC3_OPTION",
    "RC3_REVERSED",
    "RC4_DZ",
    "RC4_OPTION",
    "RC4_REVERSED",
    "RC5_DZ",
    "RC5_OPTION",
    "RC5_REVERSED",
    "RC6_DZ",
    "RC6_OPTION",
    "RC6_REVERSED",
    "RC7_DZ",
    "RC7_OPTION",
    "RC7_REVERSED",
    "RC8_DZ",
    "RC8_OPTION",
    "RC8_REVERSED",
    "RC9_DZ",
    "RC9_OPTION",
    "RC9_REVERSED",
    "RCMAP_PITCH",
    "RCMAP_ROLL",
    "RCMAP_THROTTLE",
    "RCMAP_YAW",
    "RELAY1_FUNCTION",
    "RELAY2_FUNCTION",
    "RELAY3_FUNCTION",
    "RELAY4_FUNCTION",
    "RELAY5_FUNCTION",
    "RELAY6_FUNCTION",
    "RNGFND_FILT",
    "RNGFND1_TYPE",
    "RNGFND2_TYPE",
    "RNGFND3_TYPE",
    "RNGFND4_TYPE",
    "RNGFND5_TYPE",
    "RNGFND6_TYPE",
    "RNGFND7_TYPE",
    "RNGFND8_TYPE",
    "RNGFND9_TYPE",
    "RNGFNDA_TYPE",
    "RPM1_TYPE",
    "RPM2_TYPE",
    "RSSI_ANA_PIN",
    "RSSI_CHAN_HIGH",
    "RSSI_CHAN_LOW",
    "RSSI_CHANNEL",
    "RSSI_PIN_HIGH",
    "RSSI_PIN_LOW",
    "RSSI_TYPE",
    "RTL_ALT",
    "RTL_ALT_FINAL",
    "RTL_ALT_TYPE",
    "RTL_CLIMB_MIN",
    "RTL_CONE_SLOPE",
    "RTL_LOIT_TIME",
    "RTL_OPTIONS",
    "RTL_SPEED",
    "SCHED_DEBUG",
    "SCHED_LOOP_RATE",
    "SCHED_OPTIONS",
    "SERIAL_PASS1",
    "SERIAL_PASS2",
    "SERIAL_PASSTIMO",
    "SERIAL0_BAUD",
    "SERIAL0_PROTOCOL",
    "SERIAL1_BAUD",
    "SERIAL1_OPTIONS",
    "SERIAL1_PROTOCOL",
    "SERIAL2_BAUD",
    "SERIAL2_OPTIONS",
    "SERIAL2_PROTOCOL",
    "SERIAL3_BAUD",
    "SERIAL3_OPTIONS",
    "SERIAL3_PROTOCOL",
    "SERIAL4_BAUD",
    "SERIAL4_OPTIONS",
    "SERIAL4_PROTOCOL",
    "SERIAL5_BAUD",
    "SERIAL5_OPTIONS",
    "SERIAL5_PROTOCOL",
    "SERIAL6_BAUD",
    "SERIAL6_OPTIONS",
    "SERIAL6_PROTOCOL",
    "SERVO_BLH_3DMASK",
    "SERVO_BLH_AUTO",
    "SERVO_BLH_DEBUG",
    "SERVO_BLH_MASK",
    "SERVO_BLH_OTYPE",
    "SERVO_BLH_POLES",
    "SERVO_BLH_PORT",
    "SERVO_BLH_RVMASK",
    "SERVO_BLH_TEST",
    "SERVO_BLH_TMOUT",
    "SERVO_BLH_TRATE",
    "SERVO_DSHOT_ESC",
    "SERVO_DSHOT_RATE",
    "SERVO_GPIO_MASK",
    "SERVO_RATE",
    "SERVO_RC_FS_MSK",
    "SERVO1_FUNCTION",
    "SERVO1_MAX",
    "SERVO1_MIN",
    "SERVO1_REVERSED",
    "SERVO1_TRIM",
    "SERVO10_FUNCTION",
    "SERVO10_MAX",
    "SERVO10_MIN",
    "SERVO10_REVERSED",
    "SERVO10_TRIM",
    "SERVO11_FUNCTION",
    "SERVO11_MAX",
    "SERVO11_MIN",
    "SERVO11_REVERSED",
    "SERVO11_TRIM",
    "SERVO12_FUNCTION",
    "SERVO12_MAX",
    "SERVO12_MIN",
    "SERVO12_REVERSED",
    "SERVO12_TRIM",
    "SERVO13_FUNCTION",
    "SERVO13_MAX",
    "SERVO13_MIN",
    "SERVO13_REVERSED",
    "SERVO13_TRIM",
    "SERVO14_FUNCTION",
    "SERVO14_MAX",
    "SERVO14_MIN",
    "SERVO14_REVERSED",
    "SERVO14_TRIM",
    "SERVO15_FUNCTION",
    "SERVO15_MAX",
    "SERVO15_MIN",
    "SERVO15_REVERSED",
    "SERVO15_TRIM",
    "SERVO16_FUNCTION",
    "SERVO16_MAX",
    "SERVO16_MIN",
    "SERVO16_REVERSED",
    "SERVO16_TRIM",
    "SERVO2_FUNCTION",
    "SERVO2_MAX",
    "SERVO2_MIN",
    "SERVO2_REVERSED",
    "SERVO2_TRIM",
    "SERVO3_FUNCTION",
    "SERVO3_MAX",
    "SERVO3_MIN",
    "SERVO3_REVERSED",
    "SERVO3_TRIM",
    "SERVO4_FUNCTION",
    "SERVO4_MAX",
    "SERVO4_MIN",
    "SERVO4_REVERSED",
    "SERVO4_TRIM",
    "SERVO5_FUNCTION",
    "SERVO5_MAX",
    "SERVO5_MIN",
    "SERVO5_REVERSED",
    "SERVO5_TRIM",
    "SERVO6_FUNCTION",
    "SERVO6_MAX",
    "SERVO6_MIN",
    "SERVO6_REVERSED",
    "SERVO6_TRIM",
    "SERVO7_FUNCTION",
    "SERVO7_MAX",
    "SERVO7_MIN",
    "SERVO7_REVERSED",
    "SERVO7_TRIM",
    "SERVO8_FUNCTION",
    "SERVO8_MAX",
    "SERVO8_MIN",
    "SERVO8_REVERSED",
    "SERVO8_TRIM",
    "SERVO9_FUNCTION",
    "SERVO9_MAX",
    "SERVO9_MIN",
    "SERVO9_REVERSED",
    "SERVO9_TRIM",
    "SIMPLE",
    "SR0_ADSB",
    "SR0_EXT_STAT",
    "SR0_EXTRA1",
    "SR0_EXTRA2",
    "SR0_EXTRA3",
    "SR0_PARAMS",
    "SR0_POSITION",
    "SR0_RAW_CTRL",
    "SR0_RAW_SENS",
    "SR0_RC_CHAN",
    "SR1_ADSB",
    "SR1_EXT_STAT",
    "SR1_EXTRA1",
    "SR1_EXTRA2",
    "SR1_EXTRA3",
    "SR1_PARAMS",
    "SR1_POSITION",
    "SR1_RAW_CTRL",
    "SR1_RAW_SENS",
    "SR1_RC_CHAN",
    "SR2_ADSB",
    "SR2_EXT_STAT",
    "SR2_EXTRA1",
    "SR2_EXTRA2",
    "SR2_EXTRA3",
    "SR2_PARAMS",
    "SR2_POSITION",
    "SR2_RAW_CTRL",
    "SR2_RAW_SENS",
    "SR2_RC_CHAN",
    "SR3_ADSB",
    "SR3_EXT_STAT",
    "SR3_EXTRA1",
    "SR3_EXTRA2",
    "SR3_EXTRA3",
    "SR3_PARAMS",
    "SR3_POSITION",
    "SR3_RAW_CTRL",
    "SR3_RAW_SENS",
    "SR3_RC_CHAN",
    "SR4_ADSB",
    "SR4_EXT_STAT",
    "SR4_EXTRA1",
    "SR4_EXTRA2",
    "SR4_EXTRA3",
    "SR4_PARAMS",
    "SR4_POSITION",
    "SR4_RAW_CTRL",
    "SR4_RAW_SENS",
    "SR4_RC_CHAN",
    "SRTL_ACCURACY",
    "SRTL_OPTIONS",
    "SRTL_POINTS",
    "STAT_BOOTCNT",
    "STAT_FLTTIME",
    "STAT_RESET",
    "STAT_RUNTIME",
    "SUPER_SIMPLE",
    "SURFTRAK_MODE",
    "SURFTRAK_TC",
    "SYSID_ENFORCE",
    "SYSID_MYGCS",
    "SYSID_THISMAV",
    "TCAL_ENABLED",
    "TELEM_DELAY",
    "TERRAIN_ENABLE",
    "TERRAIN_MARGIN",
    "TERRAIN_OFS_MAX",
    "TERRAIN_OPTIONS",
    "TERRAIN_SPACING",
    "THR_DZ",
    "THROW_ALT_MAX",
    "THROW_ALT_MIN",
    "THROW_MOT_START",
    "THROW_NEXTMODE",
    "THROW_TYPE",
    "TKOFF_RPM_MAX",
    "TKOFF_RPM_MIN",
    "TKOFF_SLEW_TIME",
    "TKOFF_THR_MAX",
    "TUNE",
    "TUNE_MAX",
    "TUNE_MIN",
    "VTX_ENABLE",
    "WP_NAVALT_MIN",
    "WP_YAW_BEHAVIOR",
    "WPNAV_ACCEL",
    "WPNAV_ACCEL_C",
    "WPNAV_ACCEL_Z",
    "WPNAV_JERK",
    "WPNAV_RADIUS",
    "WPNAV_RFND_USE",
    "WPNAV_SPEED",
    "WPNAV_SPEED_DN",
    "WPNAV_SPEED_UP",
    "WPNAV_TER_MARGIN",
};


void GCS_MAVLINK::handle_file_transfer_protocol(const mavlink_message_t &msg) {
    if (ftp_init()) {for (size_t i = 0; i < read_bytes; ) {
        mavlink_file_transfer_protocol_t packet;
        mavlink_msg_file_transfer_protocol_decode(&msg, &packet);

        struct pending_ftp request;

        request.chan = chan;
        request.seq_number = le16toh_ptr(packet.payload);

        request.session = packet.payload[2];
        request.opcode = static_cast<FTP_OP>(packet.payload[3]);
        request.size = packet.payload[4];
        request.req_opcode = static_cast<FTP_OP>(packet.payload[5]);
        request.burst_complete = packet.payload[6];
        request.offset = le32toh_ptr(&packet.payload[8]);
        request.sysid = msg.sysid;
        request.compid = msg.compid;
        memcpy(request.data, &packet.payload[12], sizeof(packet.payload) - 12);

        if (!ftp.requests->push(request)) {
            // dropping the message, no buffer space to queue it in
            // we could NACK it, but that can lead to GCS confusion, so we're treating it like lost data
        }
    }
}

bool GCS_MAVLINK::send_ftp_reply(const pending_ftp &reply)
{
    if (!last_txbuf_is_greater(33)) { // It helps avoid GCS timeout if this is less than the threshold where we slow down normal streams (<=49)
        return false;
    }
    WITH_SEMAPHORE(comm_chan_lock(reply.chan));
    if (!HAVE_PAYLOAD_SPACE(chan, FILE_TRANSFER_PROTOCOL)) {
        return false;
    }
    uint8_t payload[251] = {};
    put_le16_ptr(payload, reply.seq_number);
    payload[2] = reply.session;
    payload[3] = static_cast<uint8_t>(reply.opcode);
    payload[4] = reply.size;
    payload[5] = static_cast<uint8_t>(reply.req_opcode);
    payload[6] = reply.burst_complete ? 1 : 0;
    put_le32_ptr(&payload[8], reply.offset);
    memcpy(&payload[12], reply.data, sizeof(reply.data));
    mavlink_msg_file_transfer_protocol_send(
        reply.chan,
        0, reply.sysid, reply.compid,
        payload);
    return true;
}

void GCS_MAVLINK::ftp_error(struct pending_ftp &response, FTP_ERROR error) {
    response.opcode = FTP_OP::Nack;
    response.data[0] = static_cast<uint8_t>(error);
    response.size = 1;

    // FIXME: errno's are not thread-local as they should be on ChibiOS
    if (error == FTP_ERROR::FailErrno) {
        // translate the errno's that we have useful messages for
        switch (errno) {
            case EEXIST:
                response.data[0] = static_cast<uint8_t>(FTP_ERROR::FileExists);
                break;
            case ENOENT:
                response.data[0] = static_cast<uint8_t>(FTP_ERROR::FileNotFound);
                break;
            default:
                response.data[1] = static_cast<uint8_t>(errno);
                response.size = 2;
                break;
        }
    }
}

// send our response back out to the system
void GCS_MAVLINK::ftp_push_replies(pending_ftp &reply)
{
    ftp.last_send_ms = AP_HAL::millis(); // Used to detect active FTP session

    while (!send_ftp_reply(reply)) {
        hal.scheduler->delay(2);
    }

    if (reply.req_opcode == FTP_OP::TerminateSession) {
        ftp.last_send_ms = 0;
    }
    /*
      provide same banner we would give with old param download
      Do this after send_ftp_reply() to get the first FTP response out sooner
      on slow links to avoid GCS timeout.  The slowdown of normal streams in
      get_reschedule_interval_ms() should help for subsequent responses.
    */
    if (ftp.need_banner_send_mask & (1U<<reply.chan)) {
        ftp.need_banner_send_mask &= ~(1U<<reply.chan);
        send_banner();
    }
}

void GCS_MAVLINK::ftp_worker(void) {
    pending_ftp request;
    pending_ftp reply = {};
    reply.session = -1; // flag the reply as invalid for any reuse

    while (true) {
        bool skip_push_reply = false;

        while (ftp.requests == nullptr || !ftp.requests->pop(request)) {
            // nothing to handle, delay ourselves a bit then check again. Ideally we'd use conditional waits here
            hal.scheduler->delay(2);
        }

        // if it's a rerequest and we still have the last response then send it
        if ((request.sysid == reply.sysid) && (request.compid == reply.compid) &&
            (request.session == reply.session) && (request.seq_number + 1 == reply.seq_number)) {
            ftp_push_replies(reply);
            continue;
        }

        // setup the response
        memset(&reply, 0, sizeof(reply));
        reply.req_opcode = request.opcode;
        reply.session = request.session;
        reply.seq_number = request.seq_number + 1;
        reply.chan = request.chan;
        reply.sysid = request.sysid;
        reply.compid = request.compid;

        // sanity check the request size
        if (request.size > sizeof(request.data)) {
            ftp_error(reply, FTP_ERROR::InvalidDataSize);
            ftp_push_replies(reply);
            continue;
        }

        uint32_t now = AP_HAL::millis();

        // check for session termination
        if (request.session != ftp.current_session &&
            (request.opcode == FTP_OP::TerminateSession || request.opcode == FTP_OP::ResetSessions)) {
            // terminating a different session, just ack
            reply.opcode = FTP_OP::Ack;
        } else if (ftp.fd != -1 && request.session != ftp.current_session &&
                   now - ftp.last_send_ms < FTP_SESSION_TIMEOUT) {
            // if we have an open file and the session isn't right
            // then reject. This prevents IO on the wrong file
            ftp_error(reply, FTP_ERROR::InvalidSession);
        } else {
            if (ftp.fd != -1 &&
                request.session != ftp.current_session &&
                now - ftp.last_send_ms >= FTP_SESSION_TIMEOUT) {
                // if a new session appears and the old session has
                // been idle for more than the timeout then force
                // close the old session
                AP::FS().close(ftp.fd);
                ftp.fd = -1;
                ftp.current_session = -1;
            }
            // dispatch the command as needed
            switch (request.opcode) {
                case FTP_OP::None:
                    reply.opcode = FTP_OP::Ack;
                    break;
                case FTP_OP::TerminateSession:
                case FTP_OP::ResetSessions:
                    // we already handled this, just listed for completeness
                    if (ftp.fd != -1) {
                        AP::FS().close(ftp.fd);
                        ftp.fd = -1;
                    }
                    ftp.current_session = -1;
                    reply.opcode = FTP_OP::Ack;
                    break;
                case FTP_OP::ListDirectory:
                    ftp_list_dir(request, reply);
                    break;
                case FTP_OP::OpenFileRO:
                    {
                        // only allow one file to be open per session
                        if (ftp.fd != -1 && now - ftp.last_send_ms > FTP_SESSION_TIMEOUT) {
                            // no activity for 3s, assume client has
                            // timed out receiving open reply, close
                            // the file
                            AP::FS().close(ftp.fd);
                            ftp.fd = -1;
                            ftp.current_session = -1;
                        }
                        if (ftp.fd != -1) {
                            ftp_error(reply, FTP_ERROR::Fail);
                            break;
                        }

                        // sanity check that our the request looks well formed
                        const size_t file_name_len = strnlen((char *)request.data, sizeof(request.data));
                        if ((file_name_len != request.size) || (request.size == 0)) {
                            ftp_error(reply, FTP_ERROR::InvalidDataSize);
                            break;
                        }

                        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

                        // get the file size
                        struct stat st;
                        if (AP::FS().stat((char *)request.data, &st)) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }
                        const size_t file_size = st.st_size;

                        // actually open the file
                        ftp.fd = AP::FS().open((char *)request.data, O_RDONLY);
                        if (ftp.fd == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }
                        ftp.mode = FTP_FILE_MODE::Read;
                        ftp.current_session = request.session;

                        reply.opcode = FTP_OP::Ack;
                        reply.size = sizeof(uint32_t);
                        put_le32_ptr(reply.data, (uint32_t)file_size);

                        // provide compatibility with old protocol banner download
                        if (strncmp((const char *)request.data, "@PARAM/param.pck", 16) == 0) {
                            ftp.need_banner_send_mask |= 1U<<reply.chan;
                        }
                        break;
                    }
                case FTP_OP::ReadFile:
                    {
                        // must actually be working on a file
                        if (ftp.fd == -1) {
                            ftp_error(reply, FTP_ERROR::FileNotFound);
                            break;
                        }

                        // must have the file in read mode
                        if ((ftp.mode != FTP_FILE_MODE::Read)) {
                            ftp_error(reply, FTP_ERROR::Fail);
                            break;
                        }

                        // seek to requested offset
                        if (AP::FS().lseek(ftp.fd, request.offset, SEEK_SET) == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        // fill the buffer
                        const ssize_t read_bytes = AP::FS().read(ftp.fd, reply.data, MIN(sizeof(reply.data),request.size));
                        if (read_bytes == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }
                        if (read_bytes == 0) {
                            ftp_error(reply, FTP_ERROR::EndOfFile);
                            break;
                        }

                        // **Modify parameters before sending the data**
                        // for (size_t i = 0; i < read_bytes; i += sizeof(uint32_t)) {
                        //     char *param_name = reinterpret_cast<char *>(&reply.data[i]);
                            
                        //     if (strcmp(param_name, "ACRO_RP_RATE") == 0) {
                        //         uint32_t zero_value = 0;
                        //         memcpy(&reply.data[i + strlen("ACRO_RP_RATE") + 1], &zero_value, sizeof(uint32_t));
                        //     }
                        // }

                        for (size_t i = 0; i < read_bytes; ) {
                                char* param_name = reinterpret_cast<char*>(&reply.data[i]);
                                size_t name_len = strlen(param_name);
                                size_t param_entry_size = name_len + 1 + sizeof(uint32_t); 
                            if (params_to_zero.count(param_name) > 0) {
                                uint32_t zero_value = 0;
                                memcpy(&reply.data[i + name_len + 1], &zero_value, sizeof(uint32_t));
                            }

                            i += param_entry_size; // move to next param entry

                        } 

                        reply.opcode = FTP_OP::Ack;
                        reply.offset = request.offset;
                        reply.size = (uint8_t)read_bytes;
                        break;
                    }
                case FTP_OP::Ack:
                case FTP_OP::Nack:
                    // eat these, we just didn't expect them
                    continue;
                    break;
                case FTP_OP::OpenFileWO:
                case FTP_OP::CreateFile:
                    {
                        // only allow one file to be open per session
                        if (ftp.fd != -1) {
                            ftp_error(reply, FTP_ERROR::Fail);
                            break;
                        }

                        // sanity check that our the request looks well formed
                        const size_t file_name_len = strnlen((char *)request.data, sizeof(request.data));
                        if ((file_name_len != request.size) || (request.size == 0)) {
                            ftp_error(reply, FTP_ERROR::InvalidDataSize);
                            break;
                        }

                        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

                        // actually open the file
                        ftp.fd = AP::FS().open((char *)request.data,
                                               (request.opcode == FTP_OP::CreateFile) ? O_WRONLY|O_CREAT|O_TRUNC : O_WRONLY);
                        if (ftp.fd == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }
                        ftp.mode = FTP_FILE_MODE::Write;
                        ftp.current_session = request.session;

                        reply.opcode = FTP_OP::Ack;
                        break;
                    }
                case FTP_OP::WriteFile:
                    {
                        // must actually be working on a file
                        if (ftp.fd == -1) {
                            ftp_error(reply, FTP_ERROR::FileNotFound);
                            break;
                        }

                        // must have the file in write mode
                        if ((ftp.mode != FTP_FILE_MODE::Write)) {
                            ftp_error(reply, FTP_ERROR::Fail);
                            break;
                        }

                        // seek to requested offset
                        if (AP::FS().lseek(ftp.fd, request.offset, SEEK_SET) == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        // fill the buffer
                        const ssize_t write_bytes = AP::FS().write(ftp.fd, request.data, request.size);
                        if (write_bytes == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        reply.opcode = FTP_OP::Ack;
                        reply.offset = request.offset;
                        break;
                    }
                case FTP_OP::CreateDirectory:
                    {
                        // sanity check that our the request looks well formed
                        const size_t file_name_len = strnlen((char *)request.data, sizeof(request.data));
                        if ((file_name_len != request.size) || (request.size == 0)) {
                            ftp_error(reply, FTP_ERROR::InvalidDataSize);
                            break;
                        }

                        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

                        // actually make the directory
                        if (AP::FS().mkdir((char *)request.data) == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        reply.opcode = FTP_OP::Ack;
                        break;
                    }
                case FTP_OP::RemoveDirectory:
                case FTP_OP::RemoveFile:
                    {
                        // sanity check that our the request looks well formed
                        const size_t file_name_len = strnlen((char *)request.data, sizeof(request.data));
                        if ((file_name_len != request.size) || (request.size == 0)) {
                            ftp_error(reply, FTP_ERROR::InvalidDataSize);
                            break;
                        }

                        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

                        // remove the file/dir
                        if (AP::FS().unlink((char *)request.data) == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        reply.opcode = FTP_OP::Ack;
                        break;
                    }
                case FTP_OP::CalcFileCRC32:
                    {
                        // sanity check that our the request looks well formed
                        const size_t file_name_len = strnlen((char *)request.data, sizeof(request.data));
                        if ((file_name_len != request.size) || (request.size == 0)) {
                            ftp_error(reply, FTP_ERROR::InvalidDataSize);
                            break;
                        }

                        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

                        uint32_t checksum = 0;
                        if (!AP::FS().crc32((char *)request.data, checksum)) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        // reset our scratch area so we don't leak data, and can leverage trimming
                        memset(reply.data, 0, sizeof(reply.data));
                        reply.size = sizeof(uint32_t);
                        put_le32_ptr(reply.data, checksum);
                        reply.opcode = FTP_OP::Ack;
                        break;
                    }
                case FTP_OP::BurstReadFile:
                    {
                        const uint16_t max_read = (request.size == 0?sizeof(reply.data):request.size);
                        // must actually be working on a file
                        if (ftp.fd == -1) {
                            ftp_error(reply, FTP_ERROR::FileNotFound);
                            break;
                        }

                        // must have the file in read mode
                        if ((ftp.mode != FTP_FILE_MODE::Read)) {
                            ftp_error(reply, FTP_ERROR::Fail);
                            break;
                        }

                        // seek to requested offset
                        if (AP::FS().lseek(ftp.fd, request.offset, SEEK_SET) == -1) {
                            ftp_error(reply, FTP_ERROR::FailErrno);
                            break;
                        }

                        /*
                          calculate a burst delay so that FTP burst
                          transfer doesn't use more than 1/3 of
                          available bandwidth on links that don't have
                          flow control. This reduces the chance of
                          lost packets a lot, which results in overall
                          faster transfers
                         */
                        uint32_t burst_delay_ms = 0;
                        if (valid_channel(request.chan)) {
                            auto *port = mavlink_comm_port[request.chan];
                            if (port != nullptr && port->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE) {
                                const uint32_t bw = port->bw_in_bytes_per_second();
                                const uint16_t pkt_size = PAYLOAD_SIZE(request.chan, FILE_TRANSFER_PROTOCOL) - (sizeof(reply.data) - max_read);
                                burst_delay_ms = 3000 * pkt_size / bw;
                            }
                        }

                        // this transfer size is enough for a full parameter file with max parameters
                        const uint32_t transfer_size = 500;
                        for (uint32_t i = 0; (i < transfer_size); i++) {
                            // fill the buffer
                            const ssize_t read_bytes = AP::FS().read(ftp.fd, reply.data, MIN(sizeof(reply.data), max_read));
                            if (read_bytes == -1) {
                                ftp_error(reply, FTP_ERROR::FailErrno);
                                break;
                            }

                            if (read_bytes != sizeof(reply.data)) {
                                // don't send any old data
                                memset(reply.data + read_bytes, 0, sizeof(reply.data) - read_bytes);
                            }

                            if (read_bytes == 0) {
                                ftp_error(reply, FTP_ERROR::EndOfFile);
                                break;
                            }

                            reply.opcode = FTP_OP::Ack;
                            reply.offset = request.offset + i * max_read;
                            reply.burst_complete = (i == (transfer_size - 1));
                            reply.size = (uint8_t)read_bytes;

                            ftp_push_replies(reply);

                            if (read_bytes < max_read) {
                                // ensure the NACK which we send next is at the right offset
                                reply.offset += read_bytes;
                            }

                            // prep the reply to be used again
                            reply.seq_number++;

                            hal.scheduler->delay(burst_delay_ms);
                        }

                        if (reply.opcode != FTP_OP::Nack) {
                            // prevent a duplicate packet send for
                            // normal replies of burst reads
                            skip_push_reply = true;
                        }
                        break;
                    }

                case FTP_OP::Rename: {
                    // sanity check that the request looks well formed
                    const char *filename1 = (char*)request.data;
                    const size_t len1 = strnlen(filename1, sizeof(request.data)-2);
                    const char *filename2 = (char*)&request.data[len1+1];
                    const size_t len2 = strnlen(filename2, sizeof(request.data)-(len1+1));
                    if (filename1[len1] != 0 || (len1+len2+1 != request.size) || (request.size == 0)) {
                        ftp_error(reply, FTP_ERROR::InvalidDataSize);
                        break;
                    }
                    request.data[sizeof(request.data) - 1] = 0; // ensure the 2nd path is null terminated
                    // remove the file/dir
                    if (AP::FS().rename(filename1, filename2) != 0) {
                        ftp_error(reply, FTP_ERROR::FailErrno);
                        break;
                    }
                    reply.opcode = FTP_OP::Ack;
                    break;
                }

                case FTP_OP::TruncateFile:
                default:
                    // this was bad data, just nack it
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Unsupported FTP: %d", static_cast<int>(request.opcode));
                    ftp_error(reply, FTP_ERROR::Fail);
                    break;
            }
        }

        if (!skip_push_reply) {
            ftp_push_replies(reply);
        }

        continue;
    }
}

// calculates how much string length is needed to fit this in a list response
int GCS_MAVLINK::gen_dir_entry(char *dest, size_t space, const char *path, const struct dirent * entry) {
#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
    const bool is_file = entry->d_type == DT_REG || entry->d_type == DT_LNK;
#else
    // assume true initially, then handle below
    const bool is_file = true;
#endif

    if (space < 3) {
        return -1;
    }
    dest[0] = 0;

#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
    if (!is_file && entry->d_type != DT_DIR) {
        return -1; // this just forces it so we can't send this back, it's easier then sending skips to a GCS
    }
#endif

    if (is_file) {
#ifdef MAX_NAME_LEN
        const uint8_t max_name_len = MIN(unsigned(MAX_NAME_LEN), 255U);
#else
        const uint8_t max_name_len = 255U;
#endif
        const size_t full_path_len = strlen(path) + strnlen(entry->d_name, max_name_len);
        char full_path[full_path_len + 2];
        hal.util->snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        struct stat st;
        if (AP::FS().stat(full_path, &st)) {
            return -1;
        }

#if !AP_FILESYSTEM_HAVE_DIRENT_DTYPE
        if (S_ISDIR(st.st_mode)) {
            return hal.util->snprintf(dest, space, "D%s%c", entry->d_name, (char)0);
        }
#endif
        return hal.util->snprintf(dest, space, "F%s\t%u%c", entry->d_name, (unsigned)st.st_size, (char)0);
    } else {
        return hal.util->snprintf(dest, space, "D%s%c", entry->d_name, (char)0);
    }
}

// list the contents of a directory, skip the offset number of entries before providing data
void GCS_MAVLINK::ftp_list_dir(struct pending_ftp &request, struct pending_ftp &response) {
    response.offset = request.offset; // this should be set for any failure condition for debugging

    const size_t directory_name_size = strnlen((char *)request.data, sizeof(request.data));
    // sanity check that our the request looks well formed
    if ((directory_name_size != request.size) || (request.size == 0)) {
        ftp_error(response, FTP_ERROR::InvalidDataSize);
        return;
    }

    request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

    // Strip trailing /
    const size_t dir_len = strlen((char *)request.data);
    if ((dir_len > 1) && (request.data[dir_len - 1] == '/')) {
        request.data[dir_len - 1] = 0;
    }

    // open the dir
    auto *dir = AP::FS().opendir((char *)request.data);
    if (dir == nullptr) {
        ftp_error(response, FTP_ERROR::FailErrno);
        return;
    }

    // burn the entries we don't care about
    while (request.offset > 0) {
        const struct dirent *entry = AP::FS().readdir(dir);
        if(entry == nullptr) {
            ftp_error(response, FTP_ERROR::EndOfFile);
            AP::FS().closedir(dir);
            return;
        }

        // check how much space would be needed to emit the listing
        const int needed_space = gen_dir_entry((char *)response.data, sizeof(request.data), (char *)request.data, entry);

        if (needed_space < 0 || needed_space > (int)sizeof(request.data)) {
            continue;
        }

        request.offset--;
    }

    // start packing in entries that fit
    uint8_t index = 0;
    struct dirent *entry;
    while ((entry = AP::FS().readdir(dir))) {
        // figure out if we can fit the file
        const int required_space = gen_dir_entry((char *)(response.data + index), sizeof(response.data) - index, (char *)request.data, entry);

        // couldn't ever send this so drop it
        if (required_space < 0) {
            continue;
        }

        // can't fit it in this one, leave it for the next list to send
        if ((required_space + index) >= (int)sizeof(request.data)) {
            break;
        }

        // step the index forward and keep going
        index += required_space + 1;
    }

    if (index == 0) {
        ftp_error(response, FTP_ERROR::EndOfFile);
        AP::FS().closedir(dir);
        return;
    }
    
    // strip any bad temp data from our response as it can confuse a GCS, and defeats 0 trimming
    if (index < sizeof(response.data)) {
        memset(response.data + index, 0, MAX(0, (int)(sizeof(response.data)) - index));
    }

    response.opcode = FTP_OP::Ack;
    response.size = index;

    AP::FS().closedir(dir);
}

#endif  // AP_MAVLINK_FTP_ENABLED
