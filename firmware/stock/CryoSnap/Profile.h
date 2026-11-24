#ifndef PROFILE_H
#define PROFILE_H

// ============================================
// Stock dev kit version of CryoSnap v2.1.0
// ============================================
// This folder is already set up for: Stock dev kit.
// Open CryoSnap.ino and upload. There is nothing to select.
// Other versions: the firmware/ folder of the repository, or the
// Releases page (one download per version).
//
// The profile blocks below hold the defaults for every version;
// only the one selected here is compiled.

#define PROFILE_STOCK             0
#define PROFILE_MICROCENTUM_MC18  1
#define PROFILE_MICROSCOPE_STAGE  2

#ifndef CRYOSNAP_PROFILE
#define CRYOSNAP_PROFILE  PROFILE_STOCK
#endif

// --------------------------------------------
#if CRYOSNAP_PROFILE == PROFILE_STOCK
// --------------------------------------------
// No overrides: Config.h defaults are the stock kit.
#define FW_PROFILE_SUFFIX ""

// --------------------------------------------
#elif CRYOSNAP_PROFILE == PROFILE_MICROCENTUM_MC18
// --------------------------------------------
// Validated on one MC-18 with the stock CryoSnap heat sink and fan,
// ~23 C ambient, insulated and uninsulated. MC-08 / MC-24 / MC-36:
// set vmax at the console (see the startup banner), then `save`.
#define FW_PROFILE_SUFFIX "-mc18"

// Cool mode locked (fixed polarity on the bypass terminals), startup
// banner, pot range widened to reach the MC-18 cold floor (~-29 C).
#ifndef MC_EDITION
#define MC_EDITION 1
#endif
#ifndef POT_TEMP_MIN
#define POT_TEMP_MIN  -30.0f
#endif

// Preload the PID integrator with the measured hold current on every
// setpoint change and enable edge. See Feedforward.h.
#ifndef ENABLE_MC_FEEDFORWARD
#define ENABLE_MC_FEEDFORWARD 1
#endif

// Integral zone: integration frozen while |error| > 2 C. With the
// feedforward preload supplying the hold current, nothing needs to
// wind during a long approach; without it, approach windup caused
// 0.6-3.6 C overshoot.
#ifndef PID_IZONE_C
#define PID_IZONE_C 2.0f
#endif

// Converter mode switch moved for the MC-18 current range. Forced PWM
// tracks within 9 mA from 750 mA up but cannot go under ~0.34 A; pulse
// mode tracks within 10 mA up to 500 mA but sticks near 0.73 A for
// 700-800 mA commands.
#ifndef TPS_FPWM_ON_MA
#define TPS_FPWM_ON_MA   750
#endif
#ifndef TPS_FPWM_OFF_MA
#define TPS_FPWM_OFF_MA  700
#endif

// Drive limits. Cooling gain flattens past ~1.6-1.7 A; datasheet Imax
// 1.82 A and Vmax 2.28 V at Th 27 C. NOTE: a V_limit <= 5.5 V disables
// the no-supply V_limit check (the I2C NACK path still detects it).
#ifndef DEFAULT_MAX_CURRENT
#define DEFAULT_MAX_CURRENT   1.7f
#endif
#ifndef DEFAULT_MAX_VOLTAGE
#define DEFAULT_MAX_VOLTAGE   2.28f
#endif

// PID gains from step-response fits near ambient (gain ~70 C/A, time
// constant ~1.8 s, dead time ~0.5 s). The stock Kp of 200 is past the
// stability limit on this cooler.
#ifndef DEFAULT_KP
#define DEFAULT_KP    17.0f
#endif
#ifndef DEFAULT_KI
#define DEFAULT_KI     9.0f
#endif

// Saved settings from another profile must not load into this one.
#ifndef SETTINGS_VERSION
#define SETTINGS_VERSION  5
#endif

// --------------------------------------------
#elif CRYOSNAP_PROFILE == PROFILE_MICROSCOPE_STAGE
// --------------------------------------------
// Control probe: Amphenol TK95F103W (10k, curve F) in the sample water.
#define FW_PROFILE_SUFFIX "-stage"

#ifndef NTC_SENSOR
#define NTC_SENSOR  NTC_SENSOR_TK95F
#endif

// `pause` freezes the cold-side reading fed to control and telemetry,
// so pulling the probe for imaging cannot make the loop chase ambient.
// Hot-side and ambient channels stay live for the safety checks.
#ifndef ENABLE_PAUSE_RESUME
#define ENABLE_PAUSE_RESUME 1
#endif

// Saved settings from another profile must not load into this one.
#ifndef SETTINGS_VERSION
#define SETTINGS_VERSION  6
#endif

#else
#error "CRYOSNAP_PROFILE must be PROFILE_STOCK, PROFILE_MICROCENTUM_MC18 or PROFILE_MICROSCOPE_STAGE"
#endif

#endif // PROFILE_H
