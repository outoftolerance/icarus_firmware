#ifndef MISSION_STATE_H
#define MISSION_STATE_H

#include <SimpleUtils.h>
#include <Timer.h>

/**
 * @brief      Enumerator defines different states in the HAB mission
 */
enum MISSION_STATES 
{
    STAGING = 0,
    TAKEOFF,
    ASCENDING,
    DESCENDING,
    LANDING,
    RECOVERY,
    RECOVERED
};

/**
 * @brief A type to report mission state functions
 */
typedef struct
{
    unsigned long telemetry_log_interval;
    unsigned long telemetry_report_interval;
    bool beeper_enabled;
    bool strobe_enabled;
    bool cellular_enabled;
} MissionStateFunction;

/**
 * Define the behaviour of the system under different states
 */
#define STAGING_TELEMETRY_LOG_INTERVAL 5000
#define STAGING_TELEMETRY_REPORT_INTERVAL 5000
#define STAGING_BEEPER_ENABLED false
#define STAGING_STROBE_ENABLED true
#define STAGING_CELLULAR_ENABLED true

#define TAKEOFF_TELEMETRY_LOG_INTERVAL 5000
#define TAKEOFF_TELEMETRY_REPORT_INTERVAL 15000
#define TAKEOFF_BEEPER_ENABLED true
#define TAKEOFF_STROBE_ENABLED true
#define TAKEOFF_CELLULAR_ENABLED true

#define ASCENDING_TELEMETRY_LOG_INTERVAL 5000
#define ASCENDING_TELEMETRY_REPORT_INTERVAL 30000
#define ASCENDING_BEEPER_ENABLED false
#define ASCENDING_STROBE_ENABLED false
#define ASCENDING_CELLULAR_ENABLED false

#define DESCENDING_TELEMETRY_LOG_INTERVAL 5000
#define DESCENDING_TELEMETRY_REPORT_INTERVAL 30000
#define DESCENDING_BEEPER_ENABLED false
#define DESCENDING_STROBE_ENABLED false
#define DESCENDING_CELLULAR_ENABLED false

#define LANDING_TELEMETRY_LOG_INTERVAL 5000
#define LANDING_TELEMETRY_REPORT_INTERVAL 5000
#define LANDING_BEEPER_ENABLED true
#define LANDING_STROBE_ENABLED true
#define LANDING_CELLULAR_ENABLED true

#define RECOVERY_TELEMETRY_LOG_INTERVAL 5000
#define RECOVERY_TELEMETRY_REPORT_INTERVAL 30000
#define RECOVERY_BEEPER_ENABLED true
#define RECOVERY_STROBE_ENABLED true
#define RECOVERY_CELLULAR_ENABLED false

#define RECOVERED_TELEMETRY_LOG_INTERVAL 5000
#define RECOVERED_TELEMETRY_REPORT_INTERVAL 30000
#define RECOVERED_BEEPER_ENABLED false
#define RECOVERED_STROBE_ENABLED false
#define RECOVERED_CELLULAR_ENABLED true

#define TERMINAL_ALTITUDE 1000
#define SILENCE_DETECTION_TIMEOUT_INTERVAL 5000
#define DESCENT_DETECTION_TIMEOUT_INTERVAL 5000
#define LANDING_DETECTION_TIMEOUT_INTERVAL 5000
#define LANDED_ALTITUDE_DEADZONE 5
#define RECOVERED_INNACTIVITY_TIMEOUT_INTERVAL 300000

/**
 * @brief Mission state class
 * @note Provides state machine tracking for mission and associated functions
 */
class MissionState
{
    public:
        MissionState();
        ~MissionState();

        /**
         * @brief Update the mission state with latest information
         * @return bool True if successfully updated, false if error reported
         */
        bool update(const SimpleUtils::TelemetryStruct& telemetry);

        /**
         * @brief Allows setting of the mission state to a particular state
         * @return bool True if success, false if error reported
         */
        bool set(const MISSION_STATES state);

        /**
         * @brief Returns the current state of the mission
         * @return MissionStates enumerated mission state
         */
        MISSION_STATES get();

        /**
         * @brief Returns the current state of the functions related to mission states
         * @param MissionStateFunction type with values based on current mission state
         */
        void getFunction(MissionStateFunction& function);

    private:
        MISSION_STATES current_mission_state_;                  /**< Current enumerated state of the mission */
        MissionStateFunction current_mission_state_function_;   /**< Functionality of the current mission state */
        Timer recovered_timeout_;                               /**< Timer for recovered mode time-out */
        Timer descent_timeout_;                                 /**< Timer for checking descent */
        Timer landing_timeout_;                                 /**< Timer for checking landing */
        float previous_altitude_;                               /**< Stores previous loop's altitude */
};

#endif
