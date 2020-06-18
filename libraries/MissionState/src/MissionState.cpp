#include "MissionState.h"

MissionState::MissionState()
{
	descent_timeout_.setInterval(DESCENT_DETECTION_TIMEOUT_INTERVAL);
	recovered_timeout_.setInterval(RECOVERED_INNACTIVITY_TIMEOUT_INTERVAL);
	landing_timeout_.setInterval(LANDING_DETECTION_TIMEOUT_INTERVAL);
}

MissionState::~MissionState()
{
	descent_timeout_.stop();
	descent_timeout_.forceReset();

	recovered_timeout_.stop();
	recovered_timeout_.forceReset();

	landing_timeout_.stop();
	landing_timeout_.forceReset();
}

bool MissionState::update(const SimpleUtils::TelemetryStruct& telemetry)
{
	switch(current_mission_state_)
	{
		case MISSION_STATES::STAGING:
		case MISSION_STATES::TAKEOFF:
			if(telemetry.altitude_relative >= TERMINAL_ALTITUDE)
			{
				current_mission_state_ = MISSION_STATES::ASCENDING;
			}
		case MISSION_STATES::ASCENDING:
			if(telemetry.altitude < previous_altitude_)
			{
				if(descent_timeout_.isStarted())
				{
					if(descent_timeout_.check())
					{
						current_mission_state_ = MISSION_STATES::DESCENDING;
						descent_timeout_.stop();
						descent_timeout_.reset();
					}
				}
				else
				{
					descent_timeout_.start();
				}
			}
			else
			{
				if(descent_timeout_.isStarted())
				{
					descent_timeout_.stop();
					descent_timeout_.forceReset();
				}
			}

			previous_altitude_ = telemetry.altitude;
			break;
		case MISSION_STATES::DESCENDING:
			if(telemetry.altitude_relative <= TERMINAL_ALTITUDE)
			{
				current_mission_state_ = MISSION_STATES::LANDING;
			}
			break;
		case MISSION_STATES::LANDING:
			if(telemetry.altitude <= previous_altitude_ + LANDED_ALTITUDE_DEADZONE && telemetry.altitude >= previous_altitude_ - LANDED_ALTITUDE_DEADZONE)
			{
				if(landing_timeout_.isStarted())
				{
					if(landing_timeout_.check())
					{
						current_mission_state_ = MISSION_STATES::RECOVERY;
						landing_timeout_.stop();
						landing_timeout_.reset();
					}
				}
				else
				{
					landing_timeout_.start();
				}
			}
			else
			{
				if(landing_timeout_.isStarted())
				{
					landing_timeout_.stop();
					landing_timeout_.forceReset();
				}
			}

			previous_altitude_ = telemetry.altitude;
			break;
		case MISSION_STATES::RECOVERY:
			//Todo, how to detect recovery?
		case MISSION_STATES::RECOVERED:
			if(recovered_timeout_.isStarted())
			{
				if(recovered_timeout_.check())
				{
					current_mission_state_ = MISSION_STATES::RECOVERY;
					recovered_timeout_.stop();
					recovered_timeout_.reset();
				}
			}
			else
			{
				recovered_timeout_.start();
			}
			break;
	}

	return true;
}

bool MissionState::set(const MISSION_STATES state)
{
	current_mission_state_ = state;

	return true;
}

MISSION_STATES MissionState::get()
{
	return current_mission_state_;
}

void MissionState::getFunction(MissionStateFunction& function)
{
	switch(current_mission_state_)
	{
		case MISSION_STATES::STAGING:
			function.telemetry_log_interval = STAGING_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = STAGING_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = STAGING_BEEPER_ENABLED;
			function.strobe_enabled = STAGING_STROBE_ENABLED;
			break;
		case MISSION_STATES::TAKEOFF:
			function.telemetry_log_interval = TAKEOFF_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = TAKEOFF_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = TAKEOFF_BEEPER_ENABLED;
			function.strobe_enabled = TAKEOFF_STROBE_ENABLED;
			break;
		case MISSION_STATES::ASCENDING:
			function.telemetry_log_interval = ASCENDING_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = ASCENDING_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = ASCENDING_BEEPER_ENABLED;
			function.strobe_enabled = ASCENDING_STROBE_ENABLED;
			break;
		case MISSION_STATES::DESCENDING:
			function.telemetry_log_interval = DESCENDING_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = DESCENDING_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = DESCENDING_BEEPER_ENABLED;
			function.strobe_enabled = DESCENDING_STROBE_ENABLED;
			break;
		case MISSION_STATES::LANDING:
			function.telemetry_log_interval = LANDING_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = LANDING_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = LANDING_BEEPER_ENABLED;
			function.strobe_enabled = LANDING_STROBE_ENABLED;
			break;
		case MISSION_STATES::RECOVERY:
			function.telemetry_log_interval = RECOVERY_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = RECOVERY_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = RECOVERY_BEEPER_ENABLED;
			function.strobe_enabled = RECOVERY_STROBE_ENABLED;
			break;
		case MISSION_STATES::RECOVERED:
			function.telemetry_log_interval = RECOVERY_TELEMETRY_LOG_INTERVAL;
			function.telemetry_report_interval = RECOVERY_TELEMETRY_REPORT_INTERVAL;
			function.beeper_enabled = RECOVERY_BEEPER_ENABLED;
			function.strobe_enabled = RECOVERY_STROBE_ENABLED;
			break;
	}
}