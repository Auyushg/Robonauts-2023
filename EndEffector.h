/*******************************************************************************
 *
 * File: Subsystem.h
 *
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableBuilder.h"
#include "FlightBase/RSubsystem.h"
#include "RobonautsLibrary/RSpeedController.h"
#include "RobonautsLibrary/OIButton.h"
#include "RobonautsLibrary/OIAxis.h"
#include "frc/Preferences.h"
#include "frc/Servo.h"

class EndEffector : public RSubsystem, public wpi::Sendable
{
  public:
    EndEffector(std::string name);
    ~EndEffector(void);

    void rollerIn();
    void rollerOut();
    void rollerOff();

    void slurpPressed();
    void slurpReleased();
    void spitPressed();
    void spitReleased();
    void reset();

    double getRollerVel();
    void setEndRollerMotorCurrentLimits(double limit);

    void initCurrentLimits(double current_limit, double spit_current_limit);

  protected:
    void RobotInit(void);      // Called once upon creation of control
    void RobotPeriodic(void);  // called every cycle

    void AutonomousInit(void); // called once when system transitions from disabled to enabled in autonomous mode

    void TeleopInit(void);     // called once when system transitions from disabled to enabled in teleop mode

    void initPreferences();
    void readPreferences();
    void addLogVars();

    virtual void InitSendable(wpi::SendableBuilder &builder) override;

  private:

    void handleOI();
    void writeEffectors();
    void readSensors();

    RSpeedController * m_end_roller;

    //Button to turn roller motor
    OIButtonSet m_end_roll_in_btn {"roller_in"};
    OIButtonSet m_end_roll_out_btn {"roller_out"};

    OIButtonSet m_slurp_btn{"slurp"};
    OIButtonSet m_spit_btn{"spit"};
    OIButtonSet m_reset_btn{"reset"};

    //motor commands
    double roller_out_cmd {0};
    double roller_in_cmd{0};
    double roller_off_cmd {0};
    double m_roller_cmd{0.0};  // writing to the roller motor

    // sensors (from motors)
    double m_roller_dc{0.0};
    double m_roller_curr{0.0};
    double m_roller_vel{0.0};

    // for reducing traffic on can bus (presumable to work overruns)
    int m_getter_count{0};
    const int c_getter_max{3};

    // current limits
    double m_initial_current_limit;
    double m_current_limit;

    bool m_reduce_current{false};
    double m_reduce_current_time;

};

void luaRegisterEndEffector();


