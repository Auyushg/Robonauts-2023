/*******************************************************************************
 *
 * File: EndEffector.cpp
 *
 * Written by:
 *     The Robonauts
 *     FRC Team 118
 *     NASA, Johnson Space Center
 *     Clear Creek Independent School District
 *
 ******************************************************************************/

#include "FlightBase/LuaState.h"
#include "EndEffector.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "gsu/Advisory.h"
#include "LightsControl.h"
#include "ConeCubeControl.h"

// CLASS TO DO EFFECTOR FUNCTIONS
// Once implemented, delete the line(s)
// Replace hard code values and create variables for:
//    -current limits for having a game piece and not having a game piece
//    -value vacuum motor turns on
//    -servo up and servo down positions
// Add the values above to preferences
// Create a value for servo_cmd, which gets a valve of servo up or servo dn
// Create a HandleOI class and get all joystick presses there.  Add HandleOI to
//    RobotPeriodic and wrap it with "if(!IsAutonomous())" check swerveControl.cpp for example
// Create functions that can be called from Lua script
//    - command servo argument should be up or down (as a string)
//    - command solenoid open or closed (as a string)
//    - add them into luaRegisterEndEffector
// Get rid of dead code
//    - Remove m_axis (is it a holdover from tutorial?)
//    - If end_effector motor is no longer a part of this, delete it and buttons that command it
// Do we need an OI for servo_unslant or similar?
//    - Don't think we want
// Initialize values in TeleopInit and AutonomousInit
//    - Vacuum pump on (or off)
//    - slant servo up (or down)
//    - Solenoid closed (or open)
// Create a function that reads any telemetry that gets called at the beginning of RobotPeriodic
//    - Reading current might be only think for now
// For consideration: Get rid of the references to Arm and move arm to the Arm class

/* Create and initialize all of the elements of the subsystem */
EndEffector::EndEffector(std::string ctrl_name)
    : RSubsystem(ctrl_name)
    ,m_end_roller (nullptr)
{
    Advisory::pinfo("========================= Creating SubSystem [%s] =========================\n", ctrl_name.c_str());

    // setup buttons
   addOI(&m_end_roll_in_btn);
   addOI(&m_end_roll_out_btn);
   addOI(&m_slurp_btn);
   addOI(&m_spit_btn);
   addOI(&m_reset_btn);

   addRSpeedController("end_roller", &m_end_roller);

   m_current_limit = 1;
   m_initial_current_limit = 40;
}

/*  Destructor, releases any resources allocated by this class */
EndEffector::~EndEffector(void) {}

/* Initializes preferences that appear on the ShuffleBoard

   In the example below, button_cmd is a double. Doubles, floats, integers, strings, and booleans can be used with
   preferences

   if (!frc::Preferences::ContainsKey("EndEffector/button_cmd")) {
        frc::Preferences::SetDouble("EndEffector/button_cmd", button_cmd);
   }
*/
void EndEffector::initPreferences() {
    if (!frc::Preferences::ContainsKey("EndEffector/roller_in_cmd")) {
        frc::Preferences::SetDouble("EndEffector/roller_in_cmd", roller_in_cmd);
    }
    if (!frc::Preferences::ContainsKey("EndEffector/roller_out_cmd")) {
        frc::Preferences::SetDouble("EndEffector/roller_out_cmd", roller_out_cmd);
    }
    if (!frc::Preferences::ContainsKey("EndEffector/roller_off_cmd")) {
        frc::Preferences::SetDouble("EndEffector/roller_off_cmd", roller_off_cmd);
    }
}

/* Reads preferences that appear on the ShuffleBoard

   In this example the preference "EndEffector/button_cmd" is read.  If the value does not exist, the value of
   button_cmd is returned.  Doubles, floats, integers, strings, and booleans can be used with preferences.

   button_cmd = frc::Preferences::GetDouble("EndEffector/button_cmd", button_cmd);

*/
void EndEffector::readPreferences() {
   roller_in_cmd = frc::Preferences::GetDouble("EndEffector/roller_in_cmd", roller_in_cmd);
   roller_out_cmd = frc::Preferences::GetDouble("EndEffector/roller_out_cmd", roller_out_cmd);
   roller_off_cmd = frc::Preferences::GetDouble("EndEffector/roller_off_cmd", roller_off_cmd);
}

/* Take care of any initialization that needs to be done after all controls have been created. */
void EndEffector::RobotInit() {
    if (m_end_roller) {
        m_end_roller -> SetControlMode (RSpeedController::DUTY_CYCLE);
        m_end_roller ->SetBrakeMode(true);
        setEndRollerMotorCurrentLimits(m_current_limit);
    }
#ifdef COMPETITION_SHUFFLEBOARD
    frc::Shuffleboard::GetTab("Field").Add("EndEffector", *this).WithSize(2, 2).WithPosition(8, 0);
#else
    frc::Shuffleboard::GetTab("EndEffector").Add("end_effector", *this).WithSize(2, 2).WithPosition(0, 0);
#endif
}

/**********************************************************************
 *
 * Runs on a clock, separate from main class at a period specified in RoboControl.lua
 * 1. Read any sensors
 * 2. Run logic based on sensor and user inputs (coming in through setAnalog/setDigital)
 * 3. Write to effectors (either motors or relays ...)
 *
 **********************************************************************/

void EndEffector::handleOI(void)
{
    // check for button presses to the in or out buttons
    if (m_end_roll_in_btn.GetButtonPressed())
    {
        rollerIn(); // set the motor command to 50%
    }
    if (m_end_roll_in_btn.GetButtonReleased())
    {
        rollerOff(); // set the motor command to 50%
    }

    if(m_end_roll_out_btn.GetButtonPressed())
    {
        rollerOut();
    }
    if(m_end_roll_out_btn.GetButtonReleased())
    {
        rollerOff();   // set motor to off
    }

    // Smart buttons that are aware of robot state

    if(m_slurp_btn.GetButtonPressed() )
    {
        slurpPressed();
    }
    if(m_slurp_btn.GetButtonReleased() )
    {
        slurpReleased();
    }

    if (m_spit_btn.GetButtonPressed())
    {
        spitPressed();
    }
    if (m_spit_btn.GetButtonReleased())
    {
        spitReleased();
    }

    if (m_reset_btn.GetButtonPressed())
    {
        reset();
    }
}

//**********************************************************************
// at end of RobotPeriodic, write to effectors (valve and motors)
//**********************************************************************
void EndEffector::writeEffectors()
{
    static bool once_roller=false;
    // safe write, with advisories
    if(m_end_roller != nullptr)
    {
        m_end_roller->Set(m_roller_cmd);
    }
    else if(once_roller == false)
    {
        once_roller = true;
        Advisory::pinfo("EndEffector::RollerMotor = nullptr");
    }
}

//**************************************************************************
// at start of  RobotPeriodic, read "sensors" (motor current and duty cycle)
//**************************************************************************
void EndEffector::readSensors()
{
    if(m_end_roller != nullptr)
    {
        switch(m_getter_count)
        {
            case 0:
                m_roller_curr = m_end_roller->GetOutputCurrent();
            break;
            case 1:
                m_roller_dc = m_end_roller->GetMotorOutputPercent();
            break;
            case 2:
                m_roller_vel = m_end_roller->GetSpeed();
            break;
            default:
                break;
        }
        m_getter_count++;
        if(m_getter_count >= c_getter_max)
        {
            m_getter_count = 0;
        }
    }
}

void EndEffector::rollerIn()
{
    m_roller_cmd = roller_in_cmd;
}

void EndEffector::rollerOut() {
    m_roller_cmd = roller_out_cmd;
}

void EndEffector::rollerOff() {
    m_roller_cmd = roller_off_cmd;
}

void EndEffector::slurpPressed() {
    if(ConeCubeControl::isCone()) {
        setEndRollerMotorCurrentLimits(m_initial_current_limit);
        m_reduce_current = true;
        m_reduce_current_time = getPhaseElapsedTime() + 1.0;
        rollerIn();
    } else {
        rollerOut();
    }
}
void EndEffector::slurpReleased() {
    setEndRollerMotorCurrentLimits(m_current_limit);
    rollerOff();
}

void EndEffector::spitPressed() {
    if (ConeCubeControl::isCone()) {
        setEndRollerMotorCurrentLimits(m_initial_current_limit);
        rollerOut();
    } else {
        rollerIn();
    }
}

void EndEffector::spitReleased() {
    rollerOff();
    setEndRollerMotorCurrentLimits(m_current_limit);
}

void EndEffector::reset() {
    rollerOff();
}

double EndEffector::getRollerVel() {
    return m_roller_vel;
}

void EndEffector::setEndRollerMotorCurrentLimits(double limit)
{
    //m_end_roller->SetCurrentLimit(limit, limit, 1.0);

    if(m_end_roller->GetSpeedControllerType() == RSpeedController::ControllerType::TALON_FX)
    {
        RSpeedControllerTalonFXCan *fx = dynamic_cast<RSpeedControllerTalonFXCan*>(m_end_roller);
        fx->SetStatorCurrentLimit(limit, limit, 1.0);
        //fx->SetStatorCurrentLimitEnabled(true);
    }

    //m_end_roller->SetCurrentLimitEnabled(true);
}

void EndEffector::initCurrentLimits(double current_limit, double initial_current_limit) {
    m_current_limit = current_limit;
    m_initial_current_limit = initial_current_limit;
}

void EndEffector::RobotPeriodic(void) {
    readSensors();

    if(!IsAutonomous())
    {
      handleOI();
    }

    if ( m_reduce_current && getPhaseElapsedTime() > m_reduce_current_time) {
        m_reduce_current = false;
        setEndRollerMotorCurrentLimits(m_current_limit);
    }

    writeEffectors();
}

/* Prepare for Autonomous Operations */
void EndEffector::AutonomousInit(void)
{
    m_roller_cmd = roller_off_cmd;
}

/* Prepare for Teleop Operations */
void EndEffector::TeleopInit(void)
{
    m_roller_cmd = roller_off_cmd;
}

/* addLogVars is used to initialize the variables to log

   Use the call addLogVar(std::string, variable).  The variable can be a floating point (double or float), integer
   or boolean.

   addLogVar("value", value);
*/
void EndEffector::addLogVars() {
   addLogVar ("roller_cmd", m_roller_cmd);
   addLogVar ("roller_dc", m_roller_dc);
   addLogVar ("roller_curr", m_roller_curr);
   addLogVar ("roller_vel", m_roller_vel);
}

/* InitSendable is used to initialize the variables to send to the Shuffleboard

   AddBooleanProperty and AddDoubleProperty are the 2 most common methods to use to add varaibles to the shuffleboard.

   void AddBooleanProperty(std::string_view key, std::function< bool()> getter, std::function< void(bool)> setter);
   void AddDoubleProperty(std::string_view key, std::function< double()> getter, std::function< void(double)> setter);

   Functions must be used to access variables.  These can be getters and setters to variables. If a getter/setter does
   not exist, then a c++ lambda function can be used.  For example, if we want to display the variable "value" that has
   no getter and we don't want to allow the value to be set from the shuffleboard.

   builder.AddDoubleProperty("value", [this]{return value;}, nullptr);
*/
void EndEffector::InitSendable(wpi::SendableBuilder &builder) {
    builder.AddDoubleProperty ("01. roller_cmd", [this]{ return m_roller_cmd;}, nullptr);
    builder.AddDoubleProperty ("02. roller_dc", [this]{ return m_roller_dc;}, nullptr);
    builder.AddDoubleProperty ("03. roller_current", [this]{ return m_roller_curr;}, nullptr);
    builder.AddDoubleProperty ("04. roller_vel", [this]{ return m_roller_vel;}, nullptr);
}

/* luaRegister is where to tell Lua which functions to include in RobotControl.lua and autons

   luaRegister has a chained list of calls that define the Subsystem and adds function definitions to Lua.  The line
   that starts "luabridge::getGlobalNamespace" through ".endNamespace()" is technically one line of code.
   To add functions use .addFunction("lua_name",&EndEffector::method) after the "addConstructor" line in the chain.
 */
void luaRegisterEndEffector() {
    Advisory::pinfo("registering EndEffector with lua");
    lua_State * L = getLuaState();
    luabridge::getGlobalNamespace(L)
     .beginNamespace("robonauts")
      .deriveClass<EndEffector, RSubsystem>("end_effector")
       .addConstructor<void (*)(std::string)>()
       .addFunction("rollerIn",&EndEffector::rollerIn)
       .addFunction("rollerOut",&EndEffector::rollerOut)
       .addFunction("rollerOff",&EndEffector::rollerOff)
       .addFunction("slurpPressed",&EndEffector::slurpPressed)
       .addFunction("slurpReleased",&EndEffector::slurpReleased)
       .addFunction("spitPressed",&EndEffector::spitPressed)
       .addFunction("spitReleased",&EndEffector::spitReleased)
       .addFunction("initCurrentLimits",&EndEffector::initCurrentLimits)
       .addFunction("getRollerVel",&EndEffector::getRollerVel)
      .endClass()
     .endNamespace();
}

