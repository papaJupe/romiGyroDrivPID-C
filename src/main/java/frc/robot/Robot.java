// romiGyroDrivPID - C                             Robot.j

// use WPI PIDcontrol lib on romi w/ PWM motor, cmd/subsys framewk

// For live vision, attach camera to any pi port, its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg stream,
// and (when Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

 /** The VM is configured to automatically run this class, and call the
 * functions corresponding to each mode, as described in TimedRobot docs.
 *      -- very little here specific to any one robot.   */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;

  /** This function is run when the robot is first started and runs
   * periodics. Initialization of this robot's specifics done in RC    
   */
  @Override
  public void robotInit() {
    // Instantiate RobotContainer --> RC declares, instances, configs the
    // robot specific components and their function (methods).
    m_robotContainer = new RobotContainer();
  }

  // This function is called every robot packet, no matter the mode. Does
  // things that you want run during all modes, like diagnostics.
  // This runs after the mode specific periodic functions, but before
  // LiveWindow and SmartDashboard integrated updating.
  @Override
  public void robotPeriodic() {
    // Calls the Scheduler <--  responsible for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodics.
    // Must be here for anything in the Command-based framework to work.
    SmartDashboard.putNumber("Z axis Rot",
        m_robotContainer.m_drivetrain.m_gyro.getAngleZ());
    CommandScheduler.getInstance().run();
  } // end robotPeriodic

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
    }

  @Override // incessant (-/+) drift, so to reset manually in disabled
  			//  mode you can press button 1 [xbox button A]
  public void disabledPeriodic() {
    if (RobotContainer.m_controller.getRawButton(1))
      m_robotContainer.m_drivetrain.resetGyro();
  }


  // autoInit runs the autonomous command set in {@link RobotContainer}
  @Override
  public void autonomousInit() {
    // RC got selected routine from the SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // all auto methods reset encoder & gyro @ init;
    // schedule the selected autonomous command (if not empty variable)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();    }
  } // end autoInit

  // This function is called periodically during autonomous.
  @Override
  public void autonomousPeriodic() {
    // in v. C (this) straight drive uses PIDcontroller; ? need here NO
    // if (RobotContainer.m_controller.getRawButton(6))
    //   m_robotContainer.m_drivetrain.setGyroMode(true);
    // else
    //   m_robotContainer.m_drivetrain.setGyroMode(false);
  }
  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped. If you want 
    // auto cmd to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_drivetrain.resetEncoders();
    m_robotContainer.m_drivetrain.resetGyro();
  } // end teleOpInit
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { // rt bumper button hold activates
    // gyro mode to drive straight in teleop
    // ? possible to do this in RC with JoystkButt(cmd) yes, no need here
    // if (RobotContainer.m_controller.getRawButton(6))
    // m_robotContainer.m_drivetrain.setGyroMode(true);
    // else
    // m_robotContainer.m_drivetrain.setGyroMode(false);
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
} // end class

