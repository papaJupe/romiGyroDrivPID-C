// VSpace\romiGyroDrivPID - C            TurnToAngle.j auto PIDCommand

//  uses onboard gyro + PID control instead of distance wheel has turned

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {
  private final DriveSubsys m_drive;
  
  /* @param targetAngleDegrees The angle to turn to
   * @param drive - the drive subsystem to use  */
  public TurnToAngle(double targetAngleDegree, DriveSubsys drive) {
    super(
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
              DriveConstants.kTurnD),
        // Close loop using current heading
        drive::getGyroAngleZ,
        // Set reference to target
        targetAngleDegree,
        // Pipe output to turn robot; fine tuning highly dependent on
        // output multiplier along w/ k_'s
        output -> drive.arcaDriv(0, output * 0.5),
        // Require the drive
        drive);  // end super

        m_drive = drive;        
        // Set the controller to be continuous (it's an angle controller)
    getController().enableContinuousInput(-180, 180); // does mod% calcul?
    //  the delta tolerance ensures the robot is stationary at the
    // setpoint before it's considered to have reached the reference
    getController().setTolerance
         (DriveConstants.kTurnTolerDeg, DriveConstants.kTurnTolerVeloc);
  }   // end constructor
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcaDriv(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
   // m_drive.resetGyro(); // add if not in auto sequen already
  }
  
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}   // end class 






 
