// romiGyroDrivPID - C              TurnToAngle.j auto command
// modified to use onboard gyro instead of distance wheel has turned

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {
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
        // Pipe output to turn robot
        output -> drive.arcaDrivP(0, output),
        // Require the drive
        drive);  // end super

    // Set the controller to be continuous (it's an angle controller)
    getController().enableContinuousInput(-180, 180);
    //  the delta tolerance ensures the robot is stationary at the
    // setpoint before it's considered to reach the reference
    getController()
        .setTolerance(DriveConstants.kTurnTolerDeg, 
               DriveConstants.kTurnRateTolerVeloc);
  }  // end constructor

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}   // end class 






 
