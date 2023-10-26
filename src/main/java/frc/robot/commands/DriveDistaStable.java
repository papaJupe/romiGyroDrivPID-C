// romiGyroDrivPID - C  auto drive straight DriveDistanceStable.j 

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;

public class DriveDistaStable extends PIDCommand {
  // private final double m_speed;
  private final double m_distance;
  private final DriveSubsys m_drive;

  public DriveDistaStable(double speed, double inch, DriveSubsys drive) {
    super( new PIDController(
				DriveConstants.kStabilizP,
				DriveConstants.kStabilizI,
				DriveConstants.kStabilizD),
        // feedback using the turn angle (? calcul rate from angle)
        drive::getGyroAngleZ,
        // Setpoint is 0, v. initialize() below
        0,
        // Pipe the output to the turning cmd; output sign same as stick's?
        output -> drive.arcaDriv(speed, output),
        drive); // end super() // require target subsystem

    // m_speed = speed;
    m_distance = inch;
    m_drive = drive;
    // Use addRequirements() here to declare more subsystem dependency.
    // Configure additional PID options by calling 'getController' here
    // the delta tolerance ensures the robot is stable at the
    // setpoint before it's considered to have reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnTolerDeg,
                     DriveConstants.kTurnTolerVeloc);
  } // end constructor
 
  @Override  // Called when command is initially scheduled.
  public void initialize() {
    m_drive.arcaDriv(0, 0); 
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }
  
  @Override // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
} // end class
