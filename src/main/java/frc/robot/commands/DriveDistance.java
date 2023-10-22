// VSpace\romiGyroDrivPID - C  auto cmd drive straight   DriveDistance.j   

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsys;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private final DriveSubsys m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates new DriveDistance command; goes straight fwd/bak at set
   * speed for distance per wheel encoders. No stabiliz unless Rt bumper
   * button held
   * @param speed  The speed [0-1] for the robot to drive
   * @param inches The number of inches the robot to drive
   * @param drive  The drivetrain subsystem for this cmd
   */
  public DriveDistance(double speed, double inches, DriveSubsys drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcaDriv(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs it.
  @Override
  public void execute() {
    // if (m_drive.getGyroMode()) { // if it's been set to true in the subsys
    //   m_drive.arcaGyve(m_speed, 0);
    //   // System.out.println("using gyroMode");
    // } else // normal aD using params from auto distance cmd
    // will button press-->PIDcontrol override this 0 for rot. ??
      m_drive.arcaDriv(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}   // end class
