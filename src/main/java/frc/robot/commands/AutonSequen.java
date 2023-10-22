// VSpace\romiGyroDrivPID - C    auto sequence cmd  AutonSequen.j  

package frc.robot.commands; 

import frc.robot.subsystems.DriveSubsys;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonSequen extends SequentialCommandGroup {
  /* Creates an Autonomous sequence using distance & turn. Drives a
   * set distance, turn CW 180, drive back to start, turn CCW 180.
   * 
   * @param drivetrain the subsystem being controlled
   */
  public AutonSequen(DriveSubsys drivetrain) {
    addCommands(
        new DriveDistaStable(0.6, 36, drivetrain),
        new WaitCommand(5.0),
        new TurnToAngle(178, drivetrain),
        new WaitCommand(5.0), // reset helps accuracy +++
        new InstantCommand(() -> drivetrain.resetGyro()),
        new DriveDistaStable(0.6, 36, drivetrain),
        new WaitCommand(5.0),
        new InstantCommand(() -> drivetrain.resetGyro()),
        new TurnToAngle(-178, drivetrain));
  }
}  // end class
