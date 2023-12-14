// VSpace\romigyroDrivPID - C    ArcadeDrive cmd, default for teleOp

// straight drive stabilization in v. C done by PID controller made
//  in RC by button 6, R bumper press, bypassing this cmd

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsys;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.Supplier;
// import edu.wpi.first.wpilibj2.command.PrintCommand;

public class ArcadeDrive extends CommandBase {
  private final DriveSubsys mDrive; // this is just a local
  // variable for this class, was confusing to use same exact name
  // as RC's instance.

  // CONSTRUCTOR uses simpler syntax than obscure lambda
  public ArcadeDrive(DriveSubsys drivetrain) {
    mDrive = drivetrain;
    addRequirements(drivetrain);
  } // end constructor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.arcaDriv(-m_controller.getLeftY() * 0.6,
               m_controller.getRightX() * 0.5);
  } // end execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // default does not finish when interrupted
  }
} // end AD cmd class


