// romiGyroDrivPID - C    default cmd  ArcadeDrive cmd

 // v. B gets drive mode from subsys var, calls aD method +/- gyro;
// v. C uses PIDcontrollers for turn (not speed or distance)

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsys;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final DriveSubsys m_drive; // this is just a local pvt
  // variable for this class; orig. example using same exact name
  // as RC's instance was needlessly confusing.

  /* orig.:
   * // private final Supplier<Double> m_xaxisSpeedSupplier;
   * // private final Supplier<Double> m_zaxisRotateSupplier;
   * 
   * Creates a new ArcadeDrive. This command will drive your robot w/
   * the speed supplier lambdas. This command does not terminate.
   *
   * [@param] drivetrain The drivetrain subsystem for this command
   * [@param] xaxisSpeedSupplier Lambda supplier of fwrd/backward speed
   * [@param] zaxisRotateSupplier Lambda supplier of rotational speed  */

  // new CONSTRUCTOR uses simpler syntax than original obscure lambda
  public ArcadeDrive(DriveSubsys drivetrain) {
    m_drive = drivetrain;
    addRequirements(drivetrain);
  } // end constructor
  
  // orig. obscure constructor
  // public ArcadeDrive(
  // Drivetrain drivetrain,
  // Supplier<Double> xaxisSpeedSupplier,
  // Supplier<Double> zaxisRotateSupplier) {
  // m_drivetrain = drivetrain;
  // m_xaxisSpeedSupplier = xaxisSpeedSupplier;
  // m_zaxisRotateSupplier = zaxisRotateSupplier;
  // addRequirements(drivetrain); }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

 // Called every time the scheduler runs while the command is called;
 // PIDcontroller available via button (6) press and by default in
 // TurnToAngle cmd -- I don't need to switch drive method as v. B
   @Override  
   public void execute() {
          m_drive.arcaDriv(-m_controller.getLeftY() * 0.6,
             //m_controller.getRightX()* 0.5);
             m_controller.getRawAxis(0) * 0.6);
      } // end execute
    
  // original execute():
    // m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(),
    //        m_zaxisRotateSupplier.get());
  
    // mode[aD|gyro] var in subsys, set by button in telePeriodic, 
    // if (m_drivetrain.getGyroMode()) { // params kept same
    //   m_drivetrain.arcaGyve(-m_controller.getRawAxis(1) * 0.5,
                // m_controller.getRawAxis(4) * 0.5);
    //   // System.out.println("using gyroMode");
    // } else // normal control: left button speed, rt button turn


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


