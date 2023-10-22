// ROMI GYRO DRIVpid - C                  RobotContainer.j

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonSequen;
import frc.robot.commands.TurnToAngle;
// import frc.robot.commands.TurnToAngleProf;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.subsystems.DriveSubsys;

/**
 * RC is where this robot specifics are defined. Since Command-based is a
 * "declarative" paradigm, very little robot logic should be handled in
 * the {@link Robot} periodic methods other than the scheduler calls.
 * Instead, the specifics of operating the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // instance the two subsystems
  protected final DriveSubsys m_drivetrain = new DriveSubsys();
  private final OnBoardIO m_onboardIO = new OnBoardIO
       (ChannelMode.INPUT, ChannelMode.INPUT);

  // NOTE: I/O pin function config possible in web interface; v. base code

  // instance joystick @ 0 --assumes controller plugged into USB 0
  // numerous get()s in AD cmd require public stick
  public static final XboxController m_controller = new XboxController(0);

  // allows SmartDashboard to pick autonomous routine
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * CONSTRUCT container for robot: its single method, configBB() sets
   * Drivetrain [subsystem's] default Cmd, OperatorInterface (OI) actions,
   * Smart Dashbd Autonomous chooser options.
   * --- i.e. the specifics of this robot
   */
  public RobotContainer() {
    // Configure joystick button bindings et. al.;  not clear why not
    configureButtonBindings();     // just put things in this block ?
  } // end constructor

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {GenericHID} or one of its subclasses
   * edu.wpi.first.wpilibj.Joystick} or {XboxController}, and then passing
   * it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // make Default command ArcadeDrive - runs unless another command
    // is scheduled over it.
    // [orig.]m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
    // now less obscure syntax, using simpler constructor
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain));

    // m_robotDrive.setDefaultCommand(
    // // A split-stick arcade command, with forward/backward controlled
    // // by the left stick, and turning controlled by the right.
    // new RunCommand( () -> m_robotDrive.arcadeDrive(
    // -m_driverController.getLeftY(), -m_driverController.getRightX()),
    // m_robotDrive));

    // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA  // looks like it would repeat but doesn't
        .whileTrue(new PrintCommand("onbord A Press"))
        .whileFalse(new PrintCommand("onbord A Release"));

    // // stick button A resets Gyro (to 0) [instanced in drive subsystem]
    new JoystickButton(m_controller, 1) // does repeat
    .onTrue(new InstantCommand(() -> m_drivetrain.resetGyro()))
    .onTrue(new PrintCommand("Button A Press"));

    // Turn to +90 degrees [with|out a profile] when B button is pressed,
    // with a 5 second timeout
    new JoystickButton(m_controller, 2)
        .onTrue(new TurnToAngle(90, m_drivetrain)
             .withTimeout(5));

    // Turn to -90 degrees when the 'X' button is pressed, 5 second timeout
    new JoystickButton(m_controller, 3)
        .onTrue(new TurnToAngle(-90, m_drivetrain).withTimeout(5));

    // Drive at half speed when the left bumper is held
    new JoystickButton(m_controller, 5)
        .onTrue(new InstantCommand(() -> m_drivetrain.setMaxOutput(0.6)))
        .onFalse(new InstantCommand(() -> m_drivetrain.setMaxOutput(1)));

    // stabilize to drive straight with gyro when rt bumper button is held
    new JoystickButton(m_controller, 6)
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizP,
                    DriveConstants.kStabilizI,
                    DriveConstants.kStabilizD),
                // feedback comes from current angle (-->calcul turn rate?)
                    m_drivetrain::getGyroAngleZ,
                // Setpoint is 0 deg.
                0,
     // nb: uses subsys' aD(), bypassing AD cmd; ? if compatible w/ auto
                // Pipe the output to the turning cmd; L still controls speed
                output -> m_drivetrain.arcaDriv
                      (-m_controller.getLeftY() * 0.6, output),
                // Require the drive subsystem instance
                m_drivetrain));

    // in v. B pressing during teleOp activates gyro stabiliz via variable
    // new JoystickButton(m_controller, 6)
    // .onTrue(new InstantCommand(() -> m_drivetrain.setGyroMode(true)))
    // .onFalse(new InstantCommand(() -> m_drivetrain.setGyroMode(false)));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auton Sequen", new AutonSequen(m_drivetrain));
    m_chooser.addOption("Auto Turn Gyro",
          new TurnToAngle(90, m_drivetrain));
    SmartDashboard.putData(m_chooser);

  } // end configBB()

  // passes selected auto command to the scheduling {@link Robot} class.
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  } // end get.AutoCmd
} // end class
