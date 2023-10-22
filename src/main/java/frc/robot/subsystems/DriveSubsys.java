// VSpace\romiGyroDrivPID - C              DriveSubsystem. j

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;

public class DriveSubsys extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.7; // 70 mm
  private double maxFactor = 1.0; // speed multiplier w/ button 5

  // The Romi has the left and right motors set to PWM channels 0 and 1
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi onboard encoders are hardcoded to DIO pins 4/5 and 6/7
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Use DifferentialDrive: arcadeDrive method, deadband 0.02, squares inputs
  private final DifferentialDrive m_diffDrive = new DifferentialDrive
       (m_leftMotor, m_rightMotor);

  // select normal aD() or subsys's arcaGyve()<-- set by button in auto &
  // teleoPeriod, got by AD cmd in some versions, not this one
 //  public boolean gyroMode = false;

  // instance the RomiGyro
  public final RomiGyro m_gyro = new RomiGyro();

  // private final BuiltInAccelerometer m_accelerometer =
  // newBuiltInAccelerometer();

  /** Constructs a new Drivetrain subsyst */
  public DriveSubsys() {
    // We need to invert one side of the drivetrain so that + voltage
    // drives both sides forward. Depending on how your robot's
    // gearbox is constructed, you might want to invert the left side.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    resetEncoders();
    resetGyro();
  } // end constructor

  // public void setGyroMode(boolean maybe) {
  //   gyroMode = maybe; }
  // public boolean getGyroMode() {
  //   return gyroMode; }

  public void setMaxOutput(double d) {
    maxFactor = d;  }

  // sloppy naming by WPI: top aD() is subsys. method, second is inherited
  // diffDrive method; using same name (orig. base code) confuses the issue
  public void arcaDriv(double xaxisSpeed, double zaxisRotate) {
    // diffDrive's aD inverts z-rot param so that + goes CCW; I must invert
    // z-rot value here so that + value from Xpad turns bot CW
    m_diffDrive.arcadeDrive(xaxisSpeed * maxFactor, -zaxisRotate, true);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }
  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }
  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }
  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }
  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }
  // Re'zero' the gyro.
  public void resetGyro() {
    m_gyro.reset();
    System.out.println("gyro reset");
  }
  // @return The current X angle of the Romi in degrees
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }
  // @return The current Y angle of the Romi in degrees
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }
  // @return The current Z angle of the Romi in degrees
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }
  // @return The acceleration of the Romi along the X-axis in Gs
  // public double getAccelX() {
  // return m_accelerometer.getX();
  // }
  // public double getAccelY() {
  // return m_accelerometer.getY();
  // }
  // public double getAccelZ() {
  // return m_accelerometer.getZ();
  // }
  @Override
  public void periodic() {
  }
} // end class
