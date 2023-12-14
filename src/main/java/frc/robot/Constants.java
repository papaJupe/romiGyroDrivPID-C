// romiGyroDrivPID - C                             CONSTANT.J

package frc.robot;

/*
 * you can statically import this class (or one of its inner classes),     
 * wherever the constants are needed, to reduce verbosity.
 * most example values unused by Romi
 */
public final class Constants {
    public static final class DriveConstants {
    //   public static final int kLeftMotor1Port = 0;
    //   public static final int kLeftMotor2Port = 1;
    //   public static final int kRightMotor1Port = 2;
    //   public static final int kRightMotor2Port = 3;
  
    //   public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    //   public static final int[] kRightEncoderPorts = new int[] {2, 3};
    //   public static final boolean kLeftEncoderReversed = false;
    //   public static final boolean kRightEncoderReversed = true;
  
    //   public static final int kEncoderCPR = 1024;
    //   public static final double kWheelDiameterInches = 6;
    //   public static final double kEncoderDistancePerPulse =
    //    // Assumes the encoders are directly mounted on the wheel shafts
    //       (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
  
    //   public static final boolean kGyroReversed = false;
  
      public static final double kStabilizP = 0.015;
      public static final double kStabilizI = 0.0;
      public static final double kStabilizD = 0.0;
  
      public static final double kTurnP = 0.002;
      public static final double kTurnI = 0.001;
      public static final double kTurnD = 0.0;
  
      public static final double kMaxTurnVeloc = 30;
      public static final double kMaxTurnAcceler = 30;
  
      public static final double kTurnTolerDeg = 2;
      public static final double kTurnRateTolerVeloc = 10; 
      // deg per second
    }  // end drive constant
  
    // public static final class OIConstants {
    // public static final int kDriverControllerPort = 0;
    }
  
  