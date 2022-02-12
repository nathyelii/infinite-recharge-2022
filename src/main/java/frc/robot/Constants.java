// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);


    public static final class ClimberConstants {

        public static final int climberChannel = 1; 
        public static final double climbSpeed = .7;
        public static final double climbSpeedStop=0;
    }

    public static final class IntakeConstants{ 
        public static final int intakeCANBUSNUMBER= 4; 
        public static final double intakeSpeed = .9; 
        public static final double intakeSpeedStop= 0; 
    }

   public static final class ConveyorConstants{ 
       public static final int CoveyorCANBUSNUMBER= 5; 
       public static final double conveyorSpeed = .5; 
       public static final double conveyorSpeedStop= 0; 
   }

    public static final class DriveConstants {

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 3.5;

        public static final double kTrackwidthMeters = .5334;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final int RIGHTSONCANBUSNUMBER = 3;
        public static final int LEFTSONCANBUSNUMBER = 2;
        public static final int RIGHTFATHERCANBUSNUMBER = 0;
        public static final int LEFTFATHERCANBUSNUMBER = 1;

        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1397;

        public static final double METERSPERPULSE = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static double linearize(final double x1, final double y1, final double x2, final double y2,
                final double input) {
            final Double m = (y2 - y1) / (x2 - x1);
            final Double b = y1 - (-m * x1);
            return m * input + b;

        }
    }
}
