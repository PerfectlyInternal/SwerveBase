// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.PIDConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.util.OrbitPID;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

      /*
     * Swerve Constants (newly added ones)
     */
    public static final class Swerve {
        /* Module Specific Constants */
        public static final String[] MODULE_NAMES = { "Front Left", "Front Right", "Back Left", "Back Right" }; // module
                                                                                                                // #0,
        // #1, #2, #3

        public static int PEAK_CURRENT_LIMIT = 50;
        public static int CONTINUOUS_CURRENT_LIMIT = 40;
        public static boolean ANGLE_INVERT = true;
        public static boolean DRIVE_INVERT = false;
        public static boolean isGyroInverted = true;
        public static IdleMode IDLE_MODE = IdleMode.kBrake;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.61;
        public static final double WHEEL_BASE = 0.61;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1
        public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0; // 150/7:1
        public static final double DRIVE_CONVERSION_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;
        public static final double MAX_SPEED = Swerve.AutoConstants.maxSpeed;
        public static final double DRIVE_RAMP_RATE = 1.0; // TODO: tune
        public static final double ANGLE_RAMP_RATE = 0.1; // TODO: tune

        /*
         * Ideally these should be independent but for getting started same pid/ff
         * values should work just fine
         */
        public static final PIDFConfig drivePIDF = new PIDFConfig(0.3, 0.0, 0.0045); // TODO: tune
        public static final PIDFConfig anglePIDF = new PIDFConfig(0.023, 0.000001, 0.0);

        /* Custom PID Controllers */
        public static final OrbitPID robotRotationPID = new OrbitPID(0.1, 0, 0.00005);

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int magEncoderID = 0;
            public static final double angleOffset = 130.0;
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int magEncoderID = 1;
            public static final double angleOffset = 40.3;
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int magEncoderID = 2;
            public static final double angleOffset = 252.2;
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int magEncoderID = 3;
            public static final double angleOffset = 326.85;
        }

        public static final class AutoConstants {
            // PID values to follow paths. NOT *DIRECTLY* FOR MODULE SPEED, try DRIVE_PID
            // and ANGLE_PID first
            public static PIDConstants translation = new PIDConstants(0.13, 0, 0.0045);
            public static PIDConstants rotation = new PIDConstants(0.013, 0.000001, 0);
            public static double maxSpeed = 4 * 0.7; // meters
            public static double maxAcceleration = 2 * 0.9; // m/s^2
        }

    }

  public static final class Drivetrain {

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        5676.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.61; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.61; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final double DRIVE_MOTION_PROFILE_MAX_VELOCITY = 4.000;
    public static final double DRIVE_MOTION_PROFILE_MAX_ACCELERATION = 3.550;
    public static final double ROTATION_MOTION_PROFILE_MAX_VELOCITY = 180.0;
    public static final double ROTATION_MOTION_PROFILE_MAX_ACCELERATION = 180.0;
  }
}
