// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import swervelib.SwerveDrive;
import swervelib.encoders.DIOAbsoluteEncoderSwerve;
import swervelib.imu.NavXSwerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.SparkMaxSwerve;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Swerve.*;

public class DrivetrainSubsystem extends SubsystemBase {
    // swerve modules
    private SwerveModuleConfiguration swerveModuleFL = new SwerveModuleConfiguration(
        new SparkMaxSwerve(Mod0.driveMotorID, true),
        new SparkMaxSwerve(Mod0.angleMotorID, false), 
        new DIOAbsoluteEncoderSwerve(Mod0.magEncoderID), // TODO: if using CANcoders, replace this with the appropriate CANcoder class
        Mod0.angleOffset, 
        DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0, 
        anglePIDF, drivePIDF, 
        MAX_VELOCITY_METERS_PER_SECOND, 
        new SwerveModulePhysicalCharacteristics(
            DRIVE_GEAR_RATIO, ANGLE_GEAR_RATIO, WHEEL_DIAMETER, 
            DRIVE_RAMP_RATE, ANGLE_RAMP_RATE, 
            1, 1
        ), 
        MODULE_NAMES[0]
    );

    private SwerveModuleConfiguration swerveModuleFR = new SwerveModuleConfiguration(
        new SparkMaxSwerve(Mod1.driveMotorID, true),
        new SparkMaxSwerve(Mod1.angleMotorID, false), 
        new DIOAbsoluteEncoderSwerve(Mod1.magEncoderID),
        Mod1.angleOffset, 
        DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0, 
        anglePIDF, drivePIDF, 
        MAX_VELOCITY_METERS_PER_SECOND, 
        new SwerveModulePhysicalCharacteristics(
            DRIVE_GEAR_RATIO, ANGLE_GEAR_RATIO, WHEEL_DIAMETER, 
            DRIVE_RAMP_RATE, ANGLE_RAMP_RATE, 
            1, 1
        ), 
        MODULE_NAMES[1]
    );

    private SwerveModuleConfiguration swerveModuleBL = new SwerveModuleConfiguration(
        new SparkMaxSwerve(Mod2.driveMotorID, true),
        new SparkMaxSwerve(Mod2.angleMotorID, false), 
        new DIOAbsoluteEncoderSwerve(Mod2.magEncoderID),
        Mod2.angleOffset, 
        -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0, 
        anglePIDF, drivePIDF, 
        MAX_VELOCITY_METERS_PER_SECOND, 
        new SwerveModulePhysicalCharacteristics(
            DRIVE_GEAR_RATIO, ANGLE_GEAR_RATIO, WHEEL_DIAMETER, 
            DRIVE_RAMP_RATE, ANGLE_RAMP_RATE, 
            1, 1
        ), 
        MODULE_NAMES[2]
    );

    private SwerveModuleConfiguration swerveModuleBR = new SwerveModuleConfiguration(
        new SparkMaxSwerve(Mod3.driveMotorID, true),
        new SparkMaxSwerve(Mod3.angleMotorID, false), 
        new DIOAbsoluteEncoderSwerve(Mod3.magEncoderID),
        Mod3.angleOffset, 
        DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0, 
        anglePIDF, drivePIDF, 
        MAX_VELOCITY_METERS_PER_SECOND, 
        new SwerveModulePhysicalCharacteristics(
            DRIVE_GEAR_RATIO, ANGLE_GEAR_RATIO, WHEEL_DIAMETER, 
            DRIVE_RAMP_RATE, ANGLE_RAMP_RATE, 
            1, 1
        ), 
        MODULE_NAMES[3]
    );

    private SwerveModuleConfiguration[] swerveModuleConfigurations = {swerveModuleFL, swerveModuleFR, swerveModuleBL, swerveModuleBR};

    // NavX
    private SwerveIMU navx = new NavXSwerve(I2C.Port.kMXP, (byte) 200);

    // swerve drive hardware config
    private SwerveDriveConfiguration swerveDriveConfiguration = new SwerveDriveConfiguration(
        swerveModuleConfigurations, navx, MAX_VELOCITY_METERS_PER_SECOND, false
    );

    // swerve drive controller config
    private PIDFConfig swerveHeadingPIDFConfig = new PIDFConfig(0.0, 0.0, 0.0, 0.0, 0.0); // TODO: Tune this!
    private SwerveControllerConfiguration swerveControllerConfiguration = new SwerveControllerConfiguration(swerveDriveConfiguration, swerveHeadingPIDFConfig);

    // final swerve drive
    private SwerveDrive swerveDrive = new SwerveDrive(swerveDriveConfiguration, swerveControllerConfiguration);

    private Translation2d linearVelocity = new Translation2d();
    private double angularVelocity = 0.0;

    // flag to lock wheels for scoring
    public boolean lockWheels;

    public DrivetrainSubsystem() {
        lockWheels = false;
    }

    public void zeroGyroscope() {
        swerveDrive.zeroGyro();
    }

    // most used of the directions, so we give it its own function
    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    // default drive command, disengages brakes
    public void drive(Translation2d linearVelocity, double angularVelocity) {
        drive(linearVelocity, angularVelocity, false);
    }

    public void drive(Translation2d linearVelocity, double angularVelocity, boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
    }

    // stop and engage brakes
    public void stop() { 
        drive(new Translation2d(), 0, true);
    }

    public void lock() {
        stop();
        lockWheels = true;
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Translation2d getTranslation() { 
        return getPose().getTranslation(); 
    }

    public void setPoseOdometry(Pose2d pose) { 
        swerveDrive.resetOdometry(pose);
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, true, stdDevs);
    }

    @Override
    public void periodic() {
        if (lockWheels) swerveDrive.lockPose();
        else swerveDrive.drive(linearVelocity, angularVelocity, true, false);
        swerveDrive.updateOdometry();

        SmartDashboard.putNumber("NavX Yaw", swerveDrive.getYaw().getDegrees());  
        SmartDashboard.putNumber("NavX Pitch", swerveDrive.getPitch().getDegrees()); 
        SmartDashboard.putNumber("NavX Roll", swerveDrive.getRoll().getDegrees()); 

        ChassisSpeeds chassisSpeedsFieldOriented = swerveDrive.getFieldVelocity();
        SmartDashboard.putNumber("Drivetrain Speed X Field Oriented", chassisSpeedsFieldOriented.vxMetersPerSecond); 
        SmartDashboard.putNumber("Drivetrain Speed Y Field Oriented", chassisSpeedsFieldOriented.vyMetersPerSecond);
    } 
}
