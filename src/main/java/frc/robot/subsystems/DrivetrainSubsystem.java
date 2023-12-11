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
    // the constructors for these are a little long
    private SwerveModuleConfiguration swerveModuleFL = new SwerveModuleConfiguration(
        new SparkMaxSwerve(Mod0.driveMotorID, true), // create the drive motor
        new SparkMaxSwerve(Mod0.angleMotorID, false), // create the turn (angle) motor
        new DIOAbsoluteEncoderSwerve(Mod0.magEncoderID), // TODO: if using CANcoders, replace this with the appropriate CANcoder class
        Mod0.angleOffset, // we keep the offset in constants for this module, we need it here
        DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0, // these two are the position of the modules relative to the middle of the robot
        anglePIDF, drivePIDF, // pid constants for turn and drive, we define these in constants
        MAX_VELOCITY_METERS_PER_SECOND, // top speed, TODO: check if this actually affects top speed
        new SwerveModulePhysicalCharacteristics( // we also need to provide the physical properties of the module
            DRIVE_GEAR_RATIO, ANGLE_GEAR_RATIO, WHEEL_DIAMETER, // these should all have straightforward names, theyre defined in constants
            DRIVE_RAMP_RATE, ANGLE_RAMP_RATE, // time to hit top speed for both drive and turn (for turn it can be really small, it moves at most 180 deg in one motion)
            1, 1 // ticks per encoder rotation on the drive and turn encoders, TODO: double check these...
        ), 
        MODULE_NAMES[0] // name of the module, no idea why we need this tbh
    );

    // the rest of the modules are defined the same way, just with a couple different numbers
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

    // put our swerve modules in an array for later
    private SwerveModuleConfiguration[] swerveModuleConfigurations = {swerveModuleFL, swerveModuleFR, swerveModuleBL, swerveModuleBR};

    // NavX
    private SwerveIMU navx = new NavXSwerve(I2C.Port.kMXP, (byte) 200);

    // swerve drive hardware config
    // this just needs the stuff we already defined above
    private SwerveDriveConfiguration swerveDriveConfiguration = new SwerveDriveConfiguration(
        swerveModuleConfigurations, navx, MAX_VELOCITY_METERS_PER_SECOND, false // change to true if your NavX is inveted (dont think youll need this)
    );

    // swerve drive controller config
    // swerve heading PIDF is for how fast the whole robot can change heading, not each individual module
    private PIDFConfig swerveHeadingPIDFConfig = new PIDFConfig(0.0, 0.0, 0.0, 0.0, 0.0); // TODO: Tune this!
    private SwerveControllerConfiguration swerveControllerConfiguration = new SwerveControllerConfiguration(swerveDriveConfiguration, swerveHeadingPIDFConfig);

    // final swerve drive
    private SwerveDrive swerveDrive = new SwerveDrive(swerveDriveConfiguration, swerveControllerConfiguration);

    // keep track of how fast we want to go
    private Translation2d linearVelocity = new Translation2d();
    private double angularVelocity = 0.0;

    // flag to lock wheels for scoring
    public boolean lockWheels;

    public DrivetrainSubsystem() {
        lockWheels = false; // unlock wheels on startup
    }

    // reset the gyro to make wherever the robot is pointing the new "0"
    public void zeroGyroscope() {
        swerveDrive.zeroGyro();
    }

    // most used of the directions, so we give it its own function
    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    // default drive command, disengages brakes
    public void drive(Translation2d linearVelocity, double angularVelocity) {
        lockWheels = false;
        drive(linearVelocity, angularVelocity, false);
    }

    // universal drive command, sets direction to go into and brake mode
    public void drive(Translation2d linearVelocity, double angularVelocity, boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
    }

    // stop and engage brakes
    public void stop() { 
        drive(new Translation2d(), 0, true);
    }

    // lock the wheels by putting them in an X formation and braking
    public void lock() {
        stop();
        lockWheels = true;
    }

    // get where we are and where we're pointing right now
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    // get where we are but not where we're pointing right now
    public Translation2d getTranslation() { 
        return getPose().getTranslation(); 
    }

    // tell the robot where it is now
    public void setPoseOdometry(Pose2d pose) { 
        swerveDrive.resetOdometry(pose);
    }

    // update where we think we are based on information from the vision system
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, true, stdDevs);
    }

    @Override
    public void periodic() {
        // dont drive if we're locked the wheels and vice versa
        if (lockWheels) swerveDrive.lockPose();
        else swerveDrive.drive(linearVelocity, angularVelocity, true, false);
        swerveDrive.updateOdometry(); // must be called every 20 ms!

        // debug stuff
        SmartDashboard.putNumber("NavX Yaw", swerveDrive.getYaw().getDegrees());  
        SmartDashboard.putNumber("NavX Pitch", swerveDrive.getPitch().getDegrees()); 
        SmartDashboard.putNumber("NavX Roll", swerveDrive.getRoll().getDegrees()); 

        ChassisSpeeds chassisSpeedsFieldOriented = swerveDrive.getFieldVelocity();
        SmartDashboard.putNumber("Drivetrain Speed X Field Oriented", chassisSpeedsFieldOriented.vxMetersPerSecond); 
        SmartDashboard.putNumber("Drivetrain Speed Y Field Oriented", chassisSpeedsFieldOriented.vyMetersPerSecond);
    } 
}
