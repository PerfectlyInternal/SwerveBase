// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // create our drivetrain and drive command
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final DefaultDriveCommand defaultDriveCommand = new DefaultDriveCommand(
        drivetrainSubsystem, 
        () -> driverController.getLeftX(), 
        () -> driverController.getLeftY(), 
        () -> driverController.getRightX(),
        null // disables the rotation buttons
    );

    public RobotContainer() {
        // setting this as the default command causes it to run for this subsystem all the time, as we never interrupt it
        drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);
        configureBindings();
    }

    private void configureBindings() {
        // bind some buttons to other controls
        driverController.b().onTrue(new InstantCommand(drivetrainSubsystem::lock)); // lock position by putting wheels in an X formation
        driverController.a().onTrue(new InstantCommand(drivetrainSubsystem::zeroGyroscope));
    }

    public Command getAutonomousCommand() {
        return null; // no auto
    }
}
