package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final CommandJoystick rotationJoystick;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            CommandJoystick rotationJoystick) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.rotationJoystick = rotationJoystick;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // some math stuff for the rotation buttons
        double rotSpeed = -m_rotationSupplier.getAsDouble(); // equal to the turn stick input by default, gets overridden by the turn buttons later
        double curAngle = m_drivetrainSubsystem.getYaw().getDegrees() % 360.0;
        double curAngle2 = (m_drivetrainSubsystem.getYaw().getDegrees() + 180.0) % 360.0;

        SmartDashboard.putNumber("Default_Drive_Command_Cur_Angle", curAngle);
        // 2 -> 0 (180deg in alternative frame of reference)
        // 5 -> 90
        // 3 -> 180
        // 4 -> 270

        if (this.rotationJoystick != null) { // check if we disabled the rotation buttons
            if (this.rotationJoystick.button(3).getAsBoolean())
                rotSpeed = -Constants.Swerve.robotRotationPID.calculate(180.0, curAngle);
            else if (this.rotationJoystick.button(2).getAsBoolean())
                rotSpeed = -Constants.Swerve.robotRotationPID.calculate(180.0, curAngle2);
            else if (this.rotationJoystick.button(5).getAsBoolean())
                rotSpeed = -Constants.Swerve.robotRotationPID.calculate(90.0, curAngle);
            else if (this.rotationJoystick.button(4).getAsBoolean()) {
                rotSpeed = -Constants.Swerve.robotRotationPID.calculate(270.0,
                    curAngle + (Math.abs(270 - curAngle) > 180 ? 360 : 0));
            }
        }

        // most important bit, updates the drivetrain to drive where we want
        m_drivetrainSubsystem.drive(
            new Translation2d(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble()
            ),
            rotSpeed
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
