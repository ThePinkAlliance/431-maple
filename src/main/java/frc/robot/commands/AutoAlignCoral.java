// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCoral extends Command {
    private Drive drive;
    private Vision vision;

    /** Creates a new AutoAlignCoral. */
    public AutoAlignCoral(Drive drive, Vision vision) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.drive = drive;
        this.vision = vision;

        addRequirements(vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Rotation2d xOffset = vision.getTargetX(2);
        Rotation2d yOffset = vision.getTargetY(2);

        double xDiff = (0 - xOffset.getRadians()) * 1;
        double yDiff = (0 - yOffset.getRadians()) * 1;

        // drive.runVelocity(new ChassisSpeeds(xDiff, yDiff, 0));

        Logger.recordOutput("Commands/AutoAlign/diff-x", xDiff);
        Logger.recordOutput("Commands/AutoAlign/diff-y", yDiff);
        Logger.recordOutput("Commands/AutoAlign/offset-x", xOffset.getRadians());
        Logger.recordOutput("Commands/AutoAlign/offset-y", yOffset.getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
