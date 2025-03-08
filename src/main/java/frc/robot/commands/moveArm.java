// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveArm extends Command {
    CoralSubsystem coralSubsystem;
    double target;
    /** Creates a new testCommand. */
    public moveArm(CoralSubsystem coralSubsystem, double target) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralSubsystem = coralSubsystem;
        this.target = target;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralSubsystem.armCurrentTarget = target;
        coralSubsystem.moveToSetpoint();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(coralSubsystem.armCurrentTarget - coralSubsystem.armEncoder.getPosition()) < 1;
    }
}
