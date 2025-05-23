// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coral.CoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RemoveAlgae extends SequentialCommandGroup {
    /** Creates a new removeAlgae. */
    public RemoveAlgae(CoralSubsystem coralSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(coralSubsystem.setArmRotationCommand(6), coralSubsystem.setArmRotationCommand(13));
    }
}
