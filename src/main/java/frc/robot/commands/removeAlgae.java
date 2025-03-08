// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coral.CoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class removeAlgae extends SequentialCommandGroup {
  /** Creates a new removeAlgae. */
  public removeAlgae(CoralSubsystem coralSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      coralSubsystem.setArmRotationCommand(32),
      new WaitCommand(5),
      coralSubsystem.setArmRotationCommand(20),
      new WaitCommand(5),
      coralSubsystem.setArmRotationCommand(40),
      new WaitCommand(5)
    );
  }
}
