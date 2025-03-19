// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.algae.MoveAlgaeWithSpeed;
import frc.robot.commands.algae.RemoveAlgae;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.ButtonID;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.Setpoint;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsystem;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final Joystick controller = new Joystick(0);

    private SendableChooser<Alliance> allianceSelect = new SendableChooser<>();

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        coralSubsystem = new CoralSubsystem();
        algaeSubsystem = new AlgaeSubsystem();

        // Registers all the commands that pathplanner will use before autos.
        registerNamedCommands();

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                this.vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                        new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        allianceSelect.addOption("Blue", Alliance.Blue);
        allianceSelect.addOption("Red", Alliance.Red);

        SmartDashboard.putData(allianceSelect);

        // Configure the button bindings
        configureButtonBindings();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("level_3", coralSubsystem.setSetpointCommand(Setpoint.kLevel3));
        NamedCommands.registerCommand("prep_level_3", coralSubsystem.setSetpointCommand(Setpoint.kLevel3));

        NamedCommands.registerCommand("level_2", coralSubsystem.setSetpointCommand(Setpoint.kLevel2));
        NamedCommands.registerCommand("prep_level_2", coralSubsystem.setSetpointCommand(Setpoint.kLevel2));

        NamedCommands.registerCommand("level_1", coralSubsystem.setSetpointCommand(Setpoint.kLevel1));
        NamedCommands.registerCommand("prep_level_1", coralSubsystem.setSetpointCommand(Setpoint.kLevel1));

        NamedCommands.registerCommand("stow", coralSubsystem.setSetpointCommand(Setpoint.kFeederStation));
        NamedCommands.registerCommand(
                "deposit", coralSubsystem.reverseIntakeCommand().withTimeout(0.5));

        NamedCommands.registerCommand("feed_coral", Commands.none());
        NamedCommands.registerCommand("free_algae", new RemoveAlgae(coralSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> controller.getRawAxis(1), () -> controller.getRawAxis(0), () -> controller.getRawAxis(4)));

        new POVButton(controller, 0).onTrue(coralSubsystem.setSetpointCommand(Setpoint.kLevel4));
        new POVButton(controller, 270).onTrue(coralSubsystem.setSetpointCommand(Setpoint.kLevel3));
        new POVButton(controller, 180).onTrue(coralSubsystem.setSetpointCommand(Setpoint.kLevel2));

        new Trigger(() -> controller.getRawAxis(ButtonID.LS) >= 0.05)
                .onTrue(coralSubsystem.setSetpointCommand(Setpoint.kFeederStation));

        new Trigger(() -> controller.getRawAxis(ButtonID.RS) >= 0.05)
                .onTrue(coralSubsystem.setSetpointCommand(Setpoint.kLevel1));

        // new JoystickButton(controller, ButtonID.LB)
        // .onTrue(new MoveElevator(coralSubsystem,
        // Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel4)
        // .andThen(new MoveArm(coralSubsystem,
        // Constants.CoralSubsystemConstants.ArmSetpoints.kLevel4))
        // .andThen(coralSubsystem.reverseIntakeCommand(-0.22).withTimeout(3))
        // .andThen(coralSubsystem.setSetpointCommand(Setpoint.kLevel1)));

        new JoystickButton(controller, ButtonID.A).whileTrue(coralSubsystem.runIntakeCommand());
        new JoystickButton(controller, ButtonID.B).whileTrue(coralSubsystem.reverseIntakeCommand());
        new JoystickButton(controller, ButtonID.Y).whileTrue(coralSubsystem.reverseIntakeCommand(-0.25));
        new JoystickButton(controller, ButtonID.RB)
                .whileTrue(new MoveAlgaeWithSpeed(algaeSubsystem, 5, 0.5))
                .onFalse(algaeSubsystem
                        .setRotationCommand(0)
                        .withTimeout(0.5)
                        .andThen(algaeSubsystem.setPowerCommand(0)));

        new JoystickButton(controller, ButtonID.LB)
                .whileTrue(new MoveAlgaeWithSpeed(algaeSubsystem, 0, -0.5))
                .onFalse(algaeSubsystem
                        .setRotationCommand(0)
                        .withTimeout(0.5)
                        .andThen(algaeSubsystem.setPowerCommand(0)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
