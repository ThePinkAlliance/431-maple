package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.elevator.MoveElevator;
import org.littletonrobotics.junction.Logger;

public class CoralSubsystem extends SubsystemBase {
    /** Subsystem-wide setpoints */
    int level = 1;

    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    private ProfiledPIDController elevatorPIDController;
    private ProfiledPIDController armPIDController;
    private LinearFilter elevatorFilter;
    private double lastEpoch;
    private int epochCount;

    // Initialize arm SPARK. We will use MAXMotion position control for the arm, so
    // we also need to
    // initialize the closed loop controller and encoder.
    private SparkMax armMotor = new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
    public SparkClosedLoopController armController = armMotor.getClosedLoopController();
    public RelativeEncoder armEncoder = armMotor.getEncoder();

    // Initialize elevator SPARK. We will use MAXMotion position control for the
    // elevator, so we also
    // need to initialize the closed loop controller and encoder.
    private SparkFlex elevatorMotor = new SparkFlex(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    // Initialize intake SPARK. We will use open loop control for this so we don't
    // need a closed loop
    // controller like above.
    private SparkMax intakeMotor = new SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

    // Member variables for subsystem state management
    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    public double armCurrentTarget = 0;
    private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;

    // Simulation setup and variables
    private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
    private SparkFlexSim elevatorMotorSim;
    private SparkLimitSwitchSim elevatorLimitSwitchSim;
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            elevatorMotorModel,
            SimulationRobotConstants.kElevatorGearing,
            SimulationRobotConstants.kCarriageMass,
            SimulationRobotConstants.kElevatorDrumRadius,
            SimulationRobotConstants.kMinElevatorHeightMeters,
            SimulationRobotConstants.kMaxElevatorHeightMeters,
            true,
            SimulationRobotConstants.kMinElevatorHeightMeters,
            0.0,
            0.0);

    private DCMotor armMotorModel = DCMotor.getNEO(1);
    private SparkMaxSim armMotorSim;
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            armMotorModel,
            SimulationRobotConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
            SimulationRobotConstants.kArmLength,
            SimulationRobotConstants.kMinAngleRads,
            SimulationRobotConstants.kMaxAngleRads,
            true,
            SimulationRobotConstants.kMinAngleRads,
            0.0,
            0.0);

    // Mechanism2d setup for subsystem
    private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d(
            "Elevator",
            SimulationRobotConstants.kMinElevatorHeightMeters * SimulationRobotConstants.kPixelsPerMeter,
            90));
    private final MechanismLigament2d m_armMech2d = m_elevatorMech2d.append(new MechanismLigament2d(
            "Arm",
            SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
            180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));

    public CoralSubsystem() {
        /*
         * Apply the appropriate configurations to the SPARKs.
         *
         * kResetSafeParameters is used to get the SPARK to a known state. This
         * is useful in case the SPARK is replaced.
         *
         * kPersistParameters is used to ensure the configuration is not lost when
         * the SPARK loses power. This is useful for power cycles that may occur
         * mid-operation.
         */
        armMotor.configure(
                Configs.CoralSubsystem.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor.configure(
                Configs.CoralSubsystem.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(
                Configs.CoralSubsystem.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Display mechanism2d
        SmartDashboard.putData("Coral Subsystem", m_mech2d);

        // Zero arm and elevator encoders on initialization
        armEncoder.setPosition(0);
        elevatorEncoder.setPosition(0);

        // Initialize simulation values
        elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
        elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
        armMotorSim = new SparkMaxSim(armMotor, armMotorModel);

        this.elevatorPIDController = new ProfiledPIDController(1, 0.1, 0, new Constraints(15, 225));
        this.elevatorPIDController.setTolerance(1);

        this.armPIDController = new ProfiledPIDController(1, 0, 0, new Constraints(24, 576));
        this.elevatorPIDController.setTolerance(0.15);

        this.elevatorFilter = LinearFilter.movingAverage(5);
        this.lastEpoch = Timer.getFPGATimestamp();
        this.epochCount = 0;
    }

    /**
     * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion position control which
     * will allow for a smooth acceleration and deceleration to the mechanisms' setpoints.
     */
    public void moveToSetpoint() {
        // armController.setReference(armCurrentTarget,wq
        // ControlType.kMAXMotionPositionControl);
        armPIDController.setGoal(armCurrentTarget);
        elevatorPIDController.setGoal(elevatorCurrentTarget);

        // Added the elevator feedforward this needs to be characerised first!
        // elevatorClosedLoopController.setReference(
        // elevatorCurrentTarget,
        // ControlType.kMAXMotionPositionControl,
        // ClosedLoopSlot.kSlot0,
        // 0,
        // ArbFFUnits.kVoltage);
    }

    /** Zero the elevator encoder when the limit switch is pressed. */
    private void zeroElevatorOnLimitSwitch() {
        if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
            // Zero the encoder only when the limit switch is switches from "unpressed" to
            // "pressed" to
            // prevent constant zeroing while pressed
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
        } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
            wasResetByLimit = false;
        }
    }

    /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
    private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            // Zero the encoders only when button switches from "unpressed" to "pressed" to
            // prevent
            // constant zeroing while pressed
            wasResetByButton = true;
            armEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    /** Set the intake motor power in the range of [-1, 1]. */
    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    /**
     * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined positions for the
     * given setpoint.
     */
    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(() -> {
            switch (setpoint) {
                case kFeederStation:
                    armCurrentTarget = ArmSetpoints.kFeederStation;
                    elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
                    level = 0;
                    break;
                case kLevel1:
                    armCurrentTarget = ArmSetpoints.kLevel1;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
                    level = 1;
                    break;
                case kLevel2:
                    armCurrentTarget = ArmSetpoints.kLevel2;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
                    level = 2;
                    break;
                case kLevel3:
                    armCurrentTarget = ArmSetpoints.kLevel3;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
                    level = 3;
                    break;
                case kLevel4:
                    armCurrentTarget = ArmSetpoints.kLevel4;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
                    level = 4;
                    break;
            }
        });
    }

    public void setElevatorTarget(double target) {
        this.elevatorCurrentTarget = target;
    }

    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
    }

    public Command setElevatorCommand(double target) {
        return new MoveElevator(this, target);
    }

    public Command setArmRotationCommand(double target) {
        return new MoveArm(this, target);
    }

    public Command movePointUpCommand() {
        return this.runOnce(() -> {
            switch (level) {
                case 0:
                    setSetpointCommand(Setpoint.kLevel1);
                    break;
                case 1:
                    setSetpointCommand(Setpoint.kLevel2);
                    break;
                case 2:
                    setSetpointCommand(Setpoint.kLevel3);
                    break;
                case 3:
                    setSetpointCommand(Setpoint.kLevel4);
                    break;
                case 4:
                    setSetpointCommand(Setpoint.kLevel4);
            }
        });
    }

    public Command movePointDownCommand() {
        return this.runOnce(() -> {
            switch (level) {
                case 0:
                    setSetpointCommand(Setpoint.kFeederStation);
                    break;
                case 1:
                    setSetpointCommand(Setpoint.kFeederStation);
                    break;
                case 2:
                    setSetpointCommand(Setpoint.kLevel1);
                    break;
                case 3:
                    setSetpointCommand(Setpoint.kLevel2);
                    break;
                case 4:
                    setSetpointCommand(Setpoint.kLevel3);
            }
        });
    }

    /**
     * Command to run the intake motor. When the command is interrupted, e.g. the button is released, the motor will
     * stop.
     */
    public Command runIntakeCommand() {
        return this.startEnd(() -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
    }

    /**
     * Command to run the intake motor. When the command is interrupted, e.g. the button is released, the motor will
     * stop.
     */
    public Command stopIntakeCommand() {
        return this.startEnd(() -> this.setIntakePower(0), () -> this.setIntakePower(0.0));
    }

    /**
     * Command to reverses the intake motor. When the command is interrupted, e.g. the button is released, the motor
     * will stop.
     */
    public Command reverseIntakeCommand() {
        return this.startEnd(() -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
    }

    /**
     * Command to reverses the intake motor. When the command is interrupted, e.g. the button is released, the motor
     * will stop.
     */
    public Command reverseIntakeCommand(double power) {
        return this.startEnd(() -> this.setIntakePower(power), () -> this.setIntakePower(0.0));
    }

    public void stopElevator() {
        this.elevatorPIDController.setGoal(elevatorEncoder.getPosition());
    }

    @Override
    public void periodic() {
        double epoch = Timer.getFPGATimestamp() - lastEpoch;

        moveToSetpoint();
        // zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        // Display subsystem values
        Logger.recordOutput("Coral/Arm/Target Position", armCurrentTarget);
        Logger.recordOutput("Coral/Arm/Actual Position", armEncoder.getPosition());
        Logger.recordOutput("Coral/Elevator/Target Position", elevatorCurrentTarget);
        Logger.recordOutput("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
        Logger.recordOutput("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
        Logger.recordOutput("Coral/Elevator/Voltage Applied", elevatorMotor.getBusVoltage());

        double elevatorVoltage = MathUtil.clamp(this.elevatorPIDController.calculate(getElevatorPosition()), -12, 12);
        double armVoltage = MathUtil.clamp(this.armPIDController.calculate(armEncoder.getPosition()), -12, 12);

        Logger.recordOutput("Coral/Elevator/Output Voltage", elevatorVoltage);
        Logger.recordOutput("Coral/Arm/Output Voltage", armVoltage);
        Logger.recordOutput("Coral/Elevator/PID Velocity Error", elevatorPIDController.getVelocityError());
        Logger.recordOutput("Coral/Arm/PID Velocity Error", armPIDController.getVelocityError());

        if (level == 1 && elevatorPIDController.getVelocityError() == 0) {
            this.elevatorEncoder.setPosition(0);
        }

        this.elevatorMotor.setVoltage(elevatorVoltage);
        this.armMotor.setVoltage(armVoltage);

        // Update mechanism2d
        m_elevatorMech2d.setLength(
                SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
                        + SimulationRobotConstants.kPixelsPerMeter
                                * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
        m_armMech2d.setAngle(
                180
                        - ( // mirror the angles so they display in the correct direction
                        Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                                + Units.rotationsToDegrees(
                                        armEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
                        - 90 // subtract 90 degrees to account for the elevator
                );

        lastEpoch = Timer.getFPGATimestamp();

        if (lastEpoch < 4) {
            lastEpoch++;
        } else {
            lastEpoch = 0;
        }
    }

    /** Get the current drawn by each simulation physics model */
    public double getSimulationCurrentDraw() {
        return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Update sim limit switch
        elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);
        m_armSim.update(0.020);

        // Iterate the elevator and arm SPARK simulations
        elevatorMotorSim.iterate(
                ((m_elevatorSim.getVelocityMetersPerSecond()
                                        / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                                * SimulationRobotConstants.kElevatorGearing)
                        * 60.0,
                RobotController.getBatteryVoltage(),
                0.02);
        armMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(
                        m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
                RobotController.getBatteryVoltage(),
                0.02);

        Pose2d pose = new Pose2d(new Translation2d(0, this.m_elevatorSim.getPositionMeters()), new Rotation2d());

        Logger.recordOutput("Coral/ZeroedCompoenentPoses", new Pose2d[] {pose});

        // SimBattery is updated in Robot.java
    }
}
