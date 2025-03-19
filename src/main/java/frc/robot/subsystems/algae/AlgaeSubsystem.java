// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SimulationRobotConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
    private SparkFlex intakeMotor;
    private SparkMax pivotMotor;
    private SparkClosedLoopController rotationClosedLoopController;
    private ProfiledPIDController pidController;

    private SparkMaxSim armMotorSim;
    private DCMotor armMotorModel = DCMotor.getNEO(1);
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            armMotorModel,
            SimulationRobotConstants.kAlgaeReduction,
            SingleJointedArmSim.estimateMOI(SimulationRobotConstants.kAlgaeLength, SimulationRobotConstants.kAlgaeMass),
            SimulationRobotConstants.kAlgaeLength,
            SimulationRobotConstants.kAlgaeMinAngleRads,
            SimulationRobotConstants.kAlgaeMaxAngleRads,
            true,
            SimulationRobotConstants.kAlgaeMinAngleRads,
            0.0,
            0.0);

    /** Creates a new AlgaeSubsystem. */
    public AlgaeSubsystem() {
        this.intakeMotor = new SparkFlex(Constants.AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
        this.pivotMotor = new SparkMax(Constants.AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
        this.armMotorSim = new SparkMaxSim(pivotMotor, armMotorModel);
        this.pidController = new ProfiledPIDController(1, 0, 0.0025, new Constraints(2000, 10000));

        SparkMaxConfig rotationConfig = new SparkMaxConfig();

        rotationConfig.inverted(true);
        rotationConfig.idleMode(IdleMode.kBrake);

        SparkFlexConfig intakeConfig = new SparkFlexConfig();

        intakeConfig.inverted(true);

        this.intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.pivotMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.rotationClosedLoopController = pivotMotor.getClosedLoopController();
    }

    public double getPosition() {
        return this.pivotMotor.getEncoder().getPosition();
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public Command setPowerCommand(double power) {
        return this.runOnce(() -> setPower(power));
    }

    public void setRotation(double target) {
        // this.rotationClosedLoopController.setReference(target,
        // ControlType.kMAXMotionPositionControl);
        this.pidController.setGoal(target);

        Logger.recordOutput("Algae/Pivot Target", target);
    }

    public Command setRotationCommand(double rotations) {
        return this.runOnce(() -> setRotation(rotations));
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_armSim.update(0.020);

        // Iterate the elevator and arm SPARK simulations
        armMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(
                        m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kAlgaeReduction),
                RobotController.getBatteryVoltage(),
                0.02);

        Logger.recordOutput(
                "Algae/Pivot Position", armMotorSim.getRelativeEncoderSim().getPosition());
    }

    @Override
    public void periodic() {
        if (RobotController.getUserButton()) {
            pivotMotor.getEncoder().setPosition(0);
        }
        // This method will be called once per scheduler run

        double plantVoltage = (pidController.calculate(getPosition()));

        pivotMotor.setVoltage(plantVoltage);

        Logger.recordOutput("Algae/Plant Voltage", plantVoltage);
        Logger.recordOutput("Algae/Pivot Position", getPosition());
    }
}
