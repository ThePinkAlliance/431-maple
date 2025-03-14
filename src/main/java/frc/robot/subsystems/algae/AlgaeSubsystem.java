// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SimulationRobotConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
  private SparkFlex intakeMotor;
  private SparkMax pivotMotor;
  private SparkClosedLoopController rotationClosedLoopController;

  private SparkMaxSim armMotorSim;
  private DCMotor armMotorModel = DCMotor.getNEO(1);
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

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    this.intakeMotor = new SparkFlex(Constants.AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    this.pivotMotor = new SparkMax(Constants.AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
    this.armMotorSim = new SparkMaxSim(pivotMotor, armMotorModel);

    SparkMaxConfig rotationConfig = new SparkMaxConfig();

    rotationConfig.closedLoop.p(0.1).outputRange(-0.25, 0.25);
    rotationConfig.voltageCompensation(12);
    rotationConfig.idleMode(IdleMode.kBrake);

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
    return this.run(() -> setPower(power));
  }

  public void setRotation(double target) {
    this.rotationClosedLoopController.setReference(target, ControlType.kPosition);

    Logger.recordOutput("Algae/Pivot Target", target);
  }

  public Command setRotationCommand(double rotations) {
    return this.run(() -> setRotation(rotations));
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

    Logger.recordOutput("Algae/Pivot Position", armMotorSim.getRelativeEncoderSim().getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    pivotMotor.getEncoder().getPosition() / SimulationRobotConstants.kAlgaeReduction))
            - 90 // subtract 90 degrees to account for the elevator
    );

    Logger.recordOutput("Algae/Pivot Position", getPosition());
  }
}
