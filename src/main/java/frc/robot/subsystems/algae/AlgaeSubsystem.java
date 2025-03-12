// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
    private SparkFlex intakeMotor;
    private SparkMax pivotMotor;

    private SparkClosedLoopController rotationClosedLoopController;

    /** Creates a new AlgaeSubsystem. */
    public AlgaeSubsystem() {
        this.intakeMotor = new SparkFlex(Constants.AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
        this.pivotMotor = new SparkMax(Constants.AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);

        SparkMaxConfig rotationConfig = new SparkMaxConfig();

        rotationConfig.closedLoop.p(0.1).outputRange(-0.5, 0.5);

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
    public void periodic() {
        // This method will be called once per scheduler run

        Logger.recordOutput("Algae/Pivot Position", getPosition());
    }
}
