package org.frogforce503.lib.subsystem;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class ArmIOSpark  implements ArmIO{

    private SparkMax motor;
    private SparkAbsoluteEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    public SparkClosedLoopController controlller = motor.getClosedLoopController();
    private ClosedLoopSlot closedLoopSlot;
    public ArmIOSpark() {
        motor = new SparkMax(0, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        
        
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(35)
            .voltageCompensation(12);

        config.absoluteEncoder
            .zeroOffset(0.1)
            .positionConversionFactor(2 * Math.PI)
            .velocityConversionFactor((2 * Math.PI) / 60);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0.1, 0.0, 0.02, ClosedLoopSlot.kSlot0)
            .pid(0.1, 0.0, 0.02, ClosedLoopSlot.kSlot1)
            .pid(0.1, 0.0, 0.02, ClosedLoopSlot.kSlot2)
            .pid(0.1, 0.0, 0.02, ClosedLoopSlot.kSlot3);
    
        REVLibError error = motor.configure(
        config,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters
    );
    }
    public void updateInputs(ArmIOData inputs) {
        inputs = new ArmIOData(
            encoder.getPosition(),
            motor.getOutputCurrent(),
            encoder.getVelocity(),
            motor.getBusVoltage() * motor.getAppliedOutput(),
            motor.getMotorTemperature(),
            motor.getLastError()
        );
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

        
    public void setVoltage(double voltage) {
        controlller.setReference(voltage, SparkMax.ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setPosition(double positionRadians) {
        controlller.setReference(positionRadians, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setPID(double kP, double kI, double kD, int slot) {
        ClosedLoopSlot closedLoopSlot;
        
        switch (slot) {
            case 0:
                closedLoopSlot = ClosedLoopSlot.kSlot0;
                break;
            case 1:
                closedLoopSlot = ClosedLoopSlot.kSlot1;
                break;
            case 2:
                closedLoopSlot = ClosedLoopSlot.kSlot2;
                break;
            case 3:
                closedLoopSlot = ClosedLoopSlot.kSlot3;
                break;
            default:
                closedLoopSlot = ClosedLoopSlot.kSlot0;
        }
        config.closedLoop.pid(kP, kI, kD, closedLoopSlot);
    }

    @Override
    public double setOperatingMode(boolean isPositionMode) {
        return motor.getLastError().value;
    }

    @Override
    public void stop() {}

    @Override
    public void reset() {}
}
