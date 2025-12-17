package org.frogforce503.lib.subsystem;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.REVLibError;

public interface ArmIO {
    
    @AutoLog
    class ArmIOInputs {
        ArmIOData data = new ArmIOData(0, 0, false, 0, 0, 0);
    }

    record ArmIOData(double positionRadians, double voltage, boolean motorConnected, double current, double velocity, double temperature){

        public ArmIOData(double position, double appliedOutput, double outputCurrent, double velocity2,
                double motorTemperature, REVLibError lastError) {
            this(position, appliedOutput, true, outputCurrent, velocity2, motorTemperature);
        }
    }

    default void updateInputs(ArmIOData inputs) {
        updateInputs(inputs, true);
    }

    default void updateInputs(ArmIOData inputs, boolean motorConnected) {
    }
    
    public double getPosition();

    public double getVelocity();

    public void setPosition(double positionRadians);

    public void setPID(double kP, double kI, double kD, int slot);

    public double setOperatingMode(boolean isPositionMode);

    public void stop();

    public void reset();
}
