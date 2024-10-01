package excalib.math.control.motor;

import excalib.math.control.motor.motor_specs.IdleState;
import excalib.math.control.motor.motor_specs.DirectionState;

public interface Motor {
    void stopMotor();

    void setVoltage(double voltage);
    void setPercentage(double percentage);

    void setFollower(int mainMotorID);

    void setIdleMode(IdleState idleMode);

    int getDeviceID();

    double getMotorPosition();

    double getMotorVelocity();

    double getCurrent();

    double getVoltage();

    double getTemperature();

void setSoftLimit(DirectionState directionState, float limit);

void setInverted(boolean mode);

void setPositionConversionFactor(double conversionFactor);

void setVelocityConversionFactor(double conversionFactor);

void setCurrentLimit(int stallLimit, int freeLimit);

}
