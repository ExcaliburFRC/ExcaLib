package frc.excalib.hardware.motor;

import frc.excalib.hardware.motor.DirectionState;
import frc.excalib.hardware.motor.IdleState;

public interface Motor {
    void stopMotor();

    void setMotorVoltage(double voltage);

    void setPercentage(double percentage);

    void setFollower(int mainMotorID);

    void setIdleState(IdleState idleMode);

    IdleState getIdleState();

    int getDeviceID();

    double getMotorPosition();

    double getMotorVelocity();

    double getCurrent();

    double getVoltage();

    double getTemperature();

    void setSoftLimit(DirectionState directionState, float limit);

    void setInverted(DirectionState mode);

    void setPositionConversionFactor(double conversionFactor);

    void setVelocityConversionFactor(double conversionFactor);

    void setCurrentLimit(int stator, int supply);

    void setMotorPosition(double position);
}
