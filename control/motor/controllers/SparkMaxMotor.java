package frc.excalib.control.motor.controllers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;

public class SparkMaxMotor extends SparkMax implements Motor {
    private final SparkFlexConfig config = new SparkFlexConfig();

    public SparkMaxMotor(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void stopMotor() {
        super.stopMotor();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public void setPercentage(double percentage) {
        super.set(percentage);
    }

    @Override
    public void setFollower(int mainMotorID) {
        config.follow(mainMotorID);
        configure();
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        config.idleMode(idleMode == BRAKE ? kBrake : kCoast);
        configure();
    }

    @Override
    public IdleState getIdleState() {
        return configAccessor.getIdleMode() == kCoast ? COAST : BRAKE;
    }

    @Override
    public int getDeviceID() {
        return super.getDeviceId();
    }

    @Override
    public double getMotorPosition() {
        return this.getEncoder().getPosition();
    }

    @Override
    public double getMotorVelocity() {
        return this.getEncoder().getVelocity() / 60; //we divide by 60 to get it in RPS
    }

    @Override
    public double getCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return getBusVoltage() * getAppliedOutput();
    }


    @Override
    public double getTemperature() {
        return getMotorTemperature();
    }


    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        if (directionState == FORWARD) {
            config.softLimit.forwardSoftLimit(limit);
        } else {
            config.softLimit.reverseSoftLimit(limit);
        }
        configure();
    }

    @Override
    public void setInverted(DirectionState mode) {
        config.inverted(mode != FORWARD);
        configure();
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        config.encoder.positionConversionFactor(conversionFactor);
        configure();
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        config.encoder.velocityConversionFactor(conversionFactor);
        configure();
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        config.smartCurrentLimit(stallLimit, freeLimit);
        configure();
    }

    @Override
    public void setMotorPosition(double position) {
        super.getEncoder().setPosition(position);
    }

    private void configure() {
        super.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
}
