package frc.excalib.control.motor.controllers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.excalib.control.motor.Motor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;

/**
 *  This class represents a implemantation of a motor
 *  controller using the Rev SparkMax hardware.
 */
public class SparkMaxMotor extends SparkMax implements Motor {
    private final SparkFlexConfig m_config = new SparkFlexConfig();

    public SparkMaxMotor(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void stopMotor() {
        super.stopMotor();
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public void setPercentage(double percentage) {
        super.set(percentage);
    }

    @Override
    public void setFollower(int mainMotorID) {
        m_config.follow(mainMotorID);
        configure();
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        m_config.idleMode(idleMode == BRAKE ? kBrake : kCoast);
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
            m_config.softLimit.forwardSoftLimit(limit);
        } else {
            m_config.softLimit.reverseSoftLimit(limit);
        }
        configure();
    }

    @Override
    public void setInverted(DirectionState mode) {
        m_config.inverted(mode != FORWARD);
        configure();
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        m_config.encoder.positionConversionFactor(conversionFactor);
        configure();
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        m_config.encoder.velocityConversionFactor(conversionFactor);
        configure();
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        m_config.smartCurrentLimit(stallLimit, freeLimit);
        configure();
    }

    @Override
    public void setMotorPosition(double position) {
        super.getEncoder().setPosition(position);
    }

    private void configure() {
        super.configure(m_config, kResetSafeParameters, kPersistParameters);
    }
}
