package frc.excalib.control.motor.controllers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.excalib.control.motor.Motor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;

/**
 * This class represents a implemantation of a motor controller
 * using the TalonFX hardware.
 */
public class TalonFXMotor extends TalonFX implements Motor {
    private double m_positionConversionFactor;
    private double m_velocityConversionFactor;
    private IdleState m_idleState = null;

    public TalonFXMotor(int deviceId, String canbus) {
        super(deviceId, canbus);
        m_positionConversionFactor = 1;
        m_velocityConversionFactor = 1;
        setIdleState(IdleState.BRAKE);
    }

    public TalonFXMotor(int deviceId) {
        super(deviceId);
        m_positionConversionFactor = 1;
        m_velocityConversionFactor = 1;
        setIdleState(IdleState.BRAKE);
    }

    @Override
    public void setPercentage(double percentage) {
        super.setControl(new DutyCycleOut(percentage));

    }

    @Override
    public void setFollower(int mainMotorID) {
        super.setControl(new Follower(mainMotorID, false));
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        m_idleState = idleMode;
        super.setNeutralMode(idleMode == IdleState.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public double getMotorPosition() {
        return m_positionConversionFactor * super.getPosition().getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return m_velocityConversionFactor * super.getVelocity().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return super.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public IdleState getIdleState() {return m_idleState;}

    @Override
    public double getVoltage() {
        return super.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return super.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        var talonFXConfigurator = super.getConfigurator(); //TODO: implement
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        m_positionConversionFactor = conversionFactor;
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        m_velocityConversionFactor = conversionFactor;
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        var talonFXConfigurator = super.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = freeLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    @Override
    public void setMotorPosition(double position) {
        super.setPosition(position / m_positionConversionFactor);
    }

    @Override
    public void setInverted(DirectionState mode) {
        var talonFXConfigurator = new MotorOutputConfigs();
        talonFXConfigurator.withInverted(mode == FORWARD? CounterClockwise_Positive : Clockwise_Positive);
        super.getConfigurator().apply(talonFXConfigurator);
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }
}
