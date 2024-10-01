package excalib.math.control.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import excalib.math.control.motor.motor_specs.DirectionState;
import excalib.math.control.motor.motor_specs.IdleState;

public class Kraken extends TalonFX implements Motor {
    private double m_positionConversionFactor;
    private double m_velocityConversionFactor;

    public Kraken(int deviceId, String canbus) {
        super(deviceId, canbus);
        m_positionConversionFactor = 1;
        m_velocityConversionFactor = 1;
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
    public void setIdleMode(IdleState idleMode) {
        super.setNeutralMode(idleMode == IdleState.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public double getMotorPosition() {
        return m_positionConversionFactor * super.getPosition().getValue();
    }

    @Override
    public double getMotorVelocity() {
        return m_velocityConversionFactor * super.getVelocity().getValue();
    }

    @Override
    public double getCurrent() {
        return super.getSupplyCurrent().getValue();
    }

    @Override
    public double getVoltage() {
        return super.getMotorVoltage().getValue();
    }

    @Override
    public double getTemperature() {
        return super.getDeviceTemp().getValue();
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        var talonFXConfigurator = super.getConfigurator();
        
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
        limitConfigs.SupplyCurrentLimit = freeLimit;
        limitConfigs.SupplyCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    @Override
    public void setInverted(boolean mode) {
        super.setInverted(mode);
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }
}
