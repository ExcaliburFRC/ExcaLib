package frc.excalib.control.motor.controllers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import java.util.ArrayList;
import java.util.HashMap;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;

public class TalonFXMotor extends TalonFX implements Motor {
    private double m_positionConversionFactor;
    private double m_velocityConversionFactor;
    private IdleState m_idleState = null;
    private final StatusSignal<Angle> poseSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final static ArrayList<TalonFXMotor> motors = new ArrayList<>();
    private String canbus = "";
    private static final HashMap<String, ArrayList<BaseStatusSignal>> canMap = new HashMap<>();

    public TalonFXMotor(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        m_positionConversionFactor = 1;
        m_velocityConversionFactor = 1;
        setIdleState(IdleState.BRAKE);
        poseSignal = super.getPosition();
        velocitySignal = super.getVelocity();
        currentSignal = super.getSupplyCurrent();
        voltageSignal = super.getMotorVoltage();
        temperatureSignal = super.getDeviceTemp();
        this.canbus = canbus.getName();
        ArrayList<BaseStatusSignal> signals = canMap.get(this.canbus);
        if (signals == null) {
            canMap.put(this.canbus, new ArrayList<>());
            signals = canMap.get(this.canbus);
        }
        signals.add(poseSignal);
        signals.add(velocitySignal);
        signals.add(currentSignal);
        signals.add(voltageSignal);
        signals.add(temperatureSignal);

        motors.add(this);
    }

    public TalonFXMotor(int deviceId) {
        super(deviceId);
        m_positionConversionFactor = 1;
        m_velocityConversionFactor = 1;
        setIdleState(IdleState.BRAKE);

        poseSignal = super.getPosition();
        velocitySignal = super.getVelocity();
        currentSignal = super.getSupplyCurrent();
        voltageSignal = super.getMotorVoltage();
        temperatureSignal = super.getDeviceTemp();

        ArrayList<BaseStatusSignal> signals = canMap.get(this.canbus);
        if (signals == null) {
            canMap.put(this.canbus, new ArrayList<>());
            signals = canMap.get(this.canbus);
        }
        signals.add(poseSignal);
        signals.add(velocitySignal);
        signals.add(currentSignal);
        signals.add(voltageSignal);
//        signals.add(temperatureSignal);


        motors.add(this);

    }

    public static void refreshAll() {
        for (ArrayList<BaseStatusSignal> signals : canMap.values()) {
            BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
        }
    }

    public void refresh() {
        ArrayList<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(poseSignal);
        signals.add(velocitySignal);
        signals.add(currentSignal);
        signals.add(voltageSignal);
        signals.add(temperatureSignal);
        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
    }

    @Override
    public void setPercentage(double percentage) {
        super.setControl(new DutyCycleOut(percentage));
    }

    @Override
    public void setFollower(int mainMotorID) {
        super.setControl(new Follower(mainMotorID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        m_idleState = idleMode;
        super.setNeutralMode(idleMode == IdleState.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public double getMotorPosition() {
        return m_positionConversionFactor * poseSignal.getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return m_velocityConversionFactor * velocitySignal.getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return currentSignal.getValueAsDouble();
    }

    @Override
    public IdleState getIdleState() {
        return m_idleState;
    }

    @Override
    public double getVoltage() {
        return voltageSignal.getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return temperatureSignal.getValueAsDouble();
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
    public void setCurrentLimit(int statorLimit, int supplyLimit) {
        var talonFXConfigurator = super.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        talonFXConfigurator.refresh(limitConfigs);
        limitConfigs.StatorCurrentLimit = statorLimit;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimit = supplyLimit;
        limitConfigs.SupplyCurrentLimitEnable = true;
        talonFXConfigurator.apply(limitConfigs);
    }

    @Override
    public void setMotorPosition(double position) {
        super.setPosition(position / m_positionConversionFactor);
    }

    @Override
    public void setInverted(DirectionState mode) {
        var talonFXConfigurator = new MotorOutputConfigs();
        talonFXConfigurator.withInverted(mode == FORWARD ? CounterClockwise_Positive : Clockwise_Positive);
        super.getConfigurator().apply(talonFXConfigurator);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        super.setVoltage(voltage);
    }
}
