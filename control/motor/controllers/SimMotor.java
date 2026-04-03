package frc.excalib.control.motor.controllers;

import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

public class SimMotor implements Motor {
    private final int deviceId;
    
    // Configs
    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;
    private DirectionState inverted = DirectionState.FORWARD;
    private IdleState idleState = IdleState.COAST;
    
    // Outputs from simulated algorithm to the motor
    private double appliedVoltage = 0.0;
    
    // Inputs from simulator to update state (for getters)
    private double simulatedPosition = 0.0;
    private double simulatedVelocity = 0.0;
    private double simulatedCurrent = 0.0;

    public SimMotor(int deviceId) {
        this.deviceId = deviceId;
    }

    /**
     * Set the simulated physical state to be returned by sensory functions.
     * @param position True simulated position
     * @param velocity True simulated velocity
     * @param current Simulated current draw
     */
    public void setSimulatedState(double position, double velocity, double current) {
        this.simulatedPosition = Double.isNaN(position) ? 0.0 : position;
        this.simulatedVelocity = Double.isNaN(velocity) ? 0.0 : velocity;
        this.simulatedCurrent = Double.isNaN(current) ? 0.0 : current;
    }

    public double getAppliedVoltage() {
        if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) return 0.0;
        return inverted == DirectionState.FORWARD ? appliedVoltage : -appliedVoltage;
    }

    @Override
    public void stopMotor() {
        appliedVoltage = 0.0;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        if (Double.isNaN(voltage)) {
            this.appliedVoltage = 0.0;
        } else {
            this.appliedVoltage = Math.max(-12.0, Math.min(12.0, voltage));
        }
    }

    @Override
    public void setPercentage(double percentage) {
        setMotorVoltage(percentage * 12.0);
    }

    @Override
    public void setFollower(int mainMotorID) {
        // No-op for simple simulation
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        this.idleState = idleMode;
    }

    @Override
    public IdleState getIdleState() {
        return idleState;
    }

    @Override
    public int getDeviceID() {
        return deviceId;
    }

    @Override
    public double getMotorPosition() {
        double pos = simulatedPosition * positionConversionFactor;
        return inverted == DirectionState.FORWARD ? pos : -pos;
    }

    @Override
    public double getMotorVelocity() {
        double vel = simulatedVelocity * velocityConversionFactor;
        return inverted == DirectionState.FORWARD ? vel : -vel;
    }

    @Override
    public double getCurrent() {
        return simulatedCurrent;
    }

    @Override
    public double getVoltage() {
        if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) return 0.0;
        return appliedVoltage;
    }

    @Override
    public double getTemperature() {
        return 30.0; // Simulated constant temperature
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        // No-op for sim
    }

    @Override
    public void setInverted(DirectionState mode) {
        this.inverted = mode;
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        this.positionConversionFactor = conversionFactor;
    }

    public double getPositionConversionFactor() {
        return positionConversionFactor;
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        this.velocityConversionFactor = conversionFactor;
    }

    @Override
    public void setCurrentLimit(int stator, int supply) {
        // No-op for sim
    }

    @Override
    public void setMotorPosition(double position) {
        double rawPos = position / positionConversionFactor;
        this.simulatedPosition = inverted == DirectionState.FORWARD ? rawPos : -rawPos;
    }
}
