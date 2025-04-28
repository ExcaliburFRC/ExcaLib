package frc.excalib.mechanisms.diffrential;

import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

public class Diffrential extends Mechanism {
    private double setpoint;
    public Diffrential(MotorGroup motorGroup, DoubleSupplier angleSetpointSupplier, DoubleSupplier angleSupplier, DoubleSupplier setpointSupplier) {
        super(motorGroup);
        motorGroup.setIdleState(IdleState.BRAKE);
    }

    public void setVoltage(DoubleSupplier firstMotorVoltage, DoubleSupplier secondMotorVoltage) {
        super.setVoltage((firstMotorVoltage.getAsDouble() + secondMotorVoltage.getAsDouble()) /2);
    }

    public void setAngleSupplier(DoubleSupplier angleSupplier){
        
    }



}
