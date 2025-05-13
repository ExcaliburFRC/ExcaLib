package frc.excalib.control.motor.motorType;

import edu.wpi.first.math.Pair;
import frc.excalib.control.motor.Motor;
import frc.excalib.control.motor.motorType.MotorGroup;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;

public class DifferentialMotor extends MotorGroup {
    private final Motor motorA, motorB;

    /**
     *
     * @param motorA first motor in the differential mechanism
     * @param motorB second motor in the differential mechanism
     * @param negate make sure that applying the same voltage causes rotation and not translation.
     */
    public DifferentialMotor(Motor motorA, Motor motorB, boolean negate) {
        super(motorA, motorB);

        this.motorA = motorA;
        this.motorB = motorB;

        motorB.setInverted(negate? REVERSE : FORWARD);
    }

    public void setDifferentialVoltage(double motorAvoltage, double motorBvoltage){
        super.setDifferentialVoltage(motorAvoltage, motorBvoltage);
    }

    public Pair<Double, Double> getMotorPositions(){
        return Pair.of(motorA.getMotorPosition(), motorB.getMotorPosition());
    }

    /**
     * computes the avg of the two motors
     * @return the rotation of the differential axle
     */
    public double getMotorAvgPosition(){
        return (motorA.getMotorPosition() + motorB.getMotorPosition()) / 2.0;
    }

    public double getMotorDifference(){
        return motorA.getMotorPosition() - motorB.getMotorPosition();
    }

    public double getMotorSum(){
        return motorA.getMotorPosition() + motorB.getMotorPosition();
    }
}
