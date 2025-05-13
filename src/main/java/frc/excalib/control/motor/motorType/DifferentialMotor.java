package frc.excalib.control.motor.motorType;

import edu.wpi.first.math.Pair;
import frc.excalib.control.motor.Motor;
import frc.excalib.control.motor.motorType.MotorGroup;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;

public class DifferentialMotor extends MotorGroup {
    private final Motor motorA, motorB;

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
}
