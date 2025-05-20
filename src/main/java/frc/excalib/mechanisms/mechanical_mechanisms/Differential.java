package frc.excalib.mechanisms.mechanical_mechanisms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.motorType.DifferentialMotor;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log;

import java.util.function.DoubleSupplier;

public class Differential extends Mechanism {
    private final PIDController pidController_A, pidController_B;

    private final DifferentialMotor motor;

    private final double differentialMul;

    private double deltaA, deltaB;
    private double motorApos, motorBpos;

//    private DoubleSupplier angleSetpointSupplier, linearSetpointSupplier;
//    DoubleSupplier angleSetpointSupplier, DoubleSupplier linearSetpointSupplier

    /**
     * @param differentialMotor a differential motor object of both mechanism motors
     * @param gains gains for pid & ff for the two motors
     * @param differentialMul an optional multiplier to correct acc error in certain types of differential mechanism
     */
    public Differential(DifferentialMotor differentialMotor, Gains gains, double differentialMul) {
        super(differentialMotor);

        this.motor = differentialMotor;
        this.motor.setIdleState(IdleState.BRAKE);

        this.pidController_A = gains.getPIDcontroller();
        this.pidController_B = gains.getPIDcontroller();

        this.differentialMul = differentialMul;
    }

    /**
     * constructor for a manual diff, w/o advanced control capabilities
     * @param differentialMotor
     */
    public Differential(DifferentialMotor differentialMotor) {
        this(differentialMotor, new Gains(),1);
    }

    public Command moveToStateCommand(double angle, double position, double ff, SubsystemBase... requirements){
        return Commands.startRun(
                ()-> {
                    double deltaTheta = angle - getMechanismAngle();
                    double deltaPos = position - getMechanismPosition();

                    deltaA = deltaTheta + (deltaPos / (2 * differentialMul));
                    deltaB = deltaTheta - (deltaPos / (2 * differentialMul));
                },
                ()-> {
                    this.motorApos = this.motor.getMotorPositions().getFirst();
                    this.motorBpos = this.motor.getMotorPositions().getSecond();

                    setDifferentialVoltage(
                            pidController_A.calculate(motorApos, this.deltaA) + ff,
                            pidController_B.calculate(motorBpos, this.deltaB) + ff
                    );
                },
                requirements);
    }

    public Command setDifferentialVoltageCommand(DoubleSupplier voltageA, DoubleSupplier voltageB, SubsystemBase... requirements){
        return new RunCommand(
                ()-> setDifferentialVoltage(voltageA.getAsDouble(), voltageB.getAsDouble()),
                requirements);
    }

    public void move(double voltage) {
        super.setDifferentialVoltage(voltage, -voltage);
    }

    public void rotate(double voltage) {
        super.setDifferentialVoltage(voltage, voltage);
    }

    public void setDifferentialVoltage(double voltageA, double voltageB) {
        super.setDifferentialVoltage(voltageA, voltageB);
    }

    @Log.NT
    public double getMechanismAngle() {
        return this.motor.getMotorDifference();
    }

    @Log.NT
    public double getMechanismPosition(){
        return this.differentialMul * this.motor.getMotorDifference();
    }
}