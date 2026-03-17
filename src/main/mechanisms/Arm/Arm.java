package frc.excalib.mechanisms.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * This class represents an Arm Mechanism
 */
public class Arm extends Mechanism {
    private final Mass m_mass;
    private final PIDController PIDController;
    public final DoubleSupplier ANGLE_SUPPLIER;
    public final double m_kv, m_ks, m_kg;
    public final SoftLimit velocityLimit;

    public Arm(Motor motor,
               DoubleSupplier angleSupplier,
               SoftLimit velocityLimit,
               Gains gains,
               Mass mass) {
        super(motor);
        ANGLE_SUPPLIER = angleSupplier;
        this.velocityLimit = velocityLimit;
        m_kg = gains.kg;
        m_kv = gains.kv;
        m_ks = gains.ks;
        PIDController = new PIDController(gains.kp, gains.ki, gains.kd);
        m_mass = mass;
    }

    /**
     * @param setpointSupplier  the dynamic angle setpoint to go to (radians)
     * @param toleranceConsumer gets updated if the measurement is at tolerance.
     * @return a command that moves the arm to the specified dynamic setpoint.
     */
    public Command anglePositionControlCommand(
            DoubleSupplier setpointSupplier,
            Consumer<Boolean> toleranceConsumer,
            double maxOffSet,
            SubsystemBase... requirements) {
        final double dutyCycle = 0.02;
        return new RunCommand(
                () -> {
                    double error = setpointSupplier.getAsDouble() - ANGLE_SUPPLIER.getAsDouble();
                    double velocitySetpoint = error / dutyCycle;

                    velocitySetpoint = velocityLimit.limit(velocitySetpoint);
                    double phyOutput =
                            m_ks * Math.signum(velocitySetpoint) +
                                    m_kg * m_mass.getCenterOfMass().getX();
                    double pid = PIDController.calculate(ANGLE_SUPPLIER.getAsDouble(), setpointSupplier.getAsDouble());
                    double output = phyOutput + pid;
//                    System.out.println("PID:  "+pid);
//                    System.out.println("Angle:  " + ANGLE_SUPPLIER.getAsDouble());
//                    System.out.println("Setpoint:  "+ setPointSupplier.getAsDouble());
                    super.setVoltage(velocityLimit.limit(output));
                    toleranceConsumer.accept(Math.abs(error) < maxOffSet);
                }, requirements);
    }

    /**
     * @param angle             the angle setpoint to go to (radians)
     * @param toleranceConsumer gets updated if the measurement is at tolerance.
     * @return a command that moves the arm to the specified setpoint.
     */
    public Command goToAngleCommand(double angle, Consumer<Boolean> toleranceConsumer, double maxOffSet, SubsystemBase... requirements) {
        return anglePositionControlCommand(() -> angle, toleranceConsumer, maxOffSet, requirements);
    }
}
