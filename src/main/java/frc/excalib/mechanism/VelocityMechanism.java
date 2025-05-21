package frc.excalib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.Motor;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

public class VelocityMechanism extends Mechanism {
    private double m_setpoint, m_tolerance;
    private final PIDController m_pidController;
    private final BiFunction<Double, Double, Double> m_feedforwardCalculator;

    public final Trigger m_atSetpointTrigger;

    /**
     * @param motor                 the Mechanism motor
     * @param gains                 the FF and PID gains
     * @param tolerance             setpoint tolerance
     * @param feedforwardCalculator FF calculation
     */
    public VelocityMechanism(Motor motor, Gains gains, double tolerance,
                             BiFunction<Double, Double, Double> feedforwardCalculator) {
        super(motor);
        this.m_pidController = gains.getPIDcontroller();
        m_feedforwardCalculator = feedforwardCalculator;
        m_tolerance = tolerance;

        m_atSetpointTrigger = new Trigger(() -> MathUtil.isNear(this.m_setpoint, super.logMechanismVelocity(), m_tolerance));
    }

    /**
     * sets the Mechanism to a Dynamic velocity
     *
     * @param velocity     velocity supplier
     * @param requirements subsystem requirements
     * @return the command to set the FlyWheel velocity
     */
    public Command setVelocityCommand(DoubleSupplier velocity, SubsystemBase... requirements) {
        return new RunCommand(() -> setVelocity(velocity.getAsDouble()), requirements);
    }

    /**
     * @param velocity the velocity to set to the Mechanism
     */
    public void setVelocity(double velocity) {
        this.m_setpoint = velocity;

        double pid = m_pidController.calculate(super.m_motor.getMotorVelocity(), m_setpoint);
        double ff = m_feedforwardCalculator.apply(super.logMechanismPosition(), super.logMechanismVelocity());
        super.setVoltage(pid + ff);
    }
}
