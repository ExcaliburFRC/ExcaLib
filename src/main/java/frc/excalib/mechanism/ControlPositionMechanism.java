package frc.excalib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.Motor;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

public class ControlPositionMechanism extends Mechanism{
    private final PIDController m_pidController;
    private BiFunction<Double, Double, Double> m_feedforwardCalculator;

    private SoftLimit limit;

    private double m_setpoint, m_tolerance;
    public final Trigger atSetpointTrigger = new Trigger(()-> MathUtil.isNear(this.m_setpoint, super.logMechanismPosition(), m_tolerance));

    public ControlPositionMechanism(Motor motor, Gains gains, double tolerance,
                                    BiFunction<Double, Double, Double> feedforwardCalculator) {
        super(motor);
        this.m_pidController = gains.getPIDcontroller();
        m_feedforwardCalculator = feedforwardCalculator;

        m_tolerance = tolerance;
        m_setpoint = 0;
    }

    /**
     * moves the Turret to a Dynamic position with a Trapezoid profile
     * @param position position supplier
     * @param requirements subsystem requirements
     * @return the command to move the Turret
     */
    public Command setPositionCommand(DoubleSupplier position, TrapezoidProfile profile, SubsystemBase... requirements){
        return new RunCommand(() -> {
            TrapezoidProfile.State state = profile.calculate(
                    0.02,
                    new TrapezoidProfile.State(
                            super.logMechanismPosition(),
                            super.logMechanismVelocity()),
                    new TrapezoidProfile.State(position.getAsDouble(), 0)
            );
            setPosition(state.position);
        }, requirements);
    }

    public void addLimit(SoftLimit limit){
        this.limit = limit;
    }

    /**
     * @param position the position to set to the FlyWheel Dynamically
     */
    public void setPosition(double position) {
        this.m_setpoint = position;

        if (limit != null){
            if (limit instanceof ContinuousSoftLimit) m_setpoint = ((ContinuousSoftLimit) limit).getSetpoint(logMechanismPosition(), position);
            else m_setpoint = limit.limit(position);
        }

        double pid = m_pidController.calculate(super.m_motor.getMotorVelocity(), m_setpoint);
        double ff = m_feedforwardCalculator.apply(super.logMechanismPosition(), super.logMechanismVelocity());
        super.setVoltage(pid + ff);
    }
}
