package frc.excalib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.encoder.Encoder;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.Motor;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

public class PositionMechanism extends Mechanism {
    private double m_setpoint, m_tolerance;
    private PIDController m_pidController;
    private BiFunction<Double, Double, Double> m_feedforwardCalculator;
    private SoftLimit m_limit;

    public Trigger m_atSetpointTrigger;

    private PositionMechanism(Builder builder) {
        super(builder.motor);
        super.addExternalEncoder(builder.externalEncoder);

        this.m_pidController = builder.gains.getPIDcontroller();
        this.m_feedforwardCalculator = builder.feedforwardCalculator;
        this.m_tolerance = builder.tolerance;
        this.m_limit = builder.limit;
        this.m_setpoint = 0;

        this.m_atSetpointTrigger = new Trigger(() -> MathUtil.isNear(this.m_setpoint, super.logMechanismPosition(), m_tolerance));
    }

    public static final class Builder {
        private Motor motor;
        private Gains gains;
        private double tolerance = 1;
        private SoftLimit limit = null;
        private Encoder externalEncoder = null;
        private BiFunction<Double, Double, Double> feedforwardCalculator = (pos, vel) -> 0.0;

        public Builder(Motor motor, Gains gains){
            this.motor = motor;
            this.gains = gains;
        }

        public Builder withTolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public Builder withFeedforwardCalculator(BiFunction<Double, Double, Double> feedforward) {
            this.feedforwardCalculator = feedforward;
            return this;
        }

        public Builder withLimit(SoftLimit limit) {
            this.limit = limit;
            return this;
        }

        public Builder withExternalEncoder(Encoder encoder) {
            this.externalEncoder = encoder;
            return this;
        }

        public PositionMechanism build() {
            if (motor == null || gains == null) {
                throw new IllegalStateException("Motor and Gains are required for PositionMechanism");
            }
            return new PositionMechanism(this);
        }
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier, TrapezoidProfile profile, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            State state = profile.calculate(
                    0.02,
                    new State(super.logMechanismPosition(), super.logMechanismVelocity()),
                    new State(positionSupplier.getAsDouble(), 0)
            );
            setPosition(state.position);
        }, requirements);
    }

    public void setPosition(double position) {
        m_setpoint = position;

        if (m_limit != null) {
            if (m_limit instanceof ContinuousSoftLimit) m_setpoint = ((ContinuousSoftLimit) m_limit).getSetpoint(super.logMechanismPosition(), position);
            else m_setpoint = m_limit.limit(position);
        }

        double pid = m_pidController.calculate(super.logMechanismPosition(), m_setpoint);
        double ff = m_feedforwardCalculator.apply(super.logMechanismPosition(), super.logMechanismVelocity());
        super.setVoltage(pid + ff);
    }
}
