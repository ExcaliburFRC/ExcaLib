package frc.excalib.mechanism.mechanisms;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.GenericFF.GenericFF;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.Motor;
import frc.excalib.mechanism.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * This class represents an Arm Mechanism
 */
public class Arm extends Mechanism {
    private final PIDController m_PIDController;
    private final ArmFeedforward m_ffController;

    private final TrapezoidProfile m_profile;

    private double m_setpoint, m_tolerance;
    public final Trigger atSetpointTrigger = new Trigger(()-> Math.abs(this.m_setpoint - super.logMechanismPosition()) < m_tolerance);

    public Arm(Motor motor, Gains gains, TrapezoidProfile.Constraints constraints, double tolerance) {
        super(motor);
        m_PIDController = gains.getPIDcontroller();
        m_ffController = gains.applyGains(new GenericFF.ArmFF());

        m_profile = new TrapezoidProfile(constraints);

        m_tolerance = tolerance;
        m_setpoint = 0;
    }

    public Command setDynamicPositionCommand(DoubleSupplier position, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            TrapezoidProfile.State state = m_profile.calculate(
                    0.02,
                    new TrapezoidProfile.State(
                            super.logMechanismPosition(),
                            super.logMechanismVelocity()),
                    new TrapezoidProfile.State(position.getAsDouble(), 0)
            );
            setPosition(state.position);
        }, requirements);
    }

    public void setPosition(double position){
        this.m_setpoint = position;

        double pid = m_PIDController.calculate(super.logMechanismPosition(), position);
        double ff = m_ffController.calculate(super.logMechanismPosition(), super.logMechanismVelocity());
        setVoltage(pid + ff);
    }
}
