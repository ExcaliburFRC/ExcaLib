package frc.excalib.mechanisms.mechanical_mechanisms;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.GenericFF.GenericFF;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

public class LinearExtension extends Mechanism {
    private final PIDController m_pidController;
    private final ElevatorFeedforward m_ffController;

    private final TrapezoidProfile m_profile;

    private double m_setpoint, m_tolerance;
    public final Trigger atSetpointTrigger = new Trigger(()-> Math.abs(this.m_setpoint - super.logMechanismPosition()) < m_tolerance);

    public LinearExtension(Motor motor, Gains gains, TrapezoidProfile.Constraints constraints, double tolerance) {
        super(motor);
        m_pidController = gains.getPIDcontroller();
        m_ffController = gains.applyGains(new GenericFF.ElevatorFF());

        m_profile = new TrapezoidProfile(constraints);

        m_tolerance = tolerance;
        m_setpoint = 0;
    }

    /**
     * moves the LinearExtension to a Dynamic position with a trapezoid profile
     * @param position position supplier
     * @param requirements subsystem requirements
     * @return the command to move the LinearExtension
     */
    public Command setDynamicPositionCommand(DoubleSupplier position, SubsystemBase... requirements){
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

        double pid = m_pidController.calculate(super.logMechanismPosition(), position);
        double ff = m_ffController.calculate(super.logMechanismVelocity());
        setVoltage(pid + ff);
    }
}