package frc.excalib.mechanism.mechanisms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.GenericFF.GenericFF;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.Motor;
import frc.excalib.mechanism.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * A class representing a Turret Mechanism
 */
public final class Turret extends Mechanism {
    private final PIDController m_pidController;
    private final SimpleMotorFeedforward m_ffController;

    private final TrapezoidProfile m_profile;
    private final ContinuousSoftLimit m_rotationLimit;

    private double m_setpoint, m_tolerance;
    public final Trigger atSetpointTrigger = new Trigger(()-> Math.abs(this.m_setpoint - super.logMechanismPosition()) < m_tolerance);

    /**
     * @param motor the turret's motor
     * @param rotationLimit the rotational boundary for the turret (radians)
     * @param angleGains pid gains for the turret
     * @param constraints constraints for the TrapezoidProfile
     */
    public Turret(Motor motor, ContinuousSoftLimit rotationLimit, Gains angleGains, TrapezoidProfile.Constraints constraints, double tolerance) {
        super(motor);
        m_rotationLimit = rotationLimit;

        m_pidController = angleGains.getPIDcontroller();
        m_ffController = angleGains.applyGains(new GenericFF.SimpleFF());

        m_pidController.enableContinuousInput(-180, 180);

        m_profile = new TrapezoidProfile(constraints);

        m_tolerance = tolerance;
        m_setpoint = 0;
    }

    /**
     * moves the Turret to a Dynamic position with a Trapezoid profile
     * @param position position supplier
     * @param requirements subsystem requirements
     * @return the command to move the Turret
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

        /**
         * moves the turret to the desired position
         * @param setpoint the wanted position of the turret.
         */
    public void setPosition(double setpoint) {
        this.m_setpoint = setpoint;

        double limitedSetpoint = m_rotationLimit.getSetPoint(super.logMechanismPosition(), setpoint);
        double pid = m_pidController.calculate(super.logMechanismPosition(), limitedSetpoint);
        double ff = m_ffController.getKs() * Math.signum(pid);
        super.setVoltage(pid + ff);
    }
}
