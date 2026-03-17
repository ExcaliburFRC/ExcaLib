package frc.excalib.mechanisms.fly_wheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * A class the represents A FlyWheel Mechanism.
 */
public class FlyWheel extends Mechanism {
    private double lastTime, lastVelocity;
    private final PIDController m_pidController;
    private final double maxAcceleration;
    private final double maxJerk;
    private final Gains m_gains;
    private final SimpleMotorFeedforward m_FF_CONTROLLER;

    /**
     * @param motor           the FlyWheel Motor
     * @param maxAcceleration the max acceleration of the FlyWheel
     * @param maxJerk         the max jerk of the FlyWheel
     * @param gains           the FF and PID gains
     */
    public FlyWheel(Motor motor, double maxAcceleration, double maxJerk, Gains gains) {
        super(motor);
        m_gains = gains;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        this.m_pidController = new PIDController(m_gains.kp, m_gains.ki, m_gains.kd);
        this.m_FF_CONTROLLER = new SimpleMotorFeedforward(gains.ks, gains.kv, gains.ka);
    }

    /**
     * @param velocitySupplier a dynamic velocity setpoint.
     * @return a command that controls the FlyWheels velocity with high precision
     */
    public Command smartVelocityCommand(DoubleSupplier velocitySupplier, SubsystemBase... requirements) {
        return new RunCommand(
                () -> {
                    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAcceleration, maxJerk));
                    TrapezoidProfile.State state = profile.calculate(
                            0.02,
                            new TrapezoidProfile.State(super.m_motor.getMotorVelocity(), getAcceleration()),
                            new TrapezoidProfile.State(velocitySupplier.getAsDouble(), 0));
                    double ff = m_gains.ks * Math.signum(state.position) +
                            m_gains.kv * state.position +
                            m_gains.ka * state.velocity;
                    double pid = m_pidController.calculate(super.m_motor.getMotorVelocity(), state.position);
                    setVoltage(pid + ff);
                });
    }

    /**
     * @return the FlyWheels current acceleration
     */
    private double getAcceleration() {
        //TODO: move to current based calculations
        double currentTime = Timer.getFPGATimestamp();
        double currentVelocity = super.m_motor.getMotorVelocity();
        return (currentVelocity - lastVelocity) / (currentTime - lastTime);
    }

    /**
     * @param velocity the dynamic velocity setpoint
     * @return a command which controls the FlyWheels velocity
     */
    public Command setDynamicVelocityCommand(DoubleSupplier velocity, SubsystemBase... requirements) {
        return new RunCommand(() -> setDynamicVelocity(velocity.getAsDouble()), requirements);
    }

    /**
     * @param velocity the velocity to set to the FlyWheel Dynamically
     */
    public void setDynamicVelocity(double velocity) {
        double ff = m_FF_CONTROLLER.calculate(velocity);
        double pid = m_pidController.calculate(super.m_motor.getMotorVelocity(), velocity);
        super.setVoltage(pid + ff);
    }

    public void periodic() {
        lastTime = Timer.getFPGATimestamp();
        lastVelocity = super.m_motor.getMotorVelocity();
    }

    /**
     * @return the current velocity of the FlyWheel.
     */
    public double getVelocity() {
        return super.m_motor.getMotorVelocity();
    }
}