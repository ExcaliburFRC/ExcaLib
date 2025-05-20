package frc.excalib.mechanisms.fly_wheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.GenericFF.GenericFF;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * A class the represents A FlyWheel Mechanism.
 */
public class FlyWheel extends Mechanism {
    private final PIDController m_pidController;
    private final SimpleMotorFeedforward m_ffController;

    /**
     * @param motor the FlyWheel Motor
     * @param gains the FF and PID gains
     */
    public FlyWheel(Motor motor, Gains gains) {
        super(motor);
        this.m_pidController = gains.getPIDcontroller();
        this.m_ffController = gains.applyGains(new GenericFF.SimpleFF());
    }

    /**
     * sets the FlyWheel to a Dynamic velocity
     * @param velocity velocity supplier
     * @param requirements subsystem requirements
     * @return the command to set the FlyWheel velocity
     */
    public Command setDynamicVelocityCommand(DoubleSupplier velocity, SubsystemBase... requirements) {
        return new RunCommand(() -> setVelocity(velocity.getAsDouble()), requirements);
    }

    /**
     * @param velocity the velocity to set to the FlyWheel Dynamically
     */
    public void setVelocity(double velocity) {
        double pid = m_pidController.calculate(super.m_motor.getMotorVelocity(), velocity);
        double ff = m_ffController.calculate(velocity);
        super.setVoltage(pid + ff);
    }
}