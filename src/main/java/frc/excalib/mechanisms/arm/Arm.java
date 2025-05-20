package frc.excalib.mechanisms.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.GenericFF.GenericFF;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * This class represents an Arm Mechanism
 */
public class Arm extends Mechanism {
    private final PIDController m_PIDController;
    private final ArmFeedforward m_ffController;

    private final TrapezoidProfile m_profile;


    public Arm(Motor motor, Gains gains, TrapezoidProfile.Constraints constraints) {
        super(motor);
        m_PIDController = gains.getPIDcontroller();
        m_ffController = gains.applyGains(new GenericFF.ArmFF());

        m_profile = new TrapezoidProfile(constraints);
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
        double pid = m_PIDController.calculate(super.logMechanismPosition(), position);
        double ff = m_ffController.calculate(super.logMechanismPosition(), super.logMechanismVelocity());
        setVoltage(pid + ff);
    }
}
