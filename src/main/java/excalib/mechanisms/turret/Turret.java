package excalib.mechanisms.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import excalib.math.control.ContinuousSoftLimit;
import excalib.math.control.Gains;
import excalib.math.control.motor.Motor;
import excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

public final class Turret extends Mechanism {
    private final Motor m_motor;
    private final ContinuousSoftLimit m_rotationLimit;

    private final PIDController m_anglePIDcontroller;
    private final SimpleMotorFeedforward m_angleFFcontroller;
    private final DoubleSupplier m_POSITION_SUPPLIER;
    public Turret(Motor motor, ContinuousSoftLimit rotationLimit, Gains angleGains, double PIDtolerance, DoubleSupplier positionSupplier) {
        m_motor = motor;
        m_rotationLimit = rotationLimit;

        m_anglePIDcontroller = new PIDController(angleGains.kp, angleGains.ki, angleGains.kd);
        m_angleFFcontroller = new SimpleMotorFeedforward(angleGains.ks, angleGains.kv, angleGains.ka);

        m_anglePIDcontroller.setTolerance(PIDtolerance);
        m_POSITION_SUPPLIER = positionSupplier;
    }

    public Command setPosition(Rotation2d wantedPosition, SubsystemBase requirements) {
        double smartSetPoint = m_rotationLimit != null ?
                m_rotationLimit.getSetPoint(
                        m_POSITION_SUPPLIER.getAsDouble(),
                        wantedPosition.getRadians()
                ) :
                wantedPosition.getRadians();
        double pid = m_anglePIDcontroller.calculate(smartSetPoint);
        double ff = m_angleFFcontroller.calculate(pid);

        return new FunctionalCommand(
                () -> {
                },
                () -> m_motor.setVoltage(pid + ff),
                interrupt -> m_motor.stopMotor(),
                () -> false,
                requirements
        );
    }

    public Rotation2d getPosition() {
        return new Rotation2d(m_POSITION_SUPPLIER.getAsDouble());
    }

    public void stopTurret() {
        m_motor.stopMotor();
    }
}
