package frc.excalib.mechanisms.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A class representing a Turret Mechanism
 */
public final class Turret extends Mechanism {
    private final ContinuousSoftLimit m_rotationLimit;
    private final PIDController m_anglePIDcontroller;
    private final DoubleSupplier m_positionSupplier;

    /**
     * @param motor            the turrets motor
     * @param rotationLimit    the rotational boundary for the turret (radians)
     * @param angleGains       pid gains for the turret
     * @param PIDtolerance     pid tolerance for the turret (radians)
     * @param positionSupplier the position measurement
     */
    public Turret(Motor motor, ContinuousSoftLimit rotationLimit, Gains angleGains, double PIDtolerance, DoubleSupplier positionSupplier) {
        super(motor);
        m_rotationLimit = rotationLimit;

        m_anglePIDcontroller = new PIDController(angleGains.kp, angleGains.ki, angleGains.kd);

        m_anglePIDcontroller.setTolerance(PIDtolerance);
        m_anglePIDcontroller.enableContinuousInput(-Math.PI, Math.PI);

        m_positionSupplier = positionSupplier;
    }

    /**
     * @param wantedPosition a Rotation2d dynamic setpoint
     * @return a Command that moves the turret to the given setpoint
     */
    public Command setPositionCommand(Supplier<Rotation2d> wantedPosition, SubsystemBase... requirements) {
        return new RunCommand(() -> setPosition(wantedPosition.get()), requirements);
    }

    /**
     * moves the turret to the desired position
     *
     * @param wantedPosition the wanted position of the turret.
     */
    public void setPosition(Rotation2d wantedPosition) {
        double smartSetpoint = m_rotationLimit.getOptimizedSetpoint(getTurretPosition().getRadians(), wantedPosition.getRadians());
        double pid = m_anglePIDcontroller.calculate(m_positionSupplier.getAsDouble(), smartSetpoint);
        super.setVoltage(pid);
    }

    /**
     * @return get the position if the turret
     */
    public Rotation2d getTurretPosition() {
        return new Rotation2d(m_positionSupplier.getAsDouble());

    }
}
