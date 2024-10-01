package excalib.mechanisms.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import excalib.math.control.SoftLimit;
import excalib.math.control.motor.Motor;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ControlledArm extends Arm {
    public final Supplier<Translation2d> COM_SUPPLIER;
    private final PIDController m_PIDController;
    public final DoubleSupplier ANGLE_SUPPLIER;
    public final double m_kv, m_ks, m_kg;
    public final SoftLimit m_VELOCITY_LIMIT;

    public ControlledArm(Motor motor,
                         DoubleSupplier angleSupplier,
                         SoftLimit velocityLimit,
                         Supplier<Translation2d> comSupplier,
                         double kv,
                         double ks,
                         double kg,
                         double kp,
                         double ki,
                         double kd) {
        super(motor);
        ANGLE_SUPPLIER = angleSupplier;
        m_VELOCITY_LIMIT = velocityLimit;
        m_kg = kg;
        m_kv = kv;
        m_ks = ks;
        m_PIDController = new PIDController(kp, ki, kd);
        COM_SUPPLIER = comSupplier;

    }

    public Command anglePositionControlCommand(DoubleSupplier setPointSupplier) {
        final double dutyCycle = 0.02;
        return new RunCommand(() -> {
            double error = setPointSupplier.getAsDouble() - ANGLE_SUPPLIER.getAsDouble();
            double velocitySetpoint = error / dutyCycle;
            velocitySetpoint = m_VELOCITY_LIMIT.limit(velocitySetpoint);
            double phyOutput =
                    m_kv * velocitySetpoint +
                            m_ks * Math.signum(velocitySetpoint) +
                            m_kg * COM_SUPPLIER.get().getNorm() * COM_SUPPLIER.get().getAngle().getCos();
            double pid = m_PIDController.calculate(ANGLE_SUPPLIER.getAsDouble(), setPointSupplier.getAsDouble());
            super.setOutput(pid + phyOutput);
        });
    }

    public Command goToAngleCommand(double angle) {
        return anglePositionControlCommand(() -> angle);
    }
}