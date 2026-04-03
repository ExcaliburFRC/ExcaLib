package frc.excalib.swerve;

import frc.excalib.control.motor.controllers.Motor;
import frc.robot.util.VoidMotor;
import org.littletonrobotics.junction.AutoLog;
import java.util.function.DoubleSupplier;

/**
 * Hardware abstraction interface for a module in the Swerve subsystem.
 * Separates hardware interaction from logic to enable AdvantageKit log replay.
 */
public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionConverted = 0.0;
        public double driveVelocityConverted = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double turnAbsolutePosition = 0.0;
        public double turnPositionConverted = 0.0;
        public double turnVelocityConverted = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs from hardware. */
    default void updateInputs(SwerveModuleIOInputs inputs) {}

    /**
     * Returns the drive motor for the module.
     */
    default Motor getDriveMotor() {
        return new VoidMotor();
    }

    /**
     * Returns the turn (rotation) motor for the module.
     */
    default Motor getTurnMotor() {
        return new VoidMotor();
    }

    /**
     * Returns the absolute encoder supplier for the module.
     */
    default DoubleSupplier getAbsoluteEncoder() {
        return () -> 0.0;
    }
}
