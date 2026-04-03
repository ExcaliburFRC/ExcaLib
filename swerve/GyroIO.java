package frc.excalib.swerve;

import frc.excalib.control.imu.IMU;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the Swerve Gyro.
 */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yawPositionRad = 0.0;
        public double pitchPositionRad = 0.0;
        public double rollPositionRad = 0.0;
        public double accelX = 0.0;
        public double accelY = 0.0;
        public double accelZ = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    default IMU getIMU() {
        return null;
    }
}
