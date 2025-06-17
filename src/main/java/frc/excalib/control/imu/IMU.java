package frc.excalib.control.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Interface representing an Inertial Measurement Unit (IMU) for robotics control.
 * Provides methods to access orientation and acceleration data, as well as to reset or set the IMU's rotation.
 */
public interface IMU {
    /**
     * Gets the current Z-axis rotation (yaw) as a {@link Rotation2d}.
     *
     * @return the Z-axis rotation
     */
    Rotation2d getZRotation();

    /**
     * Gets the current X-axis rotation (roll) as a {@link Rotation2d}.
     *
     * @return the X-axis rotation
     */
    Rotation2d getXRotation();

    /**
     * Gets the current Y-axis rotation (pitch) as a {@link Rotation2d}.
     *
     * @return the Y-axis rotation
     */
    Rotation2d getYRotation();

    /**
     * Gets the current acceleration along the X-axis.
     *
     * @return the X-axis acceleration in meters per second squared
     */
    double getAccX();

    /**
     * Gets the current acceleration along the Y-axis.
     *
     * @return the Y-axis acceleration in meters per second squared
     */
    double getAccY();

    /**
     * Gets the current acceleration along the Z-axis.
     *
     * @return the Z-axis acceleration in meters per second squared
     */
    double getAccZ();

    /**
     * Resets the IMU's orientation to its default state.
     */
    void resetIMU();

    /**
     * Sets the IMU's rotation using a {@link Rotation3d} object.
     *
     * @param rotation the new rotation to set
     */
    void setRotation(Rotation3d rotation);

    /**
     * Sets the IMU's rotation using a {@link Rotation2d} object.
     *
     * @param rotation the new rotation to set
     */
    void setRotation(Rotation2d rotation);
}