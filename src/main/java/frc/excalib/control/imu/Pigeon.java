package frc.excalib.control.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * Implementation of the {@link IMU} interface using the Pigeon2 sensor.
 * Provides orientation and acceleration data, with support for rotation offsets.
 */
public class Pigeon extends Pigeon2 implements IMU {
    /** The offset rotation applied to all orientation readings. */
    private Rotation3d m_offsetRotation;

    /**
     * Constructs a new Pigeon instance with a specified device ID and offset rotation.
     *
     * @param deviceId the CAN device ID of the Pigeon2 sensor
     * @param offsetRotation the initial offset rotation to apply
     */
    public Pigeon(int deviceId, Rotation3d offsetRotation) {
        super(deviceId);
        this.m_offsetRotation = offsetRotation;
    }

    /**
     * Constructs a new Pigeon instance with a specified device ID, CAN bus, and offset rotation.
     *
     * @param deviceId the CAN device ID of the Pigeon2 sensor
     * @param canbus the CAN bus name
     * @param offsetRotation the initial offset rotation to apply
     */
    public Pigeon(int deviceId, String canbus, Rotation3d offsetRotation) {
        super(deviceId, canbus);
        this.m_offsetRotation = offsetRotation;
    }

    /**
     * Gets the current Z-axis rotation (yaw), adjusted by the offset.
     *
     * @return the Z-axis rotation as a {@link Rotation2d}
     */
    @Override
    public Rotation2d getZRotation() {
        return super.getRotation2d().rotateBy(m_offsetRotation.toRotation2d());
    }

    /**
     * Gets the current X-axis rotation (roll), adjusted by the offset.
     *
     * @return the X-axis rotation as a {@link Rotation2d}
     */
    @Override
    public Rotation2d getXRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getRoll().getValueAsDouble() + m_offsetRotation.getX()));
    }

    /**
     * Gets the current Y-axis rotation (pitch), adjusted by the offset.
     *
     * @return the Y-axis rotation as a {@link Rotation2d}
     */
    @Override
    public Rotation2d getYRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getPitch().getValueAsDouble() + m_offsetRotation.getY()));
    }

    /**
     * Gets the current acceleration along the X-axis.
     *
     * @return the X-axis acceleration in meters per second squared
     */
    @Override
    public double getAccX() {
        return super.getAccelerationX().getValueAsDouble() * 9.8421;
    }

    /**
     * Gets the current acceleration along the Y-axis.
     *
     * @return the Y-axis acceleration in meters per second squared
     */
    @Override
    public double getAccY() {
        return super.getAccelerationY().getValueAsDouble() * 9.8421;
    }

    /**
     * Gets the current acceleration along the Z-axis.
     *
     * @return the Z-axis acceleration in meters per second squared
     */
    @Override
    public double getAccZ() {
        return super.getAccelerationZ().getValueAsDouble() * 9.8421;
    }

    /**
     * Resets the IMU's orientation to its default state.
     */
    @Override
    public void resetIMU() {
        super.reset();
    }

    /**
     * Sets the IMU's rotation using a {@link Rotation3d} object and resets yaw.
     *
     * @param rotation the new rotation to set as offset
     */
    @Override
    public void setRotation(Rotation3d rotation) {
        this.m_offsetRotation = rotation;
        super.setYaw(0.0);
    }

    /**
     * Sets the IMU's Z-axis rotation using a {@link Rotation2d} object and resets yaw.
     *
     * @param rotation the new Z-axis rotation to set as offset
     */
    @Override
    public void setRotation(Rotation2d rotation) {
        this.m_offsetRotation = new Rotation3d(m_offsetRotation.getX(), m_offsetRotation.getY(), rotation.getRadians());
        super.setYaw(0.0);
    }

    /**
     * Gets the current angular velocity around the Z-axis.
     *
     * @return the Z-axis angular velocity in degrees per second
     */
    public double getZRotationVelocity() {
        return super.getAngularVelocityZDevice().getValueAsDouble();
    }

    /**
     * Gets the current angular velocity around the X-axis.
     *
     * @return the X-axis angular velocity in degrees per second
     */
    public double getXRotationVelocity() {
        return super.getAngularVelocityXDevice().getValueAsDouble();
    }

    /**
     * Gets the current angular velocity around the Y-axis.
     *
     * @return the Y-axis angular velocity in degrees per second
     */
    public double getYRotationVelocity() {
        return super.getAngularVelocityYDevice().getValueAsDouble();
    }

}