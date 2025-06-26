package frc.excalib.control.imu;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import static com.studica.frc.AHRS.NavXComType.kMXP_SPI;

/**
 * Implementation of the {@link IMU} interface using the NavX sensor.
 * Extends {@link AHRS} to provide orientation and acceleration data,
 * with support for rotation offsets and logging.
 */
@Logged
public class NavX extends AHRS implements IMU {
    /** The offset rotation applied to all orientation readings. */
    private Rotation3d m_offsetRotation;

    /**
     * Constructs a new NavX instance with a specified offset rotation.
     *
     * @param offsetRotation the initial offset rotation to apply
     */
    public NavX(Rotation3d offsetRotation) {
        super(kMXP_SPI);
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
        return new Rotation2d(Units.degreesToRadians(super.getRoll() + m_offsetRotation.getX()));
    }

    /**
     * Gets the current Y-axis rotation (pitch), adjusted by the offset.
     *
     * @return the Y-axis rotation as a {@link Rotation2d}
     */
    @Override
    public Rotation2d getYRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getPitch() + m_offsetRotation.getY()));
    }

    /**
     * Gets the current acceleration along the X-axis.
     *
     * @return the X-axis acceleration in meters per second squared
     */
    @Override
    @Logged
    public double getAccX() {
        return super.getRawAccelX();
    }

    /**
     * Gets the current acceleration along the Y-axis.
     *
     * @return the Y-axis acceleration in meters per second squared
     */
    @Override
    @Logged
    public double getAccY() {
        return super.getRawAccelY();
    }

    /**
     * Gets the current acceleration along the Z-axis.
     *
     * @return the Z-axis acceleration in meters per second squared
     */
    @Override
    @Logged
    public double getAccZ() {
        return super.getWorldLinearAccelZ();
    }

    /**
     * Resets the IMU's orientation to its default state.
     */
    @Override
    public void resetIMU() {
        super.reset();
    }

    /**
     * Sets the IMU's rotation using a {@link Rotation3d} object and resets angle adjustment.
     *
     * @param rotation the new rotation to set as offset
     */
    @Override
    public void setRotation(Rotation3d rotation) {
        this.m_offsetRotation = rotation;
        super.setAngleAdjustment(0.0);
    }

    /**
     * Sets the IMU's Z-axis rotation using a {@link Rotation2d} object and resets angle adjustment.
     *
     * @param rotation the new Z-axis rotation to set as offset
     */
    @Override
    public void setRotation(Rotation2d rotation) {
        this.m_offsetRotation = new Rotation3d(m_offsetRotation.getX(), m_offsetRotation.getY(), rotation.getRadians());
        super.setAngleAdjustment(0.0);
    }
}