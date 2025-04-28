package frc.excalib.control.imu;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import monologue.Annotations.Log;
import monologue.Logged;

import static com.studica.frc.AHRS.NavXComType.kMXP_SPI;

public class NavX extends AHRS implements IMU, Logged {
    private Rotation3d m_offsetRotation;

    public NavX(Rotation3d offsetRotation) {
        super(kMXP_SPI);
        this.m_offsetRotation = offsetRotation;
    }

    @Override
    public Rotation2d getZRotation() {
        return super.getRotation2d().rotateBy(m_offsetRotation.toRotation2d());
    }

    @Override
    public Rotation2d getXRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getRoll() + m_offsetRotation.getX()));
    }

    @Override
    public Rotation2d getYRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getPitch() + m_offsetRotation.getY()));
    }

    @Override
    @Log.NT
    public double getAccX() {
        return super.getRawAccelX();
    }

    @Override
    @Log.NT
    public double getAccY() {
        return super.getRawAccelY();
    }

    @Override
    @Log.NT
    public double getAccZ() {
        return super.getWorldLinearAccelZ();
    }

    @Override
    public void resetIMU() {
        super.reset();
    }

    @Override
    public void setRotation(Rotation3d rotation) {
        this.m_offsetRotation = rotation;
        super.setAngleAdjustment(0.0);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        this.m_offsetRotation = new Rotation3d(m_offsetRotation.getX(), m_offsetRotation.getY(), rotation.getRadians());
        super.setAngleAdjustment(0.0);
    }
}
