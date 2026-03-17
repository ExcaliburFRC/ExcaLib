package frc.excalib.control.imu;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Pigeon extends Pigeon2 implements IMU {
    private Rotation3d m_offsetRotation;

    public Pigeon(int deviceId, Rotation3d offsetRotation) {
        super(deviceId);
        this.m_offsetRotation = offsetRotation;
    }

    public Pigeon(int deviceId, String canbus, Rotation3d offsetRotation) {
        super(deviceId, new CANBus(canbus));
        this.m_offsetRotation = offsetRotation;
    }

    @Override
    public Rotation2d getZRotation() {
        return super.getRotation2d().rotateBy(m_offsetRotation.toRotation2d());
    }

    @Override
    public Rotation2d getXRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getRoll().getValueAsDouble() + m_offsetRotation.getX()));
    }

    @Override
    public Rotation2d getYRotation() {
        return new Rotation2d(Units.degreesToRadians(super.getPitch().getValueAsDouble() + m_offsetRotation.getY()));
    }

    @Override
    public double getAccX() {
        return super.getAccelerationX().getValueAsDouble() * 9.8421;
    }

    @Override
    public double getAccY() {
        return super.getAccelerationY().getValueAsDouble() * 9.8421;
    }

    @Override
    public double getAccZ() {
        return super.getAccelerationZ().getValueAsDouble() * 9.8421;
    }

    @Override
    public void resetIMU() {
        super.reset();
    }

    @Override
    public void setRotation(Rotation3d rotation) {
        this.m_offsetRotation = rotation;
        super.setYaw(0.0);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        this.m_offsetRotation = new Rotation3d(m_offsetRotation.getX(), m_offsetRotation.getY(), rotation.getRadians());
        super.setYaw(0.0);
    }

    public double getZRotationVelocity() {
        return super.getAngularVelocityZDevice().getValueAsDouble();
    }

    public double getXRotationVelocity() {
        return super.getAngularVelocityXDevice().getValueAsDouble();
    }

    public double getYRotationVelocity() {
        return super.getAngularVelocityYDevice().getValueAsDouble();
    }

}
