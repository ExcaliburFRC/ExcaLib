package frc.excalib.control.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface IMU {
    Rotation2d getZRotation();

    Rotation2d getXRotation();

    Rotation2d getYRotation();

    double getAccX();

    double getAccY();

    double getAccZ();

    void resetIMU();

    void setRotation(Rotation3d rotation);
    void setRotation(Rotation2d rotation);
}
