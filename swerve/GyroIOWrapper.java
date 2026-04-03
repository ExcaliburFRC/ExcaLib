package frc.excalib.swerve;

import frc.excalib.control.imu.IMU;

public class GyroIOWrapper implements GyroIO {
    private final IMU imu;

    public GyroIOWrapper(IMU imu) {
        this.imu = imu;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPositionRad = imu.getZRotation().getRadians();
        inputs.pitchPositionRad = imu.getYRotation().getRadians();
        inputs.rollPositionRad = imu.getXRotation().getRadians();
        inputs.accelX = imu.getAccX();
        inputs.accelY = imu.getAccY();
        inputs.accelZ = imu.getAccZ();
    }

    @Override
    public IMU getIMU() {
        return imu;
    }
}
