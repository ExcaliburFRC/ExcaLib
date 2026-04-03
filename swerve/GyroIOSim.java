package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.excalib.control.imu.IMU;

import java.util.function.Supplier;

/**
 * Simulated Gyro that integrates omega from swerve module states
 * to produce accurate yaw tracking in simulation.
 */
public class GyroIOSim implements GyroIO {

    private final SwerveDriveKinematics kinematics;
    private final Supplier<SwerveModuleState[]> statesSupplier;

    private double simulatedYawRad = 0.0;

    public GyroIOSim(SwerveDriveKinematics kinematics, Supplier<SwerveModuleState[]> statesSupplier) {
        this.kinematics = kinematics;
        this.statesSupplier = statesSupplier;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(statesSupplier.get());

        // Integrate omega to get yaw (Euler integration at 20ms)
        simulatedYawRad += speeds.omegaRadiansPerSecond * 0.020;

        inputs.yawPositionRad = simulatedYawRad;
        inputs.pitchPositionRad = 0.0;
        inputs.rollPositionRad = 0.0;
        inputs.accelX = 0.0;
        inputs.accelY = 0.0;
        inputs.accelZ = 0.0;
    }

    @Override
    public IMU getIMU() {
        return new IMU() {
            @Override
            public Rotation2d getZRotation() {
                return Rotation2d.fromRadians(simulatedYawRad);
            }

            @Override
            public Rotation2d getXRotation() {
                return Rotation2d.fromRadians(0.0);
            }

            @Override
            public Rotation2d getYRotation() {
                return Rotation2d.fromRadians(0.0);
            }

            @Override
            public double getAccX() { return 0.0; }

            @Override
            public double getAccY() { return 0.0; }

            @Override
            public double getAccZ() { return 0.0; }

            @Override
            public void resetIMU() {
                simulatedYawRad = 0.0;
            }

            @Override
            public void setRotation(Rotation3d rotation) {
                simulatedYawRad = rotation.getZ();
            }

            @Override
            public void setRotation(Rotation2d rotation) {
                simulatedYawRad = rotation.getRadians();
            }
        };
    }
}
