package frc.excalib.swerve.swerve_utils;

import java.util.function.DoubleSupplier;

public record SwerveSpecs(double maxVelocity, double maxOmegaRadPerSec, double maxAcc,
                          DoubleSupplier maxFrontAcc, DoubleSupplier maxSideAcc, double maxForwardAcc) {
    public SwerveSpecs(double maxVelocity, double maxOmegaRadPerSec, double maxAcc) {
        this(maxVelocity, maxOmegaRadPerSec, maxAcc, () -> 10, () -> 10, 10);
    }

    public SwerveSpecs(double maxVelocity, double maxOmegaRadPerSec, double maxAcc,
                       double maxFrontAcc, double maxSideAcc, double maxForwardAcc) {
        this(maxVelocity, maxOmegaRadPerSec, maxAcc, () -> maxFrontAcc, () -> maxSideAcc, maxForwardAcc);
    }
}
