package excalib.swerve;

public class SwerveConfigurator {
    public static void configureSwerve(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, KinematicsConfig kinematicsConfig) {
        Swerve.m_instance = new Swerve(new ModulesHolder(frontLeft, frontRight, backLeft, backRight), kinematicsConfig);
    }

    public class KinematicsConfig {
        public final double MAX_SKID_ACCELERATION_MPSS;
        public final double MAX_FRONT_ACCELERATION;
        public final double MAX_SIDE_ACCELERATION;
        public final double MAX_FORWARD_ACCELERATION_MPSS;
        public final double MAX_VELOCITY_MPS;

        public KinematicsConfig(double maxFrontAcc, double maxSideAcc, double maxSkidAcc, double maxForwardAcc, double maxVel) {
            this.MAX_FRONT_ACCELERATION = maxFrontAcc;
            this.MAX_SKID_ACCELERATION_MPSS = maxSkidAcc;
            this.MAX_SIDE_ACCELERATION = maxSideAcc;
            this.MAX_FORWARD_ACCELERATION_MPSS = maxForwardAcc;
            this.MAX_VELOCITY_MPS = maxVel;
        }
    }
}