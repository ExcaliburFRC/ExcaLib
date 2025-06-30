package frc.excalib.swerve.swerve_utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.turret.Turret;
import frc.excalib.swerve.SwerveModule;

import java.util.function.DoubleSupplier;

/**
 * A class that contains functions that simplifies the swerve configuration process
 */
public class SwerveConfigurationUtils {

    /**
     * A method to configure the drive motors
     *
     * @param driveMotors an array represents the drive motors
     * @param stallLimit the wanted stall limit for the motors
     * @param freeLimit the wanted free limit for the motors
     * @param directionState the wanted direction state for the motors
     * @param idleState the wanted idle state of the motors
     * @param moduleType the type of the module
     */
    private static void setupDriveMotors(Motor[] driveMotors, int stallLimit, int freeLimit,
                                              DirectionState directionState, IdleState idleState,
                                              SwerveModuleConfiguration moduleType) {
        for (Motor driveMotor : driveMotors) {
            driveMotor.setInverted(directionState);
            driveMotor.setIdleState(idleState);
            driveMotor.setPositionConversionFactor(moduleType.positionConversionFactor());
            driveMotor.setVelocityConversionFactor(moduleType.velocityConversionFactor());
            driveMotor.setCurrentLimit(stallLimit, freeLimit);
        }
    }

    /**
     * A method to configure the steering motors
     *
     * @param steeringMotors an array represents the steering motors
     * @param stallLimit the wanted stall limit for the motors
     * @param freeLimit the wanted free limit for the motors
     * @param directionState the wanted direction state for the motors
     * @param idleState the wanted idle state of the motors
     * @param moduleType the type of the module
     */
    private static void setupSteeringMotors(Motor[] steeringMotors, int stallLimit, int freeLimit,
                                            DirectionState directionState, IdleState idleState,
                                            SwerveModuleConfiguration moduleType) {
        for (Motor steeringMotor : steeringMotors) {
            steeringMotor.setInverted(directionState);
            steeringMotor.setIdleState(idleState);
            steeringMotor.setVelocityConversionFactor(moduleType.steeringVelocityConversionFactor());
            steeringMotor.setCurrentLimit(stallLimit, freeLimit);
        }
    }

    /**
     * A method to configure the fly wheels which represents the driving mechanism
     *
     * @param driveMotors an array represents the drive motors
     * @param stallLimit the wanted stall limit for the motors
     * @param freeLimit the wanted free limit for the motors
     * @param directionState the wanted direction state for the motors
     * @param idleState the wanted idle state of the motors
     * @param moduleType the type of the module
     * @param driveModuleGains the driving mechanisms gains
     * @param maxModuleAcceleration the max acceleration of the module
     * @param maxModuleJerk the max jerk of the module
     *
     * @return an array of fly wheels represents the driving mechanisms
     */
    public static FlyWheel[] setupDrivingMechanisms(Motor[] driveMotors, int stallLimit, int freeLimit,
                                                    DirectionState directionState, IdleState idleState,
                                                    SwerveModuleConfiguration moduleType, Gains driveModuleGains,
                                                    double maxModuleAcceleration, double maxModuleJerk) {
        setupDriveMotors(driveMotors, stallLimit, freeLimit, directionState, idleState, moduleType);

        FlyWheel[] drivingMechanisms = new FlyWheel[4];
        drivingMechanisms[0] = new FlyWheel(driveMotors[0], maxModuleAcceleration, maxModuleJerk, driveModuleGains);

        for (int i = 1; i < drivingMechanisms.length; i++) {
            drivingMechanisms[i] = new FlyWheel(driveMotors[i], drivingMechanisms[0]);
        }

        return drivingMechanisms;
    }

    /**
     * A method to configure the fly wheels which represents the driving mechanism
     *
     * @param steeringMotors an array represents the steering motors
     * @param stallLimit the wanted stall limit for the motors
     * @param freeLimit the wanted free limit for the motors
     * @param directionState the wanted direction state for the motors
     * @param idleState the wanted idle state of the motors
     * @param moduleType the type of the module
     * @param steeringModuleGains the steering mechanisms gains
     * @param steeringModulesPositionSuppliers the supplier of the module position
     * @param steeringPIDtolerance the tolerance for the PID of the steering mechanisms
     *
     * @return an array of turrets represents the steering mechanisms
     */
    public static Turret[] setupSteeringMechanisms(Motor[] steeringMotors, int stallLimit, int freeLimit,
                                                   DirectionState directionState, IdleState idleState,
                                                   SwerveModuleConfiguration moduleType, Gains steeringModuleGains,
                                                   DoubleSupplier[] steeringModulesPositionSuppliers, double steeringPIDtolerance) {
        setupSteeringMotors(steeringMotors, stallLimit, freeLimit, directionState, idleState, moduleType);

        Turret[] steeringMechanisms = new Turret[4];
        steeringMechanisms[0] = new Turret(steeringMotors[0], steeringModuleGains, steeringPIDtolerance, steeringModulesPositionSuppliers[0]);

        for (int i = 1; i < steeringMechanisms.length; i++) {
            steeringMechanisms[i] = new Turret(steeringMotors[i], steeringModulesPositionSuppliers[i], steeringMechanisms[0]);
        }

        return steeringMechanisms;
    }

    /**
     * A function that initializes the swerve modules
     *
     * @param driveWheels an array of fly wheels represents the driving mechanisms
     * @param steeringMechanisms an array of turrets represents the steering mechanisms
     * @param moduleLocations the locations of the modules relative to the center of the robot
     * @param maxModuleVelocity the max velocity of the modules
     *
     * @return an array of SwerveModules
     */
    public static SwerveModule[] setupSwerveModules(FlyWheel[] driveWheels, Turret[] steeringMechanisms,
                                                    Translation2d[] moduleLocations, double maxModuleVelocity) {
        SwerveModule[] swerveModules = new SwerveModule[4];

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i] = new SwerveModule(
                    driveWheels[i], steeringMechanisms[i],
                    moduleLocations[i], maxModuleVelocity
            );
        }

        return swerveModules;
    }

    /**
     * A function that initializes the swerve modules
     *
     * @param driveWheels an array of fly wheels represents the driving mechanisms
     * @param steeringMechanisms an array of turrets represents the steering mechanisms
     * @param moduleLocations the locations of the modules relative to the center of the robot
     * @param maxModulesVelocities an array of the max velocities of each module
     *
     * @return an array of SwerveModules
     */
    public static SwerveModule[] setupSwerveModules(FlyWheel[] driveWheels, Turret[] steeringMechanisms,
                                                    Translation2d[] moduleLocations, double[] maxModulesVelocities) {
        SwerveModule[] swerveModules = new SwerveModule[4];

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i] = new SwerveModule(
                    driveWheels[i], steeringMechanisms[i],
                    moduleLocations[i], maxModulesVelocities[i]
            );
        }

        return swerveModules;
    }
}
