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

public class SwerveConfigurationUtils {
    private static void setupDriveMotors(Motor[] driveMotors, int stallLimit, int freeLimit,
                                              DirectionState directionState, IdleState idleState,
                                              SwerveModuleType moduleType) {
        for (Motor driveMotor : driveMotors) {
            driveMotor.setInverted(directionState);
            driveMotor.setIdleState(idleState);
            driveMotor.setPositionConversionFactor(moduleType.kPositionConversionFactor);
            driveMotor.setVelocityConversionFactor(moduleType.kVelocityConversionFactor);
            driveMotor.setCurrentLimit(stallLimit, freeLimit);
        }
    }

    private static void setupSteeringMotors(Motor[] driveMotors, int stallLimit, int freeLimit,
                                            DirectionState directionState, IdleState idleState,
                                            SwerveModuleType moduleType) {
        for (Motor driveMotor : driveMotors) {
            driveMotor.setInverted(directionState);
            driveMotor.setIdleState(idleState);
            driveMotor.setVelocityConversionFactor(moduleType.kVelocityConversionFactor);
            driveMotor.setCurrentLimit(stallLimit, freeLimit);
        }
    }

    public static FlyWheel[] setupDrivingMechanisms(Motor[] driveMotors, int stallLimit, int freeLimit,
                                                    DirectionState directionState, IdleState idleState,
                                                    SwerveModuleType moduleType, Gains driveModuleGains,
                                                    double maxModuleAcceleration, double maxModuleJerk) {
        setupDriveMotors(driveMotors, stallLimit, freeLimit, directionState, idleState, moduleType);

        FlyWheel[] drivingMechanisms = new FlyWheel[4];
        FlyWheel frontLeftWheel = new FlyWheel(driveMotors[0], maxModuleAcceleration, maxModuleJerk, driveModuleGains);

        for (int i = 1; i < drivingMechanisms.length; i++) {
            drivingMechanisms[i] = new FlyWheel(driveMotors[i], frontLeftWheel);
        }

        return drivingMechanisms;
    }

    public static Turret[] setupSteeringMechanisms(Motor[] steeringMotors, int stallLimit, int freeLimit,
                                                   DirectionState directionState, IdleState idleState,
                                                   SwerveModuleType moduleType, Gains steeringModuleGains,
                                                   DoubleSupplier[] steeringModulesPositionSuppliers, double steeringPIDtolerance) {
        setupSteeringMotors(steeringMotors, stallLimit, freeLimit, directionState, idleState, moduleType);

        Turret[] steeringMechanisms = new Turret[4];
        Turret frontLeftSteeringMechanism = new Turret(steeringMotors[0], steeringModuleGains, steeringPIDtolerance, steeringModulesPositionSuppliers[0]);

        for (int i = 1; i < steeringMechanisms.length; i++) {
            steeringMechanisms[i] = new Turret(steeringMotors[i], steeringModulesPositionSuppliers[i], frontLeftSteeringMechanism);
        }

        return steeringMechanisms;
    }

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
