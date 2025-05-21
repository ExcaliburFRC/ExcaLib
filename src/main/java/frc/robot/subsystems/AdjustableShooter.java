package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.gains.GenericFF;
import frc.excalib.control.motor.controllers.SparkFlexMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motorType.MotorGroup;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanism.PositionMechanism;
import frc.excalib.mechanism.VelocityMechanism;

import java.util.function.DoubleSupplier;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.CannonConstants.*;

public class AdjustableShooter extends SubsystemBase{
    // Subsystem Hardware components
    private final SparkFlexMotor flywheelMotorA, flywheelMotorB;
    private final TalonFXMotor hoodMotor;

    // Subsystem mechanisms
    private final PositionMechanism hood;
    private final VelocityMechanism shooter;

    public AdjustableShooter() {
        // initialize hardware
        this.flywheelMotorA = new SparkFlexMotor(1, kBrushless);
        this.flywheelMotorB = new SparkFlexMotor(2, kBrushless);

        this.hoodMotor = new TalonFXMotor(3);

        // reverse one of the motors in the gearbox
        this.flywheelMotorB.setInverted(DirectionState.REVERSE);

        // setup shooterMotors conversion factors
        MotorGroup shooterMotors = new MotorGroup(flywheelMotorA, flywheelMotorB);
        shooterMotors.setPositionConversionFactor(TURRET_POSITION_CONVERSION_FACTOR);
        shooterMotors.setVelocityConversionFactor(TURRET_VELOCITY_CONVERSION_FACTOR);

        // create turret and arm mechanisms
        this.hood = new PositionMechanism(hoodMotor, TURRET_GAINS)
                .withTolerance(TURRET_TOLERANCE).withLimit(TURRET_SOFT_LIMIT)
                .withFeedforwardCalculator((position, velocity)-> TURRET_GAINS.ks * Math.signum(velocity));

        this.shooter = new VelocityMechanism(shooterMotors, ARM_GAINS)
                .withTolerance(ARM_TOLERANCE)
                .withFeedforwardCalculator((position, velocity)-> ARM_GAINS.applyGains(new GenericFF.SimpleFF()).calculate(velocity));

        setDefaultCommand(setShooterStateCommand(() -> 0, ()-> 0));
    }

    private Command moveHoodToPositionCommand(DoubleSupplier position) {
        return this.hood.setPositionCommand(position, new TrapezoidProfile(ARM_CONSTRAINTS));
    }

    private Command setShooterVelocityCommand(DoubleSupplier velocity) {
        return this.shooter.setVelocityCommand(velocity);
    }

    public Command setShooterStateCommand(DoubleSupplier hoodPosition, DoubleSupplier velocity) {
        return new ParallelCommandGroup(
                new RunCommand(()-> {}, this),
                moveHoodToPositionCommand(hoodPosition),
                setShooterVelocityCommand(velocity)
        );
    }

    public double getHoodPosition() {
        return this.hood.logMechanismPosition();
    }

    public double getShooterVelocity() {
        return this.shooter.logMechanismVelocity();
    }
}
