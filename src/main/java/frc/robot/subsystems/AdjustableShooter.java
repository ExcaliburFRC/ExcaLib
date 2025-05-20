package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.GenericFF;
import frc.excalib.control.motor.controllers.SparkFlexMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motorType.MotorGroup;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanism.ControlPositionMechanism;
import frc.excalib.mechanism.ControlVelocityMechanism;

import java.util.function.DoubleSupplier;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.CannonConstants.*;

public class AdjustableShooter extends SubsystemBase{
    // Subsystem Hardware components
    private final SparkFlexMotor flywheelMotorA, flywhellMotorB;
    private final TalonFXMotor hoodMotor;

    // Subsystem mechanisms
    private final ControlPositionMechanism hood;
    private final ControlVelocityMechanism shooter;

    public AdjustableShooter() {
        // initialize hardware
        this.flywheelMotorA = new SparkFlexMotor(1, kBrushless);
        this.flywhellMotorB = new SparkFlexMotor(2, kBrushless);

        this.hoodMotor = new TalonFXMotor(3);

        // reverse one of the motors in the gearbox
        this.flywhellMotorB.setInverted(DirectionState.REVERSE);

        // setup shooterMotors conversion factors
        MotorGroup shooterMotors = new MotorGroup(flywheelMotorA, flywhellMotorB);
        shooterMotors.setPositionConversionFactor(TURRET_POSITION_CONVERSION_FACTOR);
        shooterMotors.setVelocityConversionFactor(TURRET_VELOCITY_CONVERSION_FACTOR);

        // create turret and arm mechanisms
        this.hood = new ControlPositionMechanism(hoodMotor, TURRET_GAINS, TURRET_TOLERANCE,
                (position, velocity)-> TURRET_GAINS.ks * Math.signum(velocity));
        this.shooter = new ControlVelocityMechanism(shooterMotors, ARM_GAINS, ARM_TOLERANCE,
                (position, velocity)-> ARM_GAINS.applyGains(new GenericFF.SimpleFF()).calculate(velocity));

        this.hood.withLimits(TURRET_SOFT_LIMIT);

        setDefaultCommand(moveHoodToPositionCommand(() -> 0));
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
