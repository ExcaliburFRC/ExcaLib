package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.encoder.DutyCycleEncoder;
import frc.excalib.control.gains.GenericFF;
import frc.excalib.control.motor.Piston;
import frc.excalib.control.motor.controllers.SparkFlexMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motorType.MotorGroup;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanism.mechanisms.Arm;
import frc.excalib.mechanism.mechanisms.LinearExtension;
import frc.excalib.mechanism.mechanisms.Turret;

import java.util.function.DoubleSupplier;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.CannonConstants.*;

public class Cannon extends SubsystemBase {

    // Subsystem Hardware components
    private final SparkFlexMotor rotationMotorA, rotationMotorB;
    private final TalonFXMotor pitchMotor;
    private final Piston primer;

    private final DutyCycleEncoder angleEncoder;

    // Subsystem mechanisms
    private final Turret turret;
    private final Arm arm;

    public Cannon() {
        // initialize hardware
        this.rotationMotorA = new SparkFlexMotor(1, kBrushless);
        this.rotationMotorB = new SparkFlexMotor(2, kBrushless);

        this.pitchMotor = new TalonFXMotor(3);
        this.primer = new Piston(1, 2, PneumaticsModuleType.REVPH);

        this.angleEncoder = new DutyCycleEncoder(3);

        // reverse one of the motors in the gearbox
        this.rotationMotorB.setInverted(DirectionState.REVERSE);

        // set the encoder's offset and conversion factor
        this.angleEncoder.setPositionConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        this.angleEncoder.setPosition(ENCODER_OFFSET, true);

        // setup rotationMotors conversion factors
        MotorGroup rotationMotors = new MotorGroup(rotationMotorA, rotationMotorB);
        rotationMotors.setPositionConversionFactor(TURRET_POSITION_CONVERSION_FACTOR);
        rotationMotors.setVelocityConversionFactor(TURRET_VELOCITY_CONVERSION_FACTOR);

        // create turret and arm mechanisms
        this.turret = new Turret(rotationMotors, TURRET_SOFT_LIMIT, TURRET_GAINS, TURRET_CONSTRAINTS, TURRET_TOLERANCE);
        this.arm = new Arm(this.pitchMotor, ARM_GAINS, ARM_CONSTRAINTS, ARM_TOLERANCE);

        // set the arm mechanism to use an external encoder rather than the motor's built in
        this.arm.addExternalEncoder(this.angleEncoder);

        setDefaultCommand(moveToPositionCommand(() -> 0, () -> 0));
    }

    private Command moveToPositionCommand(DoubleSupplier rotation, DoubleSupplier pitch) {
        return new ParallelCommandGroup(
                new RunCommand(() -> {
                }, this), // add requirements to the command
                this.turret.setDynamicPositionCommand(rotation),
                this.arm.setDynamicPositionCommand(pitch)
        );
    }

    private Command shootCommand() {
        return new StartEndCommand(
                () -> this.primer.setPiston(DirectionState.FORWARD),
                () -> this.primer.setPiston(DirectionState.REVERSE)
        );
    }

    public Command shootToPositionCommand(DoubleSupplier rotation, DoubleSupplier pitch) {
        return moveToPositionCommand(rotation, pitch)
                .until(turret.atSetpointTrigger.and(arm.atSetpointTrigger))
                .andThen(shootCommand()
        );
    }

    public Command manualControlCommand(DoubleSupplier rotation, DoubleSupplier pitch, Trigger shoot) {
        return new ParallelCommandGroup(
                moveToPositionCommand(
                        () -> getCannonRotation() + rotation.getAsDouble(), // rotate up to 5 deg a sec
                        () -> getCannonPitch() + pitch.getAsDouble()), // pitch up to 5 deg a sec
                new WaitUntilCommand(shoot).andThen(shootCommand())
        );
    }


    public double getCannonRotation() {
        return this.turret.logMechanismPosition();
    }

    public double getCannonPitch() {
        return this.arm.logMechanismPosition();
    }
}
