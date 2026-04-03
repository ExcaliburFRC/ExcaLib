package frc.excalib.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import java.util.function.DoubleSupplier;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final SimMotor driveMotor;
    private final SimMotor turnMotor;

    private final FlywheelSim driveSim;
    private final FlywheelSim turnSim;

    private double driveSimPositionRaw = 0.0;  // Motor shaft rotations (for SimMotor)
    private double turnSimPositionRaw = 0.0;   // Motor shaft rotations (for SimMotor)

    // Track the OUTPUT SHAFT angle in radians (like a real CANcoder would)
    private double turnOutputAngleRad = 0.0;

    private static final double DRIVE_GEAR_RATIO = 5.27;
    private static final double TURN_GEAR_RATIO = 26.090909;




    public SwerveModuleIOSim(int driveId, int turnId) {
        this.driveMotor = new SimMotor(driveId);
        this.turnMotor = new SimMotor(turnId);

        // Drive Sim: Kraken X60, MK5N R3
        // MOI = (robotMass / 4) * wheelRadius^2 = (53.16/4) * 0.0508^2 ≈ 0.034 kg·m²
        // NOTE: FlywheelSim(plant, motor, ...measurementStdDevs) — do NOT pass gearing here!
        //       The gearing is already baked into the LinearSystem from createFlywheelSystem.
        this.driveSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.034, DRIVE_GEAR_RATIO),
                DCMotor.getKrakenX60(1)
        );

        // Turn Sim: Kraken X60, 26.09:1 steering ratio
        this.turnSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, TURN_GEAR_RATIO),
                DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double driveVolts = driveMotor.getVoltage();
        double turnVolts = turnMotor.getVoltage();

        // === BRAKE MODE SIMULATION ===
        // Real Kraken in BRAKE mode shorts motor windings → near-instant stop.
        // If voltage is ~0, immediately kill all velocity. This prevents the
        // self-sustaining oscillation where residual velocity drifts position,
        // which triggers PID corrections, which creates more velocity.
        if (Math.abs(driveVolts) < 0.01) {
            driveSim.setState(edu.wpi.first.math.VecBuilder.fill(0.0));
        }
        if (Math.abs(turnVolts) < 0.01) {
            turnSim.setState(edu.wpi.first.math.VecBuilder.fill(0.0));
        }

        // Step physics
        driveSim.setInputVoltage(driveVolts);
        driveSim.update(0.020);

        turnSim.setInputVoltage(turnVolts);
        turnSim.update(0.020);

        // --- HEAL NaNs ---
        if (Double.isNaN(driveSim.getAngularVelocityRadPerSec())) {
            driveSim.setState(edu.wpi.first.math.VecBuilder.fill(0.0));
            driveSimPositionRaw = 0.0;
        }
        if (Double.isNaN(turnSim.getAngularVelocityRadPerSec())) {
            turnSim.setState(edu.wpi.first.math.VecBuilder.fill(0.0));
            turnSimPositionRaw = 0.0;
            turnOutputAngleRad = 0.0;
        }

        // --- OUTPUT SHAFT values from FlywheelSim ---
        double driveOutputRadPerSec = driveSim.getAngularVelocityRadPerSec();
        double turnOutputRadPerSec = turnSim.getAngularVelocityRadPerSec();

        // --- MOTOR SHAFT values (multiply by gear ratio) ---
        double driveMotorRotPerSec = (driveOutputRadPerSec / (2 * Math.PI)) * DRIVE_GEAR_RATIO;
        double turnMotorRotPerSec = (turnOutputRadPerSec / (2 * Math.PI)) * TURN_GEAR_RATIO;

        // Accumulate motor shaft rotations
        driveSimPositionRaw += driveMotorRotPerSec * 0.020;
        turnSimPositionRaw += turnMotorRotPerSec * 0.020;

        // Track output shaft angle in radians (like a real CANcoder)
        turnOutputAngleRad += turnOutputRadPerSec * 0.020;

        // Update SimMotor internal state (motor shaft units)
        driveMotor.setSimulatedState(driveSimPositionRaw, driveMotorRotPerSec, driveSim.getCurrentDrawAmps());
        turnMotor.setSimulatedState(turnSimPositionRaw, turnMotorRotPerSec, turnSim.getCurrentDrawAmps());

        // --- Populate IO inputs ---
        inputs.drivePositionConverted = driveMotor.getMotorPosition();
        inputs.driveVelocityConverted = driveMotor.getMotorVelocity();
        inputs.driveAppliedVolts = driveMotor.getVoltage();
        inputs.driveCurrentAmps = driveMotor.getCurrent();

        inputs.turnAbsolutePosition = turnMotor.getMotorPosition();
        inputs.turnPositionConverted = turnMotor.getMotorPosition();
        inputs.turnVelocityConverted = turnMotor.getMotorVelocity();
        inputs.turnAppliedVolts = turnMotor.getVoltage();
        inputs.turnCurrentAmps = turnMotor.getCurrent();

        // === DIAGNOSTIC PRINTS (Module 0 only, ~2% rate = ~1/sec) ===
        if (driveMotor.getDeviceID() == 20 && Math.random() < 0.02) {
            System.out.println("===== MODULE 0 (FL) SIM DIAGNOSTICS =====");
            System.out.printf("  DRIVE Voltage In:       %.3f V%n", driveVolts);
            System.out.printf("  DRIVE Output:           %.3f rad/s%n", driveOutputRadPerSec);
            System.out.printf("  DRIVE Motor Shaft:      %.3f rot/s -> getMotorVelocity: %.3f m/s%n", driveMotorRotPerSec, driveMotor.getMotorVelocity());
            System.out.printf("  TURN  Voltage In:       %.3f V%n", turnVolts);
            System.out.printf("  TURN  Output:           %.3f rad/s%n", turnOutputRadPerSec);
            System.out.printf("  TURN  OutputAngle:      %.3f rad (%.1f deg)%n", turnOutputAngleRad, Math.toDegrees(turnOutputAngleRad));
            System.out.printf("  TURN  MotorPos Raw:     %.4f rot -> getMotorPosition: %.4f%n", turnSimPositionRaw, turnMotor.getMotorPosition());
            System.out.println("==========================================");
        }
    }

    @Override
    public Motor getDriveMotor() {
        return driveMotor;
    }

    @Override
    public Motor getTurnMotor() {
        return turnMotor;
    }

    @Override
    public DoubleSupplier getAbsoluteEncoder() {
        // Return the OUTPUT SHAFT angle in radians, just like a real CANcoder would
        // (after conversion). The Turret PID and ContinuousSoftLimit expect radians.
        return () -> turnOutputAngleRad;
    }
}
