package frc.excalib.swerve;

import com.ctre.phoenix6.CANBus;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import java.util.function.DoubleSupplier;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final TalonFXMotor driveMotor;
    private final TalonFXMotor turnMotor;
    private final DoubleSupplier absoluteEncoder;

    public SwerveModuleIOTalonFX(int driveId, int turnId, CANBus canbus, DoubleSupplier absoluteEncoder) {
        this.driveMotor = new TalonFXMotor(driveId, canbus);
        this.turnMotor = new TalonFXMotor(turnId, canbus);
        this.absoluteEncoder = absoluteEncoder;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionConverted = driveMotor.getMotorPosition();
        inputs.driveVelocityConverted = driveMotor.getMotorVelocity();
        inputs.driveAppliedVolts = driveMotor.getVoltage();
        inputs.driveCurrentAmps = driveMotor.getCurrent();

        inputs.turnAbsolutePosition = absoluteEncoder.getAsDouble();
        inputs.turnPositionConverted = turnMotor.getMotorPosition();
        inputs.turnVelocityConverted = turnMotor.getMotorVelocity();
        inputs.turnAppliedVolts = turnMotor.getVoltage();
        inputs.turnCurrentAmps = turnMotor.getCurrent();
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
        return absoluteEncoder;
    }
}
