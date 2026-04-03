package frc.excalib.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
        table.put("drivePositionConverted", drivePositionConverted);
        table.put("driveVelocityConverted", driveVelocityConverted);
        table.put("driveAppliedVolts", driveAppliedVolts);
        table.put("driveCurrentAmps", driveCurrentAmps);

        table.put("turnAbsolutePosition", turnAbsolutePosition);
        table.put("turnPositionConverted", turnPositionConverted);
        table.put("turnVelocityConverted", turnVelocityConverted);
        table.put("turnAppliedVolts", turnAppliedVolts);
        table.put("turnCurrentAmps", turnCurrentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
        drivePositionConverted = table.get("drivePositionConverted", drivePositionConverted);
        driveVelocityConverted = table.get("driveVelocityConverted", driveVelocityConverted);
        driveAppliedVolts = table.get("driveAppliedVolts", driveAppliedVolts);
        driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps);

        turnAbsolutePosition = table.get("turnAbsolutePosition", turnAbsolutePosition);
        turnPositionConverted = table.get("turnPositionConverted", turnPositionConverted);
        turnVelocityConverted = table.get("turnVelocityConverted", turnVelocityConverted);
        turnAppliedVolts = table.get("turnAppliedVolts", turnAppliedVolts);
        turnCurrentAmps = table.get("turnCurrentAmps", turnCurrentAmps);
    }
}
