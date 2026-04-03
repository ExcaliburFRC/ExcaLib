package frc.excalib.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
        table.put("yawPositionRad", yawPositionRad);
        table.put("pitchPositionRad", pitchPositionRad);
        table.put("rollPositionRad", rollPositionRad);
        table.put("accelX", accelX);
        table.put("accelY", accelY);
        table.put("accelZ", accelZ);
    }

    @Override
    public void fromLog(LogTable table) {
        yawPositionRad = table.get("yawPositionRad", yawPositionRad);
        pitchPositionRad = table.get("pitchPositionRad", pitchPositionRad);
        rollPositionRad = table.get("rollPositionRad", rollPositionRad);
        accelX = table.get("accelX", accelX);
        accelY = table.get("accelY", accelY);
        accelZ = table.get("accelZ", accelZ);
    }
}
