package frc.excalib.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveInputsAutoLogged implements LoggableInputs {
    public double poseX = 0.0;
    public double poseY = 0.0;
    public double poseTheta = 0.0;
    
    public double measuredVx = 0.0;
    public double measuredVy = 0.0;
    public double measuredOmega = 0.0;
    
    public double desiredVx = 0.0;
    public double desiredVy = 0.0;
    public double desiredOmega = 0.0;

    @Override
    public void toLog(LogTable table) {
        table.put("poseX", poseX);
        table.put("poseY", poseY);
        table.put("poseTheta", poseTheta);
        table.put("measuredVx", measuredVx);
        table.put("measuredVy", measuredVy);
        table.put("measuredOmega", measuredOmega);
        table.put("desiredVx", desiredVx);
        table.put("desiredVy", desiredVy);
        table.put("desiredOmega", desiredOmega);
    }

    @Override
    public void fromLog(LogTable table) {
        poseX = table.get("poseX", poseX);
        poseY = table.get("poseY", poseY);
        poseTheta = table.get("poseTheta", poseTheta);
        measuredVx = table.get("measuredVx", measuredVx);
        measuredVy = table.get("measuredVy", measuredVy);
        measuredOmega = table.get("measuredOmega", measuredOmega);
        desiredVx = table.get("desiredVx", desiredVx);
        desiredVy = table.get("desiredVy", desiredVy);
        desiredOmega = table.get("desiredOmega", desiredOmega);
    }
}
