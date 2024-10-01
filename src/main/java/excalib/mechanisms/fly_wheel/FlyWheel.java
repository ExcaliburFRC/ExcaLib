package excalib.mechanisms.fly_wheel;

import excalib.math.control.motor.Motor;
import excalib.mechanisms.Mechanism;

public class FlyWheel extends Mechanism {
    private final Motor m_motor;
    private final double MPS_TO_RPM;

    /**
     * creates a new FlyWheel object. the motor should
     * come with the velocity conversion factor to be rpm
     *
     * @param motor  the motor/ motor group of the FlyWheel
     * @param radius the radius of the FlyWheel in meters
     */
    public FlyWheel(Motor motor, double radius) {
        m_motor = motor;
        MPS_TO_RPM = 60 / (radius * 2 * Math.PI);
    }

    public void setOutput(double output, Unit controlType) {
        switch (controlType) {
            case MPS -> m_motor.setVoltage(output * MPS_TO_RPM);
            case RPM -> m_motor.setVoltage(output);
            case VOLTAGE -> m_motor.setVoltage(output);
            case PERCENTAGE -> m_motor.setPercentage(output);
        }
    }

    public void stopWheel() {
        m_motor.stopMotor();
    }

    public enum Unit {
        MPS,
        RPM,
        VOLTAGE,
        PERCENTAGE;
    }

    public double getVelocityMPS() {
        return m_motor.getMotorVelocity() / MPS_TO_RPM;
    }

    public double getVelocityRPM() {
        return m_motor.getMotorVelocity();
    }
    public double getVoltage(){
        return m_motor.getVoltage();
    }
}
