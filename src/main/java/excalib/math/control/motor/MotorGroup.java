package excalib.math.control.motor;

import excalib.math.control.motor.motor_specs.IdleState;
import excalib.math.control.motor.motor_specs.DirectionState;

public class MotorGroup implements Motor {

    private Motor[] motors;

    public MotorGroup(Motor... motors) {
        this.motors = motors;
    }

    @Override
    public void stopMotor() {
        for (Motor motor : motors) {
            motor.stopMotor();
        }
    }

    @Override
    public void setVoltage(double voltage) {
        for (Motor motor : motors) {
            motor.setVoltage(voltage);
        }
    }

    @Override
    public void setPercentage(double percentage) {
        for (Motor motor : motors) motor.setPercentage(percentage);
    }

    @Override
    public void setFollower(int mainMotorID) {
        for (Motor motor : motors) {
            if (motor.getDeviceID() != mainMotorID) motor.setFollower(mainMotorID);
        }
    }

    @Override
    public void setIdleMode(IdleState idleMode) {
        for (Motor motor : motors) {
            motor.setIdleMode(idleMode);
        }
    }

    @Override
    public int getDeviceID() {
        return motors.length > 0 ? motors[0].getDeviceID() : -1;
    }

    @Override
    public double getMotorPosition() {
        double totalPosition = 0;
        for (Motor motor : motors) {
            totalPosition += motor.getMotorPosition();
        }
        return totalPosition / motors.length;
    }

    @Override
    public double getMotorVelocity() {
        double totalVelocity = 0;
        for (Motor motor : motors) {
            totalVelocity += motor.getMotorVelocity();
        }
        return totalVelocity / motors.length;
    }

    @Override
    public double getCurrent() {
        double totalCurrent = 0;
        for (Motor motor : motors) {
            totalCurrent += motor.getCurrent();
        }
        return totalCurrent / motors.length;
    }

    @Override
    public double getVoltage() {
        double totalVoltage = 0;
        for (Motor motor : motors) {
            totalVoltage += motor.getVoltage();
        }
        return totalVoltage / motors.length;
    }

    @Override
    public double getTemperature() {
        double maxTemperature = Double.MIN_VALUE;
        for (Motor motor : motors) {
            double motorTemperature = motor.getTemperature();
            if (motorTemperature > maxTemperature) {
                maxTemperature = motorTemperature;
            }
        }
        return maxTemperature;
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        for (Motor motor : motors) {
            motor.setSoftLimit(directionState, limit);
        }
    }

    @Override
    public void setInverted(boolean mode) {
        for (Motor motor : motors) {
            motor.setInverted(mode);
        }
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        for (Motor motor : motors) {
            motor.setPositionConversionFactor(conversionFactor);
        }
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        for (Motor motor : motors) {
            motor.setVelocityConversionFactor(conversionFactor);
        }
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        for (Motor motor : motors) {
            motor.setCurrentLimit(stallLimit, freeLimit);
        }
    }
}
