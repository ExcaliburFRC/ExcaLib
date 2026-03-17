package frc.excalib.control.motor.controllers;

import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

public class MotorGroup implements Motor {

    private Motor[] m_motors;

    public MotorGroup(Motor... motors) {
        this.m_motors = motors;
    }

    @Override
    public void stopMotor() {
        for (Motor motor : m_motors) {
            motor.stopMotor();
        }
    }

    @Override
    public void setMotorVoltage(double voltage) {
        for (Motor motor : m_motors) {
            motor.setMotorVoltage(voltage);
        }
    }

    @Override
    public void setPercentage(double percentage) {
        for (Motor motor : m_motors) motor.setPercentage(percentage);
    }

    @Override
    public void setFollower(int mainMotorID) {
        for (Motor motor : m_motors) {
            if (motor.getDeviceID() != mainMotorID) motor.setFollower(mainMotorID);
        }
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        for (Motor motor : m_motors) {
            motor.setIdleState(idleMode);
        }
    }

    @Override
    public IdleState getIdleState() {
        return m_motors[0].getIdleState();
    }

    @Override
    public int getDeviceID() {
        return m_motors.length > 0 ? m_motors[0].getDeviceID() : -1;
    }

    @Override
    public double getMotorPosition() {
        double totalPosition = 0;
        for (Motor motor : m_motors) {
            totalPosition += motor.getMotorPosition();
        }
        return totalPosition / m_motors.length;
    }

    @Override
    public double getMotorVelocity() {
        double totalVelocity = 0;
        for (Motor motor : m_motors) {
            totalVelocity += motor.getMotorVelocity();
        }
        return totalVelocity / m_motors.length;
    }

    @Override
    public double getCurrent() {
        double totalCurrent = 0;
        for (Motor motor : m_motors) {
            totalCurrent += motor.getCurrent();
        }
        return totalCurrent / m_motors.length;
    }

    @Override
    public double getVoltage() {
        double totalVoltage = 0;
        for (Motor motor : m_motors) {
            totalVoltage += motor.getVoltage();
        }
        return totalVoltage / m_motors.length;
    }

    @Override
    public double getTemperature() {
        double maxTemperature = Double.MIN_VALUE;
        for (Motor motor : m_motors) {
            double motorTemperature = motor.getTemperature();
            if (motorTemperature > maxTemperature) {
                maxTemperature = motorTemperature;
            }
        }
        return maxTemperature;
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        for (Motor motor : m_motors) {
            motor.setSoftLimit(directionState, limit);
        }
    }

    @Override
    public void setInverted(DirectionState mode) {
        for (Motor motor : m_motors) {
            motor.setInverted(mode);
        }
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        for (Motor motor : m_motors) {
            motor.setPositionConversionFactor(conversionFactor);
        }
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        for (Motor motor : m_motors) {
            motor.setVelocityConversionFactor(conversionFactor);
        }
    }

    @Override
    public void setCurrentLimit(int stator, int supply) {
        for (Motor motor : m_motors) {
            motor.setCurrentLimit(stator, supply);
        }
    }

    @Override
    public void setMotorPosition(double position) {
        for (Motor motor : m_motors) {
            motor.setMotorPosition(position);
        }
    }
}
