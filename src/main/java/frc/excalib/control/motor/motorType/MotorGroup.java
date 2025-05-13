package frc.excalib.control.motor.motorType;

import frc.excalib.control.motor.Motor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

/**
 * Represents a group of motors that can be controlled together.
 * Implements the Motor interface to provide unified control over multiple motors.
 */
public class MotorGroup implements Motor {
    private final Motor[] m_motors;

    /**
     * Constructs a MotorGroup with the specified motors.
     *
     * @param motors The motors to be grouped together.
     */
    public MotorGroup(Motor... motors) {
        this.m_motors = motors;
    }

    /**
     * Stops all motors in the group.
     */
    @Override
    public void stopMotor() {
        for (Motor motor : m_motors) {
            motor.stopMotor();
        }
    }

    /**
     * Sets the voltage for all motors in the group.
     *
     * @param voltage The voltage to set.
     */
    @Override
    public void setVoltage(double voltage) {
        for (Motor motor : m_motors) {
            motor.setVoltage(voltage);
        }
    }

    /**
     * Sets the percentage output for all motors in the group.
     *
     * @param percentage The percentage output to set.
     */
    @Override
    public void setPercentage(double percentage) {
        for (Motor motor : m_motors) motor.setPercentage(percentage);
    }

    /**
     * Sets a follower mode for all motors except the main motor.
     *
     * @param mainMotorID The ID of the main motor.
     */
    @Override
    public void setFollower(int mainMotorID) {
        for (Motor motor : m_motors) {
            if (motor.getDeviceID() != mainMotorID) motor.setFollower(mainMotorID);
        }
    }

    /**
     * Sets the idle state for all motors in the group.
     *
     * @param idleMode The idle state to set.
     */
    @Override
    public void setIdleState(IdleState idleMode) {
        for (Motor motor : m_motors) {
            motor.setIdleState(idleMode);
        }
    }

    /**
     * Gets the idle state of the first motor in the group.
     *
     * @return The idle state of the first motor.
     */
    @Override
    public IdleState getIdleState() {
        return m_motors[0].getIdleState();
    }

    /**
     * Gets the device ID of the first motor in the group.
     *
     * @return The device ID of the first motor, or -1 if no motors are present.
     */
    @Override
    public int getDeviceID() {
        return m_motors.length > 0 ? m_motors[0].getDeviceID() : -1;
    }

    /**
     * Gets the average position of all motors in the group.
     *
     * @return The average motor position.
     */
    @Override
    public double getMotorPosition() {
        double totalPosition = 0;
        for (Motor motor : m_motors) {
            totalPosition += motor.getMotorPosition();
        }
        return totalPosition / m_motors.length;
    }

    /**
     * Gets the average velocity of all motors in the group.
     *
     * @return The average motor velocity.
     */
    @Override
    public double getMotorVelocity() {
        double totalVelocity = 0;
        for (Motor motor : m_motors) {
            totalVelocity += motor.getMotorVelocity();
        }
        return totalVelocity / m_motors.length;
    }

    /**
     * Gets the average current of all motors in the group.
     *
     * @return The average motor current.
     */
    @Override
    public double getCurrent() {
        double totalCurrent = 0;
        for (Motor motor : m_motors) {
            totalCurrent += motor.getCurrent();
        }
        return totalCurrent / m_motors.length;
    }

    /**
     * Gets the average voltage of all motors in the group.
     *
     * @return The average motor voltage.
     */
    @Override
    public double getVoltage() {
        double totalVoltage = 0;
        for (Motor motor : m_motors) {
            totalVoltage += motor.getVoltage();
        }
        return totalVoltage / m_motors.length;
    }

    /**
     * Gets the maximum temperature among all motors in the group.
     *
     * @return The maximum motor temperature.
     */
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

    /**
     * Sets a soft limit for all motors in the group.
     *
     * @param directionState The direction state for the limit.
     * @param limit          The soft limit value.
     */
    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        for (Motor motor : m_motors) {
            motor.setSoftLimit(directionState, limit);
        }
    }

    /**
     * Sets the inversion state for all motors in the group.
     *
     * @param mode The inversion state to set.
     */
    @Override
    public void setInverted(DirectionState mode) {
        for (Motor motor : m_motors) {
            motor.setInverted(mode);
        }
    }

    /**
     * Sets the position conversion factor for all motors in the group.
     *
     * @param conversionFactor The position conversion factor to set.
     */
    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        for (Motor motor : m_motors) {
            motor.setPositionConversionFactor(conversionFactor);
        }
    }

    /**
     * Sets the velocity conversion factor for all motors in the group.
     *
     * @param conversionFactor The velocity conversion factor to set.
     */
    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        for (Motor motor : m_motors) {
            motor.setVelocityConversionFactor(conversionFactor);
        }
    }

    /**
     * Sets the current limits for all motors in the group.
     *
     * @param stallLimit The stall current limit.
     * @param freeLimit  The free current limit.
     */
    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        for (Motor motor : m_motors) {
            motor.setCurrentLimit(stallLimit, freeLimit);
        }
    }

    /**
     * Sets the position of all motors in the group.
     *
     * @param position The position to set.
     */
    @Override
    public void setMotorPosition(double position) {
        for (Motor motor : m_motors) {
            motor.setMotorPosition(position);
        }
    }

    protected void setDifferentialVoltage(double motorAvoltage, double motorBvoltage) {
        // this check isn't necessary right now, but might be in the future
        if (!(this instanceof DifferentialMotor)) {
            System.out.println("setDifferentialVoltage only works with a DifferentialMotor");
            return;
        }


        m_motors[0].setVoltage(motorAvoltage);
        m_motors[1].setVoltage(motorBvoltage);
    }
}