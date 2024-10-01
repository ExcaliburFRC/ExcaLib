package excalib.math.control.motor;

import com.revrobotics.CANSparkFlex;
import excalib.math.control.motor.motor_specs.DirectionState;
import excalib.math.control.motor.motor_specs.IdleState;

public class FlexMotor extends CANSparkFlex implements Motor {


    public FlexMotor(int deviceId, MotorType type) {
        super(deviceId, type);

    }

    @Override
    public void stopMotor() {
        super.stopMotor();
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public void setPercentage(double percentage) {
        super.set(percentage);
    }

    @Override
    public void setFollower(int mainMotorID) {
        super.follow(ExternalFollower.kFollowerSpark, mainMotorID);
    }

    @Override
    public void setIdleMode(IdleState idleMode) {
        super.setIdleMode(idleMode == IdleState.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public int getDeviceID() {
        return super.getDeviceId();
    }

    @Override
    public double getMotorPosition() {
        return this.getEncoder().getPosition();
    }

    @Override
    public double getMotorVelocity() {
        return this.getEncoder().getVelocity() / 60; //we divide by 60 to get it in RPS
    }

    @Override
    public double getCurrent() {
        return 0; //todo: find out how to get current
    }

    @Override
    public double getVoltage() {
        return getBusVoltage() * getAppliedOutput(); //unlike 1937, it gets the output of what it uses and not only the busVoltage
    }


    @Override
    public double getTemperature() {
        return getMotorTemperature();
    }


    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        super.setSoftLimit(directionState == DirectionState.FORWARD ?
                SoftLimitDirection.kForward :
                SoftLimitDirection.kReverse, limit);
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        getEncoder().setPositionConversionFactor(conversionFactor);
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        getEncoder().setVelocityConversionFactor(conversionFactor);
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        super.setSmartCurrentLimit(stallLimit, freeLimit);
    }
}
