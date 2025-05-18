package frc.excalib.control.differential_gearbox;

import frc.excalib.control.motor.Motor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

public class DifferentialGearBox {
    private final Motor input1, input2;
    private final Motor diffOutput, sumOutput;

    private double diffVoltage = 0, sumVoltage = 0;

    private double diffPositionOffset = 0, sumPositionOffset = 0;
    private double diffPositionConversion = 1, diffVelocityConversion = 1;
    private double sumPositionConversion = 1, sumVelocityConversion = 1;

    public DifferentialGearBox(Motor input1, Motor input2) {
        this.input1 = input1;
        this.input2 = input2;

        diffOutput = new Motor() {
            @Override
            public void stopMotor() {
                setVoltage(0);
            }

            @Override
            public void setVoltage(double voltage) {
                diffVoltage = voltage;
                setVoltages();
            }

            @Override
            public void setPercentage(double percentage) {
                setVoltage(percentage * 12);
            }

            @Override
            public void setFollower(int mainMotorID) {}

            @Override
            public void setIdleState(IdleState idleMode) {
                input1.setIdleState(idleMode);
                input2.setIdleState(idleMode);
            }

            @Override
            public IdleState getIdleState() {
                return input1.getIdleState();
            }

            @Override
            public int getDeviceID() {
                return -1;
            }

            @Override
            public double getMotorPosition() {
                return halfDiff(input1.getMotorPosition(), input2.getMotorPosition()) * diffPositionConversion + diffPositionOffset;
            }

            @Override
            public double getMotorVelocity() {
                return halfDiff(input1.getMotorVelocity(), input2.getMotorVelocity()) * diffVelocityConversion;
            }

            @Override
            public double getCurrent() {
                return halfDiff(input1.getCurrent(), input2.getCurrent());
            }

            @Override
            public double getVoltage() {
                return halfDiff(input1.getVoltage(), input2.getVoltage());
            }

            @Override
            public double getTemperature() {
                return -1;
            }

            @Override
            public void setSoftLimit(DirectionState directionState, float limit) {}

            @Override
            public void setInverted(DirectionState mode) {}

            @Override
            public void setPositionConversionFactor(double conversionFactor) {
                diffPositionConversion = conversionFactor;
            }

            @Override
            public void setVelocityConversionFactor(double conversionFactor) {
                diffVelocityConversion = conversionFactor;
            }

            @Override
            public void setCurrentLimit(int stallLimit, int freeLimit) {}

            @Override
            public void setMotorPosition(double position) {
                double raw = (position - diffPositionOffset) / diffPositionConversion;
                double pos1 = input1.getMotorPosition();
                double pos2 = input2.getMotorPosition();
                double avg = (pos1 + pos2) / 2;
                double diff = raw;
                input1.setMotorPosition(avg + diff);
                input2.setMotorPosition(avg - diff);
            }
        };

        sumOutput = new Motor() {
            @Override
            public void stopMotor() {
                setVoltage(0);
            }

            @Override
            public void setVoltage(double voltage) {
                sumVoltage = voltage;
                setVoltages();
            }

            @Override
            public void setPercentage(double percentage) {
                setVoltage(percentage * 12);
            }

            @Override
            public void setFollower(int mainMotorID) {}

            @Override
            public void setIdleState(IdleState idleMode) {
                input1.setIdleState(idleMode);
                input2.setIdleState(idleMode);
            }

            @Override
            public IdleState getIdleState() {
                return input1.getIdleState();
            }

            @Override
            public int getDeviceID() {
                return -1;
            }

            @Override
            public double getMotorPosition() {
                return average(input1.getMotorPosition(), input2.getMotorPosition()) * sumPositionConversion + sumPositionOffset;
            }

            @Override
            public double getMotorVelocity() {
                return average(input1.getMotorVelocity(), input2.getMotorVelocity()) * sumVelocityConversion;
            }

            @Override
            public double getCurrent() {
                return average(input1.getCurrent(), input2.getCurrent());
            }

            @Override
            public double getVoltage() {
                return average(input1.getVoltage(), input2.getVoltage());
            }

            @Override
            public double getTemperature() {
                return -1;
            }

            @Override
            public void setSoftLimit(DirectionState directionState, float limit) {}

            @Override
            public void setInverted(DirectionState mode) {}

            @Override
            public void setPositionConversionFactor(double conversionFactor) {
                sumPositionConversion = conversionFactor;
            }

            @Override
            public void setVelocityConversionFactor(double conversionFactor) {
                sumVelocityConversion = conversionFactor;
            }

            @Override
            public void setCurrentLimit(int stallLimit, int freeLimit) {}

            @Override
            public void setMotorPosition(double position) {
                double raw = (position - sumPositionOffset) / sumPositionConversion;
                double diff = input1.getMotorPosition() - input2.getMotorPosition();
                double pos1 = raw + diff / 2;
                double pos2 = raw - diff / 2;
                input1.setMotorPosition(pos1);
                input2.setMotorPosition(pos2);
            }
        };
    }

    private static double average(double a, double b) {
        return (a + b) / 2;
    }

    private static double halfDiff(double a, double b) {
        return (a - b) / 2;
    }

    private void setVoltages() {
        input1.setVoltage(sumVoltage - diffVoltage / 2);
        input2.setVoltage(sumVoltage + diffVoltage / 2);
    }

    public Motor getDiffOutput() {
        return diffOutput;
    }

    public Motor getSumOutput() {
        return sumOutput;
    }
}
