package frc.excalib.control.feedback;

import frc.excalib.additional_utilities.periodics.PeriodicScheduler;
import frc.excalib.additional_utilities.periodics.PeriodicTask;
import frc.excalib.control.gains.Gains;

import java.util.function.DoubleSupplier;

public class PID extends PeriodicTask {
    private final double kp, ki, kd;
    private final double periodSeconds;

    private double setpoint;
    private double output = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean firstRun = true;

    public PID(DoubleSupplier measurementSupplier, double setpoint, Gains gains, PeriodicScheduler.PERIOD period) {
        super(() -> {}, period);
        this.setpoint = setpoint;
        this.kp = gains.kp;
        this.ki = gains.ki;
        this.kd = gains.kd;
        this.periodSeconds = period.milliseconds / 1000.0;

        super.setTask(() -> {
            double measurement = measurementSupplier.getAsDouble();
            double error = setpoint - measurement;

            integral += error * periodSeconds;
            double derivative = firstRun ? 0.0 : (error - lastError) / periodSeconds;

            output = kp * error + ki * integral + kd * derivative;
            lastError = error;
            firstRun = false;
        });
    }

    public double getOutput() {
        return output;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        firstRun = true;
    }
}
