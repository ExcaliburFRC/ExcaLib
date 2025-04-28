package frc.excalib.control.gains;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

/**
 * The SysidConfig class extends the SysIdRoutine.Config class to configure system identification routines.
 * It provides parameters for ramp rate, step voltage, and timeout duration.
 */
public class SysidConfig extends SysIdRoutine.Config {

    /**
     * Constructs a SysidConfig object with the specified ramp rate, step voltage, and timeout duration.
     *
     * @param rampRate    The rate at which the voltage ramps up, in volts per second.
     * @param stepVoltage The step voltage to apply during the system identification routine, in volts.
     * @param timeOut     The timeout duration for the system identification routine, in seconds.
     */
    public SysidConfig(double rampRate, double stepVoltage, double timeOut) {
        super(
                Volts.of(rampRate).per(Seconds.of(1).unit()), // Converts ramp rate to volts per second.
                Volts.of(stepVoltage), // Converts step voltage to volts.
                Seconds.of(timeOut) // Converts timeout duration to seconds.
        );
    }
}