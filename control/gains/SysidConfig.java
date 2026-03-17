package frc.excalib.control.gains;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SysidConfig extends SysIdRoutine.Config {
    public SysidConfig(double rampRate, double stepVoltage, double timeOut) {
        super(
                Volts.of(rampRate).per(Seconds.of(1).unit()), Volts.of(stepVoltage),
                Seconds.of(timeOut));
    }
}
