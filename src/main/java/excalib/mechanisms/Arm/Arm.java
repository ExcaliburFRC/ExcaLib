package excalib.mechanisms.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import excalib.math.control.motor.Motor;
import excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;


public class Arm extends Mechanism {
    private final Motor m_motor;
    public Arm(Motor motor){
        m_motor = motor;
    }
    protected void setOutput(double voltage){
        m_motor.setVoltage(voltage);
    }
    public Command ManualCommand(DoubleSupplier voltageSupplier){
        return new RunCommand(()->setOutput(voltageSupplier.getAsDouble()));
    }
}
