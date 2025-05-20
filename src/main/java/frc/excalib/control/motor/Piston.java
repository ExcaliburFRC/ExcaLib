package frc.excalib.control.motor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.excalib.control.motor.motor_specs.DirectionState;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Piston {
    private final DoubleSolenoid piston;
    private DirectionState direction;

    public Piston(int fwdID, int revID, PneumaticsModuleType moduleType){
        this.piston = new DoubleSolenoid(moduleType, fwdID, revID);
        this.direction = DirectionState.REVERSE;
    }

    public void setPiston(DirectionState direction){
        this.piston.set(direction == DirectionState.FORWARD? kForward : kReverse);
        this.direction = direction;
    }

    public void togglePiston(){
        if (direction == DirectionState.FORWARD) setPiston(DirectionState.REVERSE);
        else setPiston(DirectionState.FORWARD);
    }
}
