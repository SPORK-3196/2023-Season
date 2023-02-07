package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;;

public class Claw extends SubsystemBase{
    private final Solenoid piston1 
        = new Solenoid(PneumaticsModuleType.REVPH, 
        ClawConstants.piston1Port);

    private final Solenoid piston2 
        = new Solenoid(PneumaticsModuleType.REVPH, 
        ClawConstants.piston2Port);
    public Claw(){
        
    }

    public CommandBase closePiston1() {
        return this.runOnce(() -> piston1.set(ClawConstants.kSolenoidForward));
    }
    public CommandBase closePiston2() {
        return this.runOnce(() -> piston2.set(ClawConstants.kSolenoidForward));
    }
    public CommandBase openPistons() {
        return (piston1.get() ?  
        this.runOnce(() -> piston1.set(ClawConstants.kSolenoidReverse)) 
        : this.runOnce(() -> piston2.set(ClawConstants.kSolenoidReverse)));
    }

    public void TogglePiston2(){
        piston2.toggle();
    }
    public void extendPiston2(){
        piston2.set(true);
    }
    public void retractPiston2(){
        piston2.set(false);
    }
}
