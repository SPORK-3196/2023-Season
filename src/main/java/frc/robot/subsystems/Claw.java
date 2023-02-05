package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    public Solenoid piston1 = new Solenoid(PneumaticsModuleType.REVPH, 10);

    public Claw(){
    }

    public void TogglePiston(){
        piston1.toggle();
    }
    public void extendPiston(){
        piston1.set(true);
    }
    public void retractPiston(){
        piston1.set(false);
    }
}
