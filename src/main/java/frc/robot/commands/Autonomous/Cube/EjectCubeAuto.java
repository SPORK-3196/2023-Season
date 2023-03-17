package frc.robot.commands.Autonomous.Cube;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryClaw;

public class EjectCubeAuto extends CommandBase{
    EveryClaw claw;
    Timer timer;

    public EjectCubeAuto(EveryClaw claw){
        this.claw = claw;

        timer = new Timer();

        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.stopMotor();

        timer.reset();
        timer.start();

    }
    @Override
    public void execute(){
        claw.runMotor(.4);
    }
    @Override
    public void end(boolean interrupted){
        claw.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(1.5);
    }
}
