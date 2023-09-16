package frc.robot.commands.Autonomous.Cube;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryClaw;

public class IntakeCubeAuto extends CommandBase{
    EveryClaw claw;
    Timer timer;
    double time;
    public IntakeCubeAuto(EveryClaw claw, double time){
        this.claw = claw;
        this.time = time;

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
        claw.runMotor(-.6);
    }
    @Override
    public void end(boolean interrupted){
        claw.runMotor(-0.03);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(time);
    }
}