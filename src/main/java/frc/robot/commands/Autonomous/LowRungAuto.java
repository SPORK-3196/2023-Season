package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Cube.EjectCubeAuto;
import frc.robot.commands.Autonomous.Positions.AutoMoveArmToLow;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;

public class LowRungAuto extends SequentialCommandGroup{
    Drivetrain drivetrain;
    EveryClaw claw;

    public LowRungAuto(Drivetrain drivetrain, EveryClaw claw){
        super(
            new AutoMoveArmToLow(),
            new EjectCubeAuto(claw),
            new AutoBackArmRest(drivetrain, 0.5, 5.25)        
        );
    }
}