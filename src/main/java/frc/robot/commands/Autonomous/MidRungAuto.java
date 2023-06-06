package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Cube.EjectCubeAuto;
import frc.robot.commands.Autonomous.Positions.AutoArmToRestPos;
import frc.robot.commands.Autonomous.Positions.AutoMoveArmToMid;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;

public class MidRungAuto extends SequentialCommandGroup{
    Drivetrain drivetrain;
    EveryClaw claw;

    public MidRungAuto(Drivetrain drivetrain, EveryClaw claw){
        super(
            new AutoMoveArmToMid(),
            new EjectCubeAuto(claw),
            new AutoArmToRestPos()       
        );
    }
}