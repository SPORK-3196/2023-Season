package frc.robot.commands.Autonomous.Cube;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EveryClaw;
import frc.robot.commands.Autonomous.Positions.AutoArmToPickup;

public class DropIntakeCubeAuto extends SequentialCommandGroup{
    EveryClaw claw;
    public DropIntakeCubeAuto(EveryClaw claw) {
        super(
            new AutoArmToPickup(),
            new IntakeCubeAuto(claw, 3)
        );
    }
}
