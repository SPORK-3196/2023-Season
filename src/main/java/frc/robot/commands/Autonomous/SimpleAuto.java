package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;

public class SimpleAuto extends SequentialCommandGroup{
    Drivetrain drivetrain;
    Arm arm;
    Lift lift;

    public SimpleAuto(Drivetrain drivetrain, Arm arm, Lift lift){
        super(
            new MoveArmLiftToStartPos(arm, lift),
            new DriveOntoChargePad(drivetrain)
        );
    }
}
