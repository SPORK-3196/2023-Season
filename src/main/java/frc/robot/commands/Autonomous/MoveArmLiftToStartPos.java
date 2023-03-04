package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Autonomous.Positions.ArmPosition;
import frc.robot.commands.Autonomous.Positions.ElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

public class MoveArmLiftToStartPos extends ParallelCommandGroup{
    Arm arm;
    Lift lift;

    public MoveArmLiftToStartPos(Arm arm, Lift lift){
        super(
            new ArmPosition(arm),
            new ElevatorPosition(lift)
        );
    }
}
