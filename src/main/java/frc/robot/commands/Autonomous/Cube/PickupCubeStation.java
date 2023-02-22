package frc.robot.commands.Autonomous.Cube;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Autonomous.Positions.ArmStationPosition;
import frc.robot.commands.Claw.CloseClawCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;

public class PickupCubeStation extends ParallelCommandGroup{
    Arm arm;
    Lift lift;
    Claw claw;
    public PickupCubeStation(Arm arm, Lift lift, Claw claw){
        super(
            new ArmStationPosition(arm, lift),
            new CloseClawCube(claw)
        );
    }
}
