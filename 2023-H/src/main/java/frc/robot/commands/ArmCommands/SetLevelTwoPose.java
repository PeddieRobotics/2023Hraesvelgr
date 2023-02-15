// package frc.robot.commands.ArmCommands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Claw;
// import frc.robot.utils.Constants.ShoulderConstants;
// import frc.robot.utils.Constants.WristConstants;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


// public class SetLevelTwoPose extends CommandBase{
//     private Arm arm;
//     private Claw claw;

//     public SetLevelTwoPose() {
//         arm = Arm.getInstance();
//         claw = Claw.getInstance();
//         if(claw.hasCone())
//             new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderLevelTwoConeAngle), new SetWristPosition(WristConstants.kWristLevelTwoConeAngle));
//         else 
//             new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderLevelTwoCubeAngle), new SetWristPosition(WristConstants.kWristLevelTwoCubeAngle));
//         addRequirements(arm);
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }

//     @Override
//     public boolean isFinished() {
//         if((arm.getShoulderPosition() == ShoulderConstants.kShoulderLevelTwoConeAngle && arm.getWristPosition() == WristConstants.kWristLevelTwoConeAngle) || (arm.getShoulderPosition() == ShoulderConstants.kShoulderLevelTwoCubeAngle && arm.getWristPosition() == WristConstants.kWristLevelTwoCubeAngle))
//             return true;
//         else
//             return false;
//     }
// }
