package frc.robot.utils;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;


public class FourMeterAuto {
    
    private final Trajectory path;
    private final Command command;
    
    public FourMeterAuto(DriveSubsystem driveSubsystem){
        path = driveSubsystem.pathList.get(1);
        
        command = 
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> driveSubsystem.resetPose(path.getInitialPose()), driveSubsystem),
                    driveSubsystem.ramsete(path),
                    new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem));
                
            
    }

    public Command getCommand(){
        System.out.println("Four meters");
        return command;
    }

}
