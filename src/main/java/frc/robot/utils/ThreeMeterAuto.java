package frc.robot.utils;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeMeterAuto {
    
        private final Trajectory path;
        private final Command command;
        
        public ThreeMeterAuto(DriveSubsystem driveSubsystem){
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
