// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.utils.Vision;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class AutoAimCommand extends CommandBase {
  PhotonCamera camera;
  PIDController controller;
  DriveSubsystem driveSubsystem;
  DoubleSupplier forward;
  
  double rotationSpeed = 0;
  PhotonPipelineResult result;

  public AutoAimCommand(Vision vision, DriveSubsystem driveSubsystem, DoubleSupplier forward) {
    this.camera = vision.camera;
    this.controller = vision.controller;
    this.driveSubsystem = driveSubsystem;
    this.forward = forward;

    addRequirements(driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = camera.getLatestResult();

    if (result.hasTargets()) {
      rotationSpeed = controller.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      rotationSpeed = 0;
    }
    driveSubsystem.arcadeDrive(forward.getAsDouble(), rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(result.getBestTarget().getYaw()) <= 0.5){
      return true;
    } else{
      return false;
    }
  }
}
