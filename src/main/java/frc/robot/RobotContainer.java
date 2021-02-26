// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevatorSubsystem);
  private final ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevatorSubsystem);

  final XboxController driverController = new XboxController(USBConstants.DRIVER_CONTROLLER_PORT);
  
  private void calibrate() {
      System.out.println("Gyro is calibrating...");
      driveSubsystem.calibrateGyro();
      }


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    calibrate();
      
    configureButtonBindings();
       driveSubsystem.setDefaultCommand(new RunCommand(() -> {
         driveSubsystem.arcadeDrive(
          -driverController.getY(GenericHID.Hand.kLeft), 
          driverController.getX(GenericHID.Hand.kRight));
        }, driveSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // final Button a = new JoystickButton(driverController, XboxControllerConstants.A_BUTTON);
    new JoystickButton(driverController, XboxControllerConstants.LB_BUTTON).whileHeld(elevatorDownCommand);
    new JoystickButton (driverController, XboxControllerConstants.RB_BUTTON).whileHeld(elevatorUpCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An  will run in autonomous
    return new ElevatorDownCommand(elevatorSubsystem);
  }
}
