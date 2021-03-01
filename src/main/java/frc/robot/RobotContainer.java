// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.Vision;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

 

  private final Vision vision = new Vision();
  final XboxController driverController = new XboxController(UsbConstants.DRIVER_CONTROLLER_PORT);

  private final AutoAimCommand autoAimCommand = new AutoAimCommand(vision, driveSubsystem, () -> -driverController.getY(GenericHID.Hand.kLeft));
  private final StartEndCommand elevatorDownCommand = new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(-.25), () -> elevatorSubsystem.setElevatorSpeed(0), elevatorSubsystem);
  private final StartEndCommand elevatorUpCommand = new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(.25), () -> elevatorSubsystem.setElevatorSpeed(0), elevatorSubsystem);


  
  private void calibrate() {
    System.out.println("Gyro is calibrating...");
    driveSubsystem.calibrateGyro();
    }

    public DriveSubsystem getDriveSubsystem(){
      return driveSubsystem;
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
    Button lb = new JoystickButton(driverController, XboxConstants.LB_BUTTON);
    Button rb = new JoystickButton(driverController, XboxConstants.RB_BUTTON);

    lb.whileHeld(elevatorDownCommand);
    rb.whileHeld(elevatorUpCommand);

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
