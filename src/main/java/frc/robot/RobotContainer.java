// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.AutoAimCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.FourMeterAuto;
import frc.robot.utils.ThreeMeterAuto;
import frc.robot.utils.Vision;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RobotContainer {
  final XboxController driverController = new XboxController(UsbConstants.DRIVER_CONTROLLER_PORT);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
 

  private final Vision vision = new Vision();
  private final ThreeMeterAuto threeMeterAuto = new ThreeMeterAuto(driveSubsystem);
  private final FourMeterAuto fourMeterAuto = new FourMeterAuto(driveSubsystem);

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final AutoAimCommand autoAimCommand = new AutoAimCommand
  (vision, driveSubsystem, () -> -driverController.getY(GenericHID.Hand.kLeft));

  private final StartEndCommand elevatorDownCommand = new StartEndCommand
  (() -> elevatorSubsystem.setElevatorSpeed(-.25), 
  () -> elevatorSubsystem.setElevatorSpeed(0), elevatorSubsystem);

  private final StartEndCommand elevatorUpCommand = new StartEndCommand
  (() -> elevatorSubsystem.setElevatorSpeed(.25), 
  () -> elevatorSubsystem.setElevatorSpeed(0), elevatorSubsystem);

  private final StartEndCommand shootCommand = new StartEndCommand
    (() -> shooterSubsystem.setSpeed(.25) , 
    () -> shooterSubsystem.setSpeed(0), shooterSubsystem);

  
  private void calibrate() {
      System.out.println("Gyro is calibrating...");
      driveSubsystem.calibrateGyro();
      }

    public DriveSubsystem getDriveSubsystem(){
      return driveSubsystem;
    }
    
    public XboxController getDriverController() {
      return driverController;
    }

    private void shuffleboardSetup(){
      final ShuffleboardTab tab = Shuffleboard.getTab("Auto");

      tab.add("Select program for auto", chooser);
      chooser.setDefaultOption("3 meter", threeMeterAuto.getCommand());
      chooser.addOption("4 meter", fourMeterAuto.getCommand());
    }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    calibrate();
    shuffleboardSetup();
      
    configureButtonBindings();
      driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      driveSubsystem.arcadeDrive(
      -driverController.getY(GenericHID.Hand.kLeft), 
      driverController.getX(GenericHID.Hand.kRight));
    }, driveSubsystem));


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
    Button a = new JoystickButton(driverController, XboxConstants.A_BUTTON);

    lb.whileHeld(elevatorDownCommand);
    rb.whileHeld(elevatorUpCommand);
    a.whenPressed(autoAimCommand);
    a.whileHeld(shootCommand);



  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new SequentialCommandGroup(
      chooser.getSelected(), autoAimCommand.withTimeout(2), shootCommand.withTimeout(5));
    
  }
}
