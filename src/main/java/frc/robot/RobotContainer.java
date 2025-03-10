package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.ArcadeDrive;


public class RobotContainer {


   // private final DefaultDrive m_defaultDrive =
   //   new DefaultDrive(m_driveSubsystem, this::getControllerLeftY, this::getControllerRightY);
   private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
   private final XboxController gamepad0 = new XboxController(0);


   
   // Init For Autonomous
   private final SendableChooser<Command> autoChooser;


   private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_driveSubsystem, -gamepad0.getLeftY(), -gamepad0.getRightX(), gamepad0.getLeftBumperButton(), gamepad0.getBButton(), gamepad0.getXButton());

   public RobotContainer() {
     // Build an auto chooser. This will use Commands.none() as the default option.
     autoChooser = AutoBuilder.buildAutoChooser();
     // Another option that allows you to specify the default auto by its name
     // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
     //NamedCommands.registerCommand("EjectCommand", m_shooterCommand);
     SmartDashboard.putData("Auto Chooser", autoChooser);
     m_driveSubsystem.setDefaultCommand(m_arcadeDrive);
     }
  
     public Command getAutonomousCommand() {
       return autoChooser.getSelected();
     }
}
