// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandXboxController opController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Shooter shooter;

  private final Indexer indexer;

  private final Hopper hopper;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter();
        indexer = new Indexer();
        hopper = new Hopper();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        shooter = new Shooter();
        indexer = new Indexer();
        hopper = new Hopper();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter();
        indexer = new Indexer();
        hopper = new Hopper();
        break;
    }

    NamedCommands.registerCommand("Shooter Out", shooter.setVelocity(RotationsPerSecond.of(47.5)));
    NamedCommands.registerCommand("Intake In", shooter.set(0.69));
    NamedCommands.registerCommand("Intake to Hopper", indexer.set(0.8));
    NamedCommands.registerCommand("Hopper to Shooter", indexer.set(-0.6));
    NamedCommands.registerCommand(
        "Indexer Off", new ParallelDeadlineGroup(new WaitCommand(0.2), indexer.set(0)));
    NamedCommands.registerCommand(
        "Shooter/Intake Off", new ParallelDeadlineGroup(new WaitCommand(0.2), shooter.set(0)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Shoot auto",
    //     new SequentialCommandGroup(
    //         shooter.setVelocity(RotationsPerSecond.of(90)),
    //         new WaitCommand(1.5),
    //         indexer.setDutyCycle(() -> 0.8)));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX() * 0.8));

    shooter.setDefaultCommand(shooter.set(0));
    indexer.setDefaultCommand(indexer.set(0));
    hopper.setDefaultCommand(hopper.set(0));

    // drive.setDefaultCommand(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> {
    //           Rotation2d rot = new Rotation2d();
    //           if (Math.hypot(controller.getRightY(), controller.getRightX()) > 0.9) {
    //             rot =
    //                 new Rotation2d(
    //                     -(Math.atan2(controller.getRightY(), controller.getRightX())
    //                         - Math.PI / 2));
    //           } else {
    //             rot = drive.getRotation();
    //           }
    //           Logger.recordOutput("Commanded Rotation", rot);
    //           return rot;
    //         }));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> {
    //               Rotation2d rot = DriveCommands.hubAngle(drive.getPose());
    //               Logger.recordOutput("Commanded Rotation", rot);
    //               return rot;
    //             }));

    // shooter to hub
    opController.rightTrigger().whileTrue(shooter.setVelocity(RotationsPerSecond.of(47.5)));

    // shooter reverse (never really used)
    opController.leftTrigger().whileTrue(shooter.set(-0.9));

    // hopper indexer to shooter
    // controller.rightBumper().whileTrue(indexer.set(-0.35));

    // hopper indexer to shooter
    opController.rightBumper().whileTrue(indexer.set(-0.6));

    // intake to hopper
    opController
        .leftBumper()
        .whileTrue(new ParallelCommandGroup(indexer.set(0.8), shooter.set(0.69)));

    opController.a().whileTrue(hopper.set(1));
    opController.y().whileTrue(hopper.set(-1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
