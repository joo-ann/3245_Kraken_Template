package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.SwerveFieldCentricFacingAngleProfiledRequest;
import frc.robot.Util.Util;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Double> speedChooser = new SendableChooser<>();
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private final double TurtleSpeed = 0.1;
  private final double MaxAngularRate = Math.PI * 4.0;
  private final double TurtleAngularRate = Math.PI * 0.5;
  private double AngularRate = MaxAngularRate;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final CommandXboxController m_testController = new CommandXboxController(2);
  private final CommandXboxController m_sysidController = new CommandXboxController(3);

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.3);

  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(MaxSpeed * 0.005)
      .withRotationalDeadband(AngularRate * 0.005);
  SwerveFieldCentricFacingAngleProfiledRequest driveLockedAngle = new SwerveFieldCentricFacingAngleProfiledRequest()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(MaxSpeed * 0.005)
      .withRotationalDeadband(AngularRate * 0.005)
      .withHeadingController(new ProfiledPIDController(5, 0, 0, new Constraints(Math.PI * 2.0, Math.PI * 4.0)));

  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private Supplier<SwerveRequest> controlStyle = () -> {
    return drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
        .withRotationalRate(-m_driverController.getRightX() * AngularRate);
  };
  private Double lastSpeed = 0.65;
  private boolean isSwerveLockingAngle = false;
  private Rotation2d lockedAngle = new Rotation2d();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightStick()
        .toggleOnTrue(
            Commands.startEnd(() -> {
              lockedAngle = new Rotation2d();
              isSwerveLockingAngle = true;
            }, () -> isSwerveLockingAngle = false));

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          SmartDashboard.putNumber("locked angle", lockedAngle.getDegrees());
          double cubicWeight = 0.75;
          var x = Util.cubic(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1), cubicWeight);
          var y = Util.cubic(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1), cubicWeight);
          var r = Util.cubic(MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1), cubicWeight);

          if (Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - lockedAngle.getDegrees()) < 3) {
            isSwerveLockingAngle = false;
          }
          if (!isSwerveLockingAngle) {
            return drive.withVelocityX(x * MaxSpeed)
                .withVelocityY(y * MaxSpeed)
                .withRotationalRate(r * AngularRate);
          } else {
            return driveLockedAngle.withVelocityX(x * MaxSpeed)
                .withVelocityY(y * MaxSpeed)
                .withTargetDirection(lockedAngle);
          }
        }).ignoringDisable(true));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));
    m_driverController.pov(270).whileTrue(drivetrain.applyRequest(() -> {
      double x = 0;
      double y = 0;
      double r = 0;
      double maxSpeed = 2.0;
      Pose2d goal = new Pose2d(new Translation2d(1.5, 0), new Rotation2d());

      return forwardStraight.withDeadband(.05).withVelocityX(x).withVelocityY(y)
          .withRotationalRate(Units.degreesToRadians(r));
    }));

    m_testController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_testController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-m_testController.getLeftY(), -m_testController.getLeftX()))));
    m_testController.x().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d())));
    m_testController.y()
        .whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(90))));

    m_sysidController.x().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    m_sysidController.x().and(m_sysidController.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    m_sysidController.y().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    m_sysidController.y().and(m_sysidController.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    m_sysidController.a().and(m_sysidController.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    m_sysidController.a().and(m_sysidController.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    m_sysidController.b().and(m_sysidController.pov(0)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    m_sysidController.b().and(m_sysidController.pov(180)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));

    m_sysidController.back().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveSlipTest());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void newSpeed() {
    lastSpeed = speedChooser.getSelected();
    MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * lastSpeed;
  }
}
