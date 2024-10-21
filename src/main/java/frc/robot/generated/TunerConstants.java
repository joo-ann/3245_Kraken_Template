package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {

    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(69).withKI(0).withKD(0)
            .withKS(0.1).withKV(1.5).withKA(0.001);

    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.186).withKI(0).withKD(0)
            .withKS(0.1).withKV(0.117).withKA(0.029);


    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;


    private static final double kSlipCurrentA = 75.0;


    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = Units.metersToInches(0.0989) / 2.0;


    public static final double kSpeedAt12VoltsMps = 107.0 / kDriveGearRatio * Math.PI * 2
            * Units.inchesToMeters(kWheelRadiusInches); 
    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "Default Name";
    private static final int kPigeonId = 10;


    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.25)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    private static final int kFrontLeftDriveMotorId = 11;
    private static final int kFrontLeftSteerMotorId = 12;
    private static final int kFrontLeftEncoderId = 19;
    private static final double kFrontLeftEncoderOffset = -0.065186;

    private static final double kFrontLeftXPosInches = 10.375;
    private static final double kFrontLeftYPosInches = 10.375;


    private static final int kFrontRightDriveMotorId = 17;
    private static final int kFrontRightSteerMotorId = 18;
    private static final int kFrontRightEncoderId = 22;
    private static final double kFrontRightEncoderOffset = 0.317383; 

    private static final double kFrontRightXPosInches = 10.375;
    private static final double kFrontRightYPosInches = -10.375;


    private static final int kBackLeftDriveMotorId = 13;
    private static final int kBackLeftSteerMotorId = 14;
    private static final int kBackLeftEncoderId = 20;
    private static final double kBackLeftEncoderOffset = -0.416260; 
    private static final double kBackLeftXPosInches = -10.375;
    private static final double kBackLeftYPosInches = 10.375;


    private static final int kBackRightDriveMotorId = 15;
    private static final int kBackRightSteerMotorId = 16;
    private static final int kBackRightEncoderId = 21;
    private static final double kBackRightEncoderOffset = 0.261963;

    private static final double kBackRightXPosInches = -10.375;
    private static final double kBackRightYPosInches = -10.375;

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
            kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
            FrontLeft, FrontRight, BackLeft, BackRight);
}
