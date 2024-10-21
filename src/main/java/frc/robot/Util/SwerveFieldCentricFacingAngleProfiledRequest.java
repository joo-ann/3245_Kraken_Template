package frc.robot.Util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;


public class SwerveFieldCentricFacingAngleProfiledRequest implements SwerveRequest {

    public double VelocityX = 0;

    public double VelocityY = 0;

    public Rotation2d TargetDirection = new Rotation2d();

    public double Deadband = 0;

    public double RotationalDeadband = 0;

    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;

    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public ProfiledPIDController HeadingController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = VelocityX;
        double toApplyY = VelocityY;

        double rotationRate = HeadingController.calculate(parameters.currentPose.getRotation().getRadians(),
                TargetDirection.getRadians());

        double toApplyOmega = rotationRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds
                .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                        parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    public SwerveFieldCentricFacingAngleProfiledRequest withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }


    public SwerveFieldCentricFacingAngleProfiledRequest withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    public SwerveFieldCentricFacingAngleProfiledRequest withTargetDirection(Rotation2d targetDirection) {
        this.TargetDirection = targetDirection;
        return this;
    }

    public SwerveFieldCentricFacingAngleProfiledRequest withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    public SwerveFieldCentricFacingAngleProfiledRequest withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    public SwerveFieldCentricFacingAngleProfiledRequest withDriveRequestType(
            SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }


    public SwerveFieldCentricFacingAngleProfiledRequest withSteerRequestType(
            SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }

    public SwerveFieldCentricFacingAngleProfiledRequest withHeadingController(
            ProfiledPIDController headingController) {
        this.HeadingController = headingController;
        return this;
    }
}
