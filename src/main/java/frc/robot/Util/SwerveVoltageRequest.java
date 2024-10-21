package frc.robot.Util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class SwerveVoltageRequest implements SwerveRequest {
    private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
    private final VoltageOut m_voltageOutControl = new VoltageOut(0.0);

    private double m_targetVoltage = 0.0;
    private boolean m_driveType = true;

    public SwerveVoltageRequest(boolean driveType) {
        m_driveType = driveType;
    }

    public SwerveVoltageRequest() {
        m_driveType = true;
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (var module : modulesToApply) 
        {
            if (m_driveType) {
                module.getSteerMotor().setControl(m_motionMagicControl);

                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));
            }
            else {
                module.getSteerMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));

                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(0));
            }
        }

        return StatusCode.OK;
    }


    public SwerveVoltageRequest withVoltage(double targetVoltage) {
        this.m_targetVoltage = targetVoltage;
        return this;
    }
}
