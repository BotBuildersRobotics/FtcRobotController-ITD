package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDrive  extends CommandBase {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_leftx;
    private final DoubleSupplier m_lefty;

    private final DoubleSupplier m_rightx;

    private final DoubleSupplier m_scale;

    private final BooleanSupplier m_ptoEnabled;

    private final Telemetry telemetry;

    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier scale, BooleanSupplier ptoEnabled, Telemetry tele ) {
        m_drive = subsystem;
        m_leftx = leftX;
        m_lefty = leftY;
        m_rightx = rightX;
        m_scale = scale;
        m_ptoEnabled = ptoEnabled;
        this.telemetry = tele;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        //only allow driver to drive when PTO is not enabled
        if(m_ptoEnabled.getAsBoolean() == false) {
            m_drive.drive(m_leftx.getAsDouble(), m_lefty.getAsDouble(), m_rightx.getAsDouble(), m_scale.getAsDouble());
        }
    }

}
