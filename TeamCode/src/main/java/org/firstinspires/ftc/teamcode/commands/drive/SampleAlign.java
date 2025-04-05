package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

public class SampleAlign extends CommandBase {
    private final MecanumDrive drive;
    private final LimelightSubsystem vision;
    private final Telemetry telemetry;


    private PIDController alignPID = new PIDController(0.1,0,0);
    public SampleAlign(
            MecanumDrive drive,
            LimelightSubsystem vision,
            Telemetry telemetry) {

        this.drive = drive;
        this.vision = vision;
        this.telemetry = telemetry;
        this.alignPID.setSetPoint(0);

        this.alignPID.setTolerance(0.01);
        vision.setPipeline(0);

        addRequirements(vision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double XPosOfSample = this.vision.getPythonX();
        double YPosOfSample = this.vision.getPythonY();
        boolean visible = this.vision.isTargetVisible();
        double alignMovement = alignPID.calculate(XPosOfSample);

        telemetry.addData("PID", alignMovement);
        telemetry.addData("XPos", XPosOfSample);
        telemetry.addData("XPos LL", this.vision.getTx());
        telemetry.addData("YPos", YPosOfSample);
        telemetry.addData("Visible", visible);

       // telemetry.update();

        double scale = 0.2;
        Vector2d sticks = new Vector2d(
                alignMovement * scale,
                -alignMovement * scale
        );

        drive.setDrivePowers(new PoseVelocity2d(
                sticks,
                0
        ));

    }

    @Override
    public boolean isFinished() {
        /*if(!vision.isTargetVisible()){
            //no target - just end
            return true;
        }
        else if(alignPID.atSetPoint()){
            return true;
        }*/
        return false;
    }
}
