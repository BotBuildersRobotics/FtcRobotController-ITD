package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Common Blue TeleOp")
public class CommonBlueTeleOp extends CommonTeleOp {



    @Override
    public void initialize() {
       this.setDesiredColour(IntakeSubsystem.SampleColour.BLUE_OR_NEUTRAL);
       super.initialize();
    }



}


