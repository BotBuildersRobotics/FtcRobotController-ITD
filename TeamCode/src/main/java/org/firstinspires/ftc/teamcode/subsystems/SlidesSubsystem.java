package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlidesSubsystem extends SubsystemBase {

    //Define motors and servos
    private DcMotor verticalSlideMotor;

    // Define variables
    private int stowedSlidesPosition = 0;
    private int backwardsTransferPosition = 0;
    private int lowChamberPosition = 150;
    private int highChamberPosition = 500;
    private int lowBasketPosition = 800;
    private int highBasketPosition = 870;//2150;

    private int dumpPosition = 800;

    private int dumpHalfPos = 400;

    private int deliverHighChamberPosition = 0;

    //IMPORTANT: this value gives the motor some breathing room on the retraction
    //we can cook motors if this value is too LOW
    //prob only want to make this number larger than 50
    private int STOWED_SLIDE_DIFFERENCE = 50;

    //private static final PIDFController slidePIDF = new PIDFController(0.01,0,0.0002, 0.00018);

    private static final PIDFController slidePIDF = new PIDFController(0.1,0,0.0002, 0.00018);

    public double target;

    public SlidesSubsystem(final HardwareMap hMap) {
        verticalSlideMotor = hMap.get(DcMotor.class, "verticalSlidesMotor");
        verticalSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void resetVerticalSlides(){
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void stowSlides() {
        //verticalSlideMotor.setTargetPosition(stowedSlidesPosition);
        //verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //verticalSlideMotor.setPower(1);
        setSlideTarget(stowedSlidesPosition);
    }

    public void NoPowerSlides(){
        verticalSlideMotor.setPower(0);
    }

    public double getCurrentSlidePos(){
        return verticalSlideMotor.getCurrentPosition();
    }

    public boolean AreSlidesStowed() {
        return verticalSlideMotor.getCurrentPosition() < (stowedSlidesPosition + STOWED_SLIDE_DIFFERENCE);
    }

    public void highChamberDeliver(){
        verticalSlideMotor.setTargetPosition(deliverHighChamberPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
    }

    public boolean IsAtHighChamberDeliver(){
        return verticalSlideMotor.getCurrentPosition() < (deliverHighChamberPosition +50);
    }

    public void backwardsTransfer() {
        verticalSlideMotor.setTargetPosition(backwardsTransferPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
    }

    public boolean AreSlidesAllowingBackwardsTransfer() {
        return verticalSlideMotor.getCurrentPosition() > (backwardsTransferPosition - 50);
    }

    public void lowChamber() {
        verticalSlideMotor.setTargetPosition(lowChamberPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
    }

    public boolean IsAtLowChamber() {
        return verticalSlideMotor.getCurrentPosition() > (lowChamberPosition - 50);
    }
    public void highChamber() {
        verticalSlideMotor.setTargetPosition(highChamberPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
    }

    public boolean IsAtHighChamber() {
        return verticalSlideMotor.getCurrentPosition() > (highChamberPosition - 50);
    }

    public void lowBasket() {
        /*verticalSlideMotor.setTargetPosition(lowBasketPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);*/
        setSlideTarget(lowBasketPosition);
    }

    public void dumpPosition(){
        setSlideTarget(dumpPosition);
    }

    public boolean IsAtDumpPosition(){
        return verticalSlideMotor.getCurrentPosition() > (dumpPosition - 50);
    }

    public void halfDump(){
        setSlideTarget(dumpHalfPos);
    }

    public boolean IsAtHalfDumpPosition(){
        return verticalSlideMotor.getCurrentPosition() > (dumpHalfPos - 50);
    }

    public boolean IsAtLowBasket() {
        return verticalSlideMotor.getCurrentPosition() > (lowBasketPosition - 50);
    }
    public void highBasket() {
        /*
        verticalSlideMotor.setTargetPosition(highBasketPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
        */
         setSlideTarget(highBasketPosition);
    }

    public boolean IsAtHighBasket() {
        return verticalSlideMotor.getCurrentPosition() > (highBasketPosition - 50);
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, highBasketPosition), 0);
        slidePIDF.setSetPoint(target);
    }

    public void autoUpdateSlides() {
        double power = slidePIDF.calculate(verticalSlideMotor.getCurrentPosition(), target);
        verticalSlideMotor.setPower(power);

    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}