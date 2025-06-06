package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AscentSubsystem extends SubsystemBase {

    //Define motors and servos


    private Servo acentHookArmServo;

    private Servo ptoLeftServo;
    private Servo ptoRightServo;

    // Define variables
    private int ascentStowedPosition = 0;
    private int ascentLowRungPreparedPosition = 0;
    private int ascentLowRungPosition = 3300;//4300;// 6650; // Distance 274mm C2C  (old 6400)
    private int ascentHighRungPosition = 0;
    private int ascentFinishedLevel3Position = 0;

    private double ascentOpenHookPosition = 1;
    private double ascentClosedHookPosition = 0;

    private double ptoDriveLeftPos = 0.08;
    private double ptoDriveRightPos = 0.30;

    private double ptoClimbLeftPos = 0.65;//0.83;
    private double ptoClimbRightPos = 0.84;

    public AscentSubsystem(final HardwareMap hMap) {


        acentHookArmServo = hMap.get(Servo.class, "ascentHookArmServo");
        ptoLeftServo = hMap.get(Servo.class, "ptoLeft");
        ptoRightServo = hMap.get(Servo.class, "ptoRight");

        acentHookArmServo.getController().pwmEnable();

        ptoRightServo.setPosition(0.30);
        ptoLeftServo.setPosition(0.08);



        acentHookArmServo.setDirection(Servo.Direction.REVERSE);

        ascentCloseHooks();
    }

    public void PTODriveEnabled(){
        ptoLeftServo.setPosition(ptoDriveLeftPos);
        ptoRightServo.setPosition(ptoDriveRightPos);
    }

    public void PTOClimbEnabled(){
        ptoLeftServo.setPosition(ptoClimbLeftPos);
        ptoRightServo.setPosition(ptoClimbRightPos);
    }

    public void ascentStow() {
        //Stows the ascent mechanism

    }

    public boolean IsAscentStowed() {
       return true;
    }

    public void ascentLowRungPrepare() {
        //Stows the ascent mechanism

    }

    public boolean IsAscentPrepared() {
        return true;
    }

    public void ascentLowRung() {
        //Stows the ascent mechanism

    }

    public boolean IsAscentLowRung() {
        return true;
    }

    public int getLeftMotorPos(){
        return 0;
    }

    public int getRightMotorPos(){
        return 0;
    }

    public void ascentHighRung() {
        //Stows the ascent mechanism

    }

    public boolean IsAscentHighRung() {
        return true;
    }

    public void ascentFinishLevel3() {
        //Stows the ascent mechanism

    }

    public boolean IsAscentFinishedLevel3() {
        return true;
    }

    public void ascentOpenHooks() {
        acentHookArmServo.setPosition(ascentOpenHookPosition);

    }

    public void disableServos(){

        acentHookArmServo.getController().pwmDisable();


    }

    public boolean AreAscentHooksOpen() {
        return true;
    }

    public void ascentCloseHooks() {
        acentHookArmServo.setPosition(ascentClosedHookPosition);

    }

    public boolean AreAscentHooksClosed() {
        return true;
    }
}