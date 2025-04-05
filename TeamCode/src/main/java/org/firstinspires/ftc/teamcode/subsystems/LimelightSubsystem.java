package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightSubsystem extends SubsystemBase {

    private Limelight3A limelight;
    private Telemetry telemetry;
    private LLResult result;

    private final int startingPipeline = 0;

    public static double CAMERA_HEIGHT = 307.0 - 16;
    public static double CAMERA_ANGLE = -25.0;
    public static double TARGET_HEIGHT = 19.05;

    public static double strafeConversionFactor = 6.6667;
    public static double cameraStrafeToBot = -20;

    public static double sampleToRobotDistance = 145;

    private double pythonX;
    private double pythonY;
    public LimelightSubsystem(final HardwareMap hMap, Telemetry telemetry){

        this.limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // per sec;
        limelight.start();
        limelight.pipelineSwitch(startingPipeline);
        limelight.updatePythonInputs(0, 0, 0, 0, 0, 0, 0, 0);

        this.telemetry = telemetry;
    }

    public void setPipeline(int pipeline){
        limelight.stop();
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public double getTx() {
        if (result == null) {
            return -999;
        }
        return result.getTx();
    }

    public double getTy() {
        if (result == null) {
            return -999;
        }
        return result.getTy();
    }

    public double getDistance() {
        double ty = getTy();
        if (isNear(0, ty, 0.01)) {
            return 0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - sampleToRobotDistance;
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !isNear(0, result.getTa(), 0.0001);
    }

    @Override
    public void periodic() {
         result = this.limelight.getLatestResult();
        double[] result_array = result.getPythonOutput();

        if(result != null) {
            this.telemetry.addData("LL R", result.getTx());
        }
        if (result_array == null) {
            this.telemetry.addData("LL Python", "none");

            return;
        }
        if (result_array.length == 0) return;


        pythonX = result_array[1] ;
        pythonY = result_array[2];

        this.telemetry.addData("LL X", pythonX);
        this.telemetry.addData("LL Y", pythonY);
        this.telemetry.update();
    }

    public double getPythonX(){
       return pythonX;
    }
    public double getPythonY(){
       return pythonY;
    }

    public boolean isNear(double expected, double actual, double tolerance) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        return Math.abs(expected - actual) < tolerance;
    }
}
