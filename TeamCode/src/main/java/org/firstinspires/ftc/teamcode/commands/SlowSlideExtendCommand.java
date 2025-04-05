package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class SlowSlideExtendCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;

    private final TransferSubsystem transferSubsystem;

    private ElapsedTime timer;

    private int steps = 0;

    public SlowSlideExtendCommand(IntakeSubsystem subsystem, TransferSubsystem transfer) {
        intakeSubsystem = subsystem;
        transferSubsystem = transfer;
        timer = new ElapsedTime();
        timer.reset();
        steps = 0;
        // Use addRequirements() here to declare subsystem dependencies.
       // addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        intakeSubsystem.intakeSlidesIn();
    }

    @Override
    public void execute() {

        if(timer.milliseconds() > 50){
            intakeSubsystem.IncrSlides();
            steps++;
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {

        if(steps > 120 || intakeSubsystem.hasItemInIntake()){
            return true;
        }
        //we need to check to see if our time allotment is done.
        return false;

    }
}
