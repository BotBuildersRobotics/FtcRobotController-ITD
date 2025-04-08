package org.firstinspires.ftc.teamcode.opmodes.auto;
import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.AscentOpenHooksCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SampleAlign;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.OTOSDrive;
import org.firstinspires.ftc.teamcode.utils.PinpointDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "SampleAlign", group = "Autonomous")
@Disabled
public class SampleAlignAuto extends CommandOpMode {

    IntakeSubsystem intakeSubsystem;

    LimelightSubsystem limelightSubsystem;


    @Override
    public void initialize() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry);


        // instantiate your MecanumDrive at a particular pose.
        PinpointDrive drive = new PinpointDrive(hardwareMap,
                new Pose2d(-14.8, -64, Math.toRadians(-90)));

        //pose to the submersible wall

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.NEUTRAL);
        //intakeSubsystem.intakePivotDown();

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                    new RepeatCommand(
                        new SampleAlign(drive, limelightSubsystem, telemetry)
                    )
                )
        );

    }

}
