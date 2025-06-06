package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.AutoColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SampleAlign;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.SubIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.PinpointDrive;

@Autonomous(name = "Test Sub Collection", group = "Autonomous")
@Disabled
public class SubCollectionAuto extends CommandOpMode {

    IntakeSubsystem intakeSubsystem;

    TransferSubsystem transferSubsystem;

    RobotStateSubsystem robotState;

    SlidesSubsystem slidesSubsystem;

    @Override
    public void initialize() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        slidesSubsystem = new SlidesSubsystem(hardwareMap);
        robotState = new RobotStateSubsystem();

        // instantiate your MecanumDrive at a particular pose.
        PinpointDrive drive = new PinpointDrive(hardwareMap,
                new Pose2d(-14.8, -64, Math.toRadians(-90)));

        //pose to the submersible wall

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.RED_OR_NEUTRAL);


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                   new SubIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new InstantCommand(()->{
                                        intakeSubsystem.TelemeteryMessage("Going back to sub, found sample");
                                    }),
                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState ), // do the delivery
                                    new OpenGripplerCommand(transferSubsystem),
                                    new WaitCommand(250),
                                    new SequentialCommandGroup(
                                            new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                            new IntakePivotUpCommand(intakeSubsystem,robotState)

                                    ),
                                    new InstantCommand(), //move back to the sub
                                    new SubIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new InstantCommand(()->{
                                                        intakeSubsystem.TelemeteryMessage("Going back to sub, found 2nd sample");
                                                    }),
                                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState)

                                            ),

                                            new SequentialCommandGroup( //nothing found - just end in a good state
                                                    new IntakePivotUpCommand(intakeSubsystem, robotState),
                                                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)
                                            ),
                                            intakeSubsystem::hasItemInIntake
                                    )

                            ), // clean up and run back to the sub
                            new SequentialCommandGroup(
                                    new IntakePivotUpCommand(intakeSubsystem, robotState),
                                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                                    new InstantCommand(() ->{
                                        intakeSubsystem.TelemeteryMessage("moving location");
                                    }),
                                    new SubIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState)

                            )
                            , // we failed a pick .. what now?
                            intakeSubsystem::hasItemInIntake
                    )
                )
        );

    }

}
