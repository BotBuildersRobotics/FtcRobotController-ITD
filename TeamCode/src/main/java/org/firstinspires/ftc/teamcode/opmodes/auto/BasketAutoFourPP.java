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
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.AscentOpenHooksCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.groups.AutoIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.SubIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.PinpointDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "RED BASKET | FOUR | SUB", group = "Autonomous")
public class BasketAutoFourPP extends CommandOpMode {

    TrajectoryActionBuilder dropOffPreload;
    TrajectoryActionBuilder apSample; //ap = alliance preload

    TrajectoryActionBuilder apSlowMoveIn;
    TrajectoryActionBuilder deliverAPSample;
    TrajectoryActionBuilder deliverAPSampleMoveIn;

    TrajectoryActionBuilder firstSample;

    TrajectoryActionBuilder firstSampleSlowMoveIn;

    TrajectoryActionBuilder firstSampleDeliver;

    TrajectoryActionBuilder firstSampleDeliverIn;
    TrajectoryActionBuilder secondSample;
    TrajectoryActionBuilder secondSampleSlowMoveIn;
    TrajectoryActionBuilder deliverSecondSample;
    TrajectoryActionBuilder deliverSecondSampleMoveIn;
    TrajectoryActionBuilder thirdSample;
    TrajectoryActionBuilder thirdSampleSlowMoveIn;
    TrajectoryActionBuilder deliverThirdSample;

    TrajectoryActionBuilder deliverThirdSampleMoveIn;
    TrajectoryActionBuilder firstSub;

    TrajectoryActionBuilder firstSubDeliver;


    Action pleaseWork;
    IntakeSubsystem intakeSubsystem;

    TransferSubsystem transferSubsystem;

    RobotStateSubsystem robotState;

    SlidesSubsystem slidesSubsystem;

    AscentSubsystem ascentSubsystem;

    //Change these offsets, they can be negative values
    int X_OFFSET = 0; // a larger negative number takes it closer to the basket
    int Y_OFFSET = 0; // a larger number takes it closer to the submersible

    @Override
    public void initialize() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        robotState = new RobotStateSubsystem();
        slidesSubsystem = new SlidesSubsystem(hardwareMap);
        ascentSubsystem = new AscentSubsystem(hardwareMap);

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.RED_OR_NEUTRAL);


        // instantiate your MecanumDrive at a particular pose.
        PinpointDrive drive = new PinpointDrive(hardwareMap,
                new Pose2d(-48, -64, Math.toRadians(0)));

        //pose to the submersible wall
        //X is up and down the wall
        //Y is into the submersible
        Pose2d dropOffPose = new Pose2d(-69, -52, Math.toRadians(-290));

        dropOffPreload = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(dropOffPose, Math.toRadians(-290))
                .endTrajectory();



        Pose2d firstSamplePose = new Pose2d(-72, -52, Math.toRadians(-295));


        firstSample = dropOffPreload.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstSamplePose, Math.toRadians(-295))
                .endTrajectory();

        Pose2d firstSampleSlowMoveInPose = new Pose2d(-65, -49, Math.toRadians(-290));

        firstSampleSlowMoveIn = firstSample.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstSampleSlowMoveInPose, Math.toRadians(-290), new TranslationalVelConstraint(5))
                .endTrajectory();

        Pose2d firstSampleDeliverPose = new Pose2d(-64 + X_OFFSET,-52 + Y_OFFSET,Math.toRadians(-305));

        firstSampleDeliver = firstSampleSlowMoveIn.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstSampleDeliverPose, Math.toRadians(-305))
                .endTrajectory();


        Pose2d secondSamplePose = new Pose2d(-65 + X_OFFSET,-59 + Y_OFFSET,Math.toRadians(100));


        secondSample = firstSampleDeliver.fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(secondSamplePose,Math.toRadians(90))
                .endTrajectory();

        Pose2d secondSampleMoveInPose = new Pose2d(-65 + X_OFFSET,-50 + Y_OFFSET,Math.toRadians(100));


        secondSampleSlowMoveIn = secondSample.fresh()
                .splineToLinearHeading(secondSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(5))
                .endTrajectory();

        Pose2d deliverSecondSamplePose = new Pose2d(-68 + X_OFFSET,-51 + Y_OFFSET,Math.toRadians(-290));


        deliverSecondSample = secondSampleSlowMoveIn.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(deliverSecondSamplePose,Math.toRadians(-290))
                .endTrajectory();



        Pose2d thirdSamplePose = new Pose2d(-67 + X_OFFSET,-57 + Y_OFFSET,Math.toRadians(-242));

        thirdSample = deliverSecondSample.fresh()
                .setTangent(Math.toRadians(103))
                .splineToLinearHeading(thirdSamplePose,Math.toRadians(103))
                .endTrajectory();

        Pose2d thirdSampleMoveInPose = new Pose2d(-69 + X_OFFSET,-48 + Y_OFFSET,Math.toRadians(-242));

        thirdSampleSlowMoveIn = thirdSample.fresh()
                .splineToLinearHeading(thirdSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(6))
                .endTrajectory();

        Pose2d deliverThirdMovePose = new Pose2d(-64 + X_OFFSET,-52 + Y_OFFSET,Math.toRadians(-315));

        deliverThirdSample = thirdSampleSlowMoveIn.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(deliverThirdMovePose , Math.toRadians(-315))
                .endTrajectory();


        firstSub = deliverThirdSample.fresh()
                .setTangent(Math.toRadians(90))
               // .splineToLinearHeading(new Pose2d(-10,-10,Math.toRadians(180)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-26, -0, Math.toRadians(0)), Math.toRadians(0))
                .endTrajectory();

        Pose2d firstBackMove = new Pose2d(-46, -10, Math.toRadians(0));

        Pose2d firstSubDeliverPose = new Pose2d(-64, -52, Math.toRadians(-315));

        firstSubDeliver = firstSub.fresh()
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(firstSubDeliverPose, Math.toRadians(-315), new TranslationalVelConstraint(100))
                        .endTrajectory();

        slidesSubsystem.setTelemtary(telemetry);
        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.RED_OR_NEUTRAL);
        intakeSubsystem.intakePivotDown();
        transferSubsystem.closeGrippler();
        ascentSubsystem.PTODriveEnabled();


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                    new SequentialCommandGroup(

                            new SequentialCommandGroup(
                                    new CloseGripplerCommand(transferSubsystem),
                                            // do the drop off if we have the sample
                                            new SequentialCommandGroup(

                                                    new ActionCommand(dropOffPreload.build(), new ArraySet<>()),
                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState ),
                                                    new WaitCommand(200), //give the slides time to move up

                                                    new OpenGripplerCommand(transferSubsystem),
                                                    new WaitCommand(250)

                                            )

                            ),

                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new WaitCommand(500),
                                            new ActionCommand(firstSample.build(), new ArraySet<>())
                                    ),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new WaitCommand(500),
                                            new InstantCommand(intakeSubsystem::intakeSlidesOut) // new IntakeSlidesOutCommand(intakeSubsystem),

                                    )
                            ),
                           // new SequentialCommandGroup(

                                    /*new ParallelCommandGroup(
                                            new SequentialCommandGroup(
                                                    new WaitCommand(900),
                                                    new InstantCommand(intakeSubsystem::intakePivotDown)//,
                                        //            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                            )//,
                                      //      new ActionCommand(firstSampleSlowMoveIn.build(), new ArraySet<>())
                                    ),*/
                                    new ParallelCommandGroup(
                                            new SequentialCommandGroup(
                                                    new WaitCommand(400),
                                                    new InstantCommand(intakeSubsystem::intakePivotDown),
                                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000),
                                                    new InstantCommand(intakeSubsystem::slowIntake)
                                            ),
                                            new ActionCommand(firstSampleSlowMoveIn.build(), new ArraySet<>())
                                    ),

                            //),
                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),

                            new SequentialCommandGroup(

                                    // do the drop off if we have the sample
                                    new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(400), //time for the slides
                                                                new ActionCommand(firstSampleDeliver.build(), new ArraySet<>()
                                                                )
                                                        ),
                                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )

                                            ),
                                            new OpenGripplerCommand(transferSubsystem),
                                            new WaitCommand(250)
                                    )

                            ),
                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                        new WaitCommand(500), //wait for robot to move away
                                        new ActionCommand(secondSample.build(), new ArraySet<>())
                                    ),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new InstantCommand(intakeSubsystem::intakeSlidesOut), // new IntakeSlidesOutCommand(intakeSubsystem),
                                            new InstantCommand(intakeSubsystem::intakePivotDown) // new IntakePivotDownCommand(intakeSubsystem, robotState)
                                    )
                            ),
                           // new SequentialCommandGroup(

                                    /*new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(900)//,
                                           // new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                        )//,
                                      //  new ActionCommand(secondSampleSlowMoveIn.build(), new ArraySet<>())
                                    ),*/
                                    new ParallelCommandGroup(
                                            new SequentialCommandGroup(
                                                    new WaitCommand(900),
                                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000),
                                                    new InstantCommand(intakeSubsystem::slowIntake)
                                            ),
                                            new ActionCommand(secondSampleSlowMoveIn.build(), new ArraySet<>())
                                    ),

                            //),


                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),



                            new SequentialCommandGroup(
                                    new CloseGripplerCommand(transferSubsystem),

                                    // do the drop off if we have the sample
                                    new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    new SequentialCommandGroup(
                                                            new WaitCommand(400),
                                                            new ActionCommand(deliverSecondSample.build(), new ArraySet<>())
                                                            ),
                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
                                            ),
                                            new OpenGripplerCommand(transferSubsystem),
                                            new WaitCommand(350)
                                    )


                            ),


                            new ParallelCommandGroup(

                                    new SequentialCommandGroup(
                                            new WaitCommand(500),
                                            new ActionCommand(thirdSample.build(), new ArraySet<>())
                                    ),
                                    new SequentialCommandGroup(
                                            new WaitCommand(200), //give time to move from the basket
                                            new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
                                    )
                            ),
                            //third sample


                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new IntakeSlidesOutCommand(intakeSubsystem),
                                            new InstantCommand(intakeSubsystem::intakePivotDown)//,
                                           // new WaitCommand(500)//,
                                         //   new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                    )//,

                                   // new ActionCommand(thirdSampleSlowMoveIn.build(), new ArraySet<>())

                            ),
                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new WaitCommand(100),
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000),
                                            new InstantCommand(intakeSubsystem::slowIntake)
                                    ),
                                    new ActionCommand(thirdSampleSlowMoveIn.build(), new ArraySet<>())
                            ),
                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),

                            new SequentialCommandGroup(
                                    new ParallelCommandGroup(

                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState ),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(800),
                                                new ActionCommand(deliverThirdSample.build(), new ArraySet<>())

                                            )
                                    ),
                                   // new ActionCommand(deliverThirdSampleMoveIn.build(), new ArraySet<>()),
                                    new OpenGripplerCommand(transferSubsystem),
                                    new WaitCommand(250)
                            ),
                           // new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),

                            new ParallelCommandGroup(
                                    new ActionCommand(firstSub.build(), new ArraySet<>()),
                                    new SequentialCommandGroup(
                                        new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                        new IntakePivotUpCommand(intakeSubsystem,robotState)

                                    )

                            ),

                            //we are in the submersible now


                            new InstantCommand(()->{

                                PoseStorage.currentPose = new Pose2d(0, 0, Math.toRadians(90));

                            }),
                            new SubIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new InstantCommand(()->{
                                                intakeSubsystem.TelemeteryMessage("Going back to sub, found sample");
                                            }),
                                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                            new IntakePivotUpCommand(intakeSubsystem, robotState),
                                            new ParallelCommandGroup(
                                                new ActionCommand(firstSubDeliver.build(), new ArraySet<>()),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(600), //give the bot time to move
                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState ) // do the delivery
                                                )
                                            ),
                                            new WaitCommand(250),
                                            new OpenGripplerCommand(transferSubsystem),
                                            new WaitCommand(250),
                                            new SequentialCommandGroup(
                                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                                    new IntakePivotUpCommand(intakeSubsystem,robotState)

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
                )
        );

    }

}
