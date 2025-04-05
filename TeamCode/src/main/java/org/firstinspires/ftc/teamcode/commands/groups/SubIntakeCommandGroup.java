package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.AutoColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.PoopChuteCloseCommand;
import org.firstinspires.ftc.teamcode.commands.SlowIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlowSlideExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class SubIntakeCommandGroup extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    private final RobotStateSubsystem robotState;

    public SubIntakeCommandGroup(IntakeSubsystem Isubsystem, TransferSubsystem Itransfer, RobotStateSubsystem state) {
        intakeSubsystem = Isubsystem;
        transferSubsystem = Itransfer;
        robotState = state;

        addCommands(
                new IntakePivotDownCommand(intakeSubsystem, robotState),
                new ParallelRaceGroup(
                    new AutoColourAwareIntakeCommand(intakeSubsystem),
                        new SlowSlideExtendCommand(intakeSubsystem, transferSubsystem)
                ),
                new IntakeOffCommand(intakeSubsystem)




        );


        addRequirements(Isubsystem);
        addRequirements(Itransfer);
    }
}