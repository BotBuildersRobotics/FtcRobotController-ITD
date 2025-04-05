package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class RobotStateSubsystem extends SubsystemBase {

  public enum SlideHeight{
      STOW,
      LOW,
      HIGH
  }

  public enum PivotState{
      LOW,
      HIGH
  }

  public enum PTOState{
      ENABLED,
      DISABLED
  }

  public SlideHeight slidePosition = SlideHeight.STOW;

  public PivotState pivotPosition = PivotState.HIGH;

  public PTOState ptoState = PTOState.DISABLED;


}
