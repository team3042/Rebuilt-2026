package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class RetractInake extends Command {

  private final Intake intake;
  private final double power;

  public RetractInake(double pow) {
    intake = Robot.intake;
    addRequirements(intake);

    power = pow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(intake.getIntakeMotorPosition() < 0) {
      SmartDashboard.putNumber("Actual intake pos", intake.getIntakeMotorPosition());
      intake.powerToIntakeIn(power);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.powerToIntakeOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIntakeMotorPosition() >= 0;
  }
}
