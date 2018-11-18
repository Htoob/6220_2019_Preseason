package frc.team6220.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team6220.robot.RobotMap;
import frc.team6220.robot.commands.TeleopDrive;

public class Intake extends Subsystem {

    private TalonSRX Intake1 = new TalonSRX(RobotMap.Intake1);
    private TalonSRX Intake2 = new TalonSRX(RobotMap.Intake2);

    public Intake(){
        Intake1.setNeutralMode(NeutralMode.Brake);
        Intake2.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TeleopDrive());
    }

    public void teleopDrive(XboxController xbox){
        if (xbox.getAButton()) {
            Intake1.set(ControlMode.PercentOutput, (.8));
            Intake2.set(ControlMode.PercentOutput, (-.8));
        } else if (xbox.getYButton()) {
            Intake1.set(ControlMode.PercentOutput, (-.75));
            Intake2.set(ControlMode.PercentOutput, (.75));
        } else {
            Intake1.set(ControlMode.PercentOutput, (0));
            Intake2.set(ControlMode.PercentOutput, (0));
        }
    }
}
