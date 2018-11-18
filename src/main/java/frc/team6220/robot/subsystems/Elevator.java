package frc.team6220.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team6220.robot.RobotMap;
import frc.team6220.robot.commands.TeleopDrive;

public class Elevator extends Subsystem {

    private TalonSRX Elevator1 = new TalonSRX(RobotMap.Elevator1);
    private TalonSRX Elevator2 = new TalonSRX(RobotMap.Elevator2);

    public Elevator(){
        Elevator1.setNeutralMode(NeutralMode.Brake);
        Elevator2.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TeleopDrive());
    }

    public void teleopDrive(XboxController xbox){
        Elevator1.set(ControlMode.PercentOutput, .8 * (-xbox.getY(GenericHID.Hand.kLeft)));
        Elevator2.set(ControlMode.PercentOutput, .8 * (-xbox.getY(GenericHID.Hand.kLeft)));
    }

}
