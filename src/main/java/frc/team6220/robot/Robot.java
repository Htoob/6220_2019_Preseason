package frc.team6220.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6220.robot.commands.AutoCommands;
import frc.team6220.robot.commands.TeleopDrive;
import frc.team6220.robot.subsystems.DriveTrain;
import frc.team6220.robot.subsystems.Elevator;
import frc.team6220.robot.subsystems.Intake;

public class Robot extends IterativeRobot {
    Command auto;
    Command teleop;

    public static DriveTrain DriveTrain;
    public static Elevator Elevator;
    public static Intake Intake;
    public static OI oi;

    @Override
    public void robotInit() {
        DriveTrain = new DriveTrain();
        Elevator = new Elevator();
        Intake = new Intake();
        oi = new OI();
        auto = new AutoCommands();
        teleop = new TeleopDrive();
        CameraServer.getInstance().startAutomaticCapture();
        SmartDashboard.putData(DriveTrain);
        LiveWindow.add(Robot.DriveTrain.gyro);
    }

    @Override
    public void autonomousInit() {
        auto.start();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        LiveWindow.updateValues();
    }

    @Override
    public void teleopInit() {
        auto.cancel();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
        LiveWindow.updateValues();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }
}
