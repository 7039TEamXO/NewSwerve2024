package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.Swerve;


public class DashBoard {

    Swerve swerve = new Swerve();
    private static HttpCamera limelightcamera = new HttpCamera("limelight", "http://10.70.39.11:5800");
    private static ShuffleboardTab driver = Shuffleboard.getTab("Driver");
    public static ShuffleboardTab data = Shuffleboard.getTab("Data");
    private static String m_autoSelected = "";
    private final static SendableChooser<String> m_chooser = new SendableChooser<>();
    private static GenericEntry auto_time_delay_input = driver.add("Auto delay", 0).withPosition(11, 0).withSize(3, 3)
            .getEntry();
    private static GenericEntry ampOffset = driver.add("amp offset", 0).withPosition(0, 3).withSize(3, 3)
            .getEntry();

    private static SwerveModule[] swerveModules = {
        new SwerveModule(0, Constants.Swerve.FrontLeftSwerveModule.constants),
        new SwerveModule(1, Constants.Swerve.FrontRightSwerveModule.constants),
        new SwerveModule(2, Constants.Swerve.BackLeftSwerveModule.constants),
        new SwerveModule(3, Constants.Swerve.BackRightSwerveModule.constants)
    };


public static void init() {

    data.addNumber("FrontLeftModule", () -> swerveModules[0].getCANcoder().getDegrees()).withPosition(0, 0).withSize(4, 3);
    data.addNumber("FrontRightModule", () -> swerveModules[1].getCANcoder().getDegrees()).withPosition(4, 0).withSize(4, 3);
    data.addNumber("BackLeftModule", () -> swerveModules[2].getCANcoder().getDegrees()).withPosition(8, 0).withSize(4, 3);
    data.addNumber("BackRightModule", () -> swerveModules[3].getCANcoder().getDegrees()).withPosition(12, 0).withSize(4, 3);



}
}
