package frc.robot.subsystems.swerve;

import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO create a driversubsytem folder and add it there
public class Swerve extends SubsystemBase {
    public static ShuffleboardTab dataTab = Shuffleboard.getTab("Data");
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    // public Pigeon2 gyro;
    public final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    public Swerve() {
        // gyro.getConfigurator().apply(new Pigeon2Configuration());

        gyro.reset();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.FrontLeftSwerveModule.constants),
                new SwerveModule(1, Constants.Swerve.FrontRightSwerveModule.constants),
                new SwerveModule(2, Constants.Swerve.BackLeftSwerveModule.constants),
                new SwerveModule(3, Constants.Swerve.BackRightSwerveModule.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        initShuffleboard();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        // System.out.println("gyro " + getGyroYaw());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    // added temperery 180 degrees to reset angle to make the forward the shooter
    public void zeroHeading() {
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(-gyro.getYaw() + 180), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new
        // Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
    }

    private void initShuffleboard() {
        dataTab.addNumber("FrontLeftModule", () -> mSwerveMods[0].getCANcoder().getDegrees()).withPosition(0, 0)
                .withSize(4, 3);
        dataTab.addNumber("FrontRightModule", () -> mSwerveMods[1].getCANcoder().getDegrees()).withPosition(4, 0)
                .withSize(4, 3);
        dataTab.addNumber("BackLeftModule", () -> mSwerveMods[2].getCANcoder().getDegrees()).withPosition(8, 0)
                .withSize(4,
                        3);
        dataTab.addNumber("BackRightModule", () -> mSwerveMods[3].getCANcoder().getDegrees()).withPosition(12, 0)
                .withSize(4, 3);

    }

}