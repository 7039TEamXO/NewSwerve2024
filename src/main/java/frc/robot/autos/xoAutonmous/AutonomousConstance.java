package frc.robot.autos.xoAutonmous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;

public class AutonomousConstance {
    class AutoTrejectorys{
        public final AutoPoint[] exampelAuto = {
            createAutoPoint(1, 0, 0, RobotState.TRAVEL),
            createAutoPoint(2, 0, 0, RobotState.TRAVEL, 2)
        };



        private static AutoPoint createAutoPoint(float x, float y, float angle, RobotState state){
            return new AutoPoint(new Pose2d(x,y,new Rotation2d(angle)), state);
        }

        private static AutoPoint createAutoPoint(float x, float y, float angle, RobotState state, float maxVel){
            return new AutoPoint(new Pose2d(x,y,new Rotation2d(angle)), state, maxVel);
        }
    }
}
