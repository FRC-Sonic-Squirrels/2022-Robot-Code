package com.team2930.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.drive.Vector2d;

public class SwerveUtils {

/**
 * 
 * headingToPoint - return the heading to a point from the robot
 * 
 * @param robotPose
 * @param point
 * @param robotOffset - add this angle 
 * @return targetHeading 
 */
  //public static Rotation2d headingToPoint(Pose2d robotPose, Translation2d point, Rotation2d robotOffset) {

    // vector from robot to point
  //   Vector2d robotVector =
  //       new Vector2d(point.getX() - robotPose.getX(), point.getY() - robotPose.getY());

  //   Rotation2d targetHeading = getTargetHeading(robotVector, new Vector2d(1, 0));
    
  //   // add this angle to the target heading, useful if you want to aim the back of the robot at the target
  //   targetHeading.plus(robotOffset);

  //   // to make it work on left side of circle
  //   targetHeading.times(Math.signum(point.getY() - robotPose.getY()));

  //   // double distance = Math.sqrt(Math.pow(point.getX()- point.getX(), 2) + Math.pow(point.getY() -
  //   // robotPose.getY(), 2));

  //   return targetHeading;
  // }

  // /**
  //  * getTargetHeading - calculate the angle from the robot to a point using the dot product
  //  * 
  //  * @param robotLocation
  //  * @param point
  //  * @return
  //  */
  // public static Rotation2d getTargetHeading(Vector2d robotLocation, Vector2d point) {
  //   double product = robotLocation.dot(point);
  //   double magnitudes = robotLocation.magnitude() * point.magnitude();
  //   double angle_rad = Math.acos(product / magnitudes);

  //   return new Rotation2d(angle_rad);
  // }

}
