// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PoseHelper {
    private final Translation2d m_pose1;
    private final Translation2d m_pose2;

    public PoseHelper(Translation2d _pose1, Translation2d _pose2){
        this.m_pose1 = _pose1;
        this.m_pose2 = _pose2;
    }

    public double DistanceBetweenPoses(){
        return Math.sqrt(Math.pow((m_pose2.getY()-m_pose1.getY()),2) + Math.pow((m_pose2.getX()-m_pose1.getX()),2));

    }

    public double DiffYBetweenPoses(){
        return (m_pose2.getY()-m_pose1.getY());
    }
    
    public double DiffXBetweenPoses(){
        return (m_pose2.getX()-m_pose1.getX());
    }

    public double AngleBetweenPoses(){
        
        return Math.toDegrees(
            Math.asin((m_pose2.getY()-m_pose1.getY())/DistanceBetweenPoses())
        );
    }
}
