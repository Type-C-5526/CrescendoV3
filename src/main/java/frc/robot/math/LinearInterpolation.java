// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.math;

import java.util.Collections;
import java.util.List;

/** Add your docs here. */
public class LinearInterpolation {

    private List<Point> m_Points;

    public LinearInterpolation(List<Point> points){
        this.m_Points = points;
    }
 
    public double interpolate(double x){
        Collections.sort(m_Points);

        // Encontrar los puntos entre los cuales se encuentra x
        for (int i = 0; i < m_Points.size() - 1; i++) {
            Point p0 = m_Points.get(i);
            Point p1 = m_Points.get(i + 1);
            if (p0.x <= x && x <= p1.x) {
                // Interpolación lineal
                return p0.y + (p1.y - p0.y) * ((x - p0.x) / (p1.x - p0.x));
            }
        }

        return Double.NaN; 
    }
}

