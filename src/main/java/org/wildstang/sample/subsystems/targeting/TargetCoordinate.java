package org.wildstang.sample.subsystems.targeting;

import org.wildstang.framework.core.Core;

import edu.wpi.first.math.geometry.Pose2d;

// INCHES
public class TargetCoordinate{
        private double blueX, blueY, blueHeading;
        private Pose2d position;

        public static TargetCoordinate fromPose2d(Pose2d pose2d) {
            return new TargetCoordinate(pose2d.getTranslation().getX(), pose2d.getTranslation().getY(), pose2d.getRotation().getDegrees());
        }

        public TargetCoordinate(double i_blueX, double i_blueY){
            this(i_blueX, i_blueY,  0);
        }
        public TargetCoordinate(double i_blueX, double i_blueY, double i_blueHeading){
            this.blueX = i_blueX;
            this.blueY = i_blueY;
            this.blueHeading = i_blueHeading;
        }
        public double getX(){
            return blueX;
        }
        public double getY(){
            return blueY;
        }
        public double getHeading(){
            return  blueHeading;
        }
    }
