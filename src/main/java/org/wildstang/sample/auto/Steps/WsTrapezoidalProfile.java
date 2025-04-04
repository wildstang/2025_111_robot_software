package org.wildstang.sample.auto.Steps;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsTrapezoidalProfile {

    // Default values, can be specified in constructor
    private double MAX_ACCEL = 1;
    private double MAX_DECEL = -1; // Should be negative
    private double MAX_VELOCITY = 1;

    private double distance;
    private boolean isTriangle;
    private double constVelocityStart;
    private double decelerationStart;
    private double end;
    private double accelerationDist;

    /**
     * Creates a trapezoidal motion profile to travel a certain positive distance
     * Will use triangle profile if the distance is too short to reach max velocity
     * @param distance Positive value
     * @param maxAcceleration
     * @param maxDeceleration Negative value
     * @param maxVelocity
     */
    public WsTrapezoidalProfile(double distance, double maxAcceleration, double maxDeceleration, double maxVelocity) {
        MAX_ACCEL = maxAcceleration;
        MAX_DECEL = maxDeceleration;
        MAX_VELOCITY = maxVelocity;
        this.distance = distance;
        init();
    }

    /**
     * Uses defualt values for acceleration, deceleration, and max velocity
     * @param distance
     */
    public WsTrapezoidalProfile(double distance) {
        this.distance = distance;
        init();
    }

    private void init(){

        // Calculate distances traveled when accelerating to or decelerating from max velocity
        accelerationDist = Math.pow(MAX_VELOCITY, 2) / (2 * MAX_ACCEL);
        double decelerationDist = Math.pow(MAX_VELOCITY, 2) / (2 * -MAX_DECEL); // Negative so you get a positive distance

        // Full trapezoidal profile with constant velocity time
        if (accelerationDist + decelerationDist < distance) {
            constVelocityStart = MAX_VELOCITY / MAX_ACCEL;
            decelerationStart = constVelocityStart + (distance - (accelerationDist + decelerationDist)) / MAX_VELOCITY;
            end = decelerationStart + MAX_VELOCITY / -MAX_DECEL;
            isTriangle = false;
        } else {
            end = Math.sqrt((2 * distance * (MAX_DECEL - MAX_ACCEL)) / (MAX_DECEL * MAX_ACCEL));
            decelerationStart = (MAX_DECEL * end) / (MAX_DECEL - MAX_ACCEL);
            accelerationDist = Math.pow(decelerationStart * MAX_ACCEL, 2) / (2 * MAX_ACCEL);
            isTriangle = true;
        }
    }

    public double getFinalTIme() {
        return end;
    }

    public double getDistance() {
        return distance;
    }

    public State calculateState(double time) {
        if (isTriangle) {

            double maxVelocity = (MAX_DECEL * end * MAX_ACCEL) / (MAX_DECEL - MAX_ACCEL);

            // In accelerating portion
            if (time < decelerationStart) {
                SmartDashboard.putString("Profile state", "accelerating");

                return new State(time * MAX_ACCEL, MAX_ACCEL, Math.pow(time * MAX_ACCEL, 2) / (2 * MAX_ACCEL));

            // In decelerating portion
            } else if (time < end) {
                SmartDashboard.putString("Profile state", "decelerating");

                return new State(maxVelocity + (time - decelerationStart) * MAX_DECEL, MAX_DECEL, accelerationDist + maxVelocity * (time - decelerationStart) + MAX_DECEL * Math.pow((time - decelerationStart), 2) / 2);
            } else {
                return new State(0,0,distance);
            }
        } else {

            // In accelerating portion
            if (time < constVelocityStart) {

                return new State(time * MAX_ACCEL, MAX_ACCEL, Math.pow(time * MAX_ACCEL, 2) / (2 * MAX_ACCEL));

            // In constant velocity portion
            } else if (time < decelerationStart) {
                SmartDashboard.putString("Profile state", "constant velocity");

                return new State(MAX_VELOCITY, 0, accelerationDist + (time - constVelocityStart) * MAX_VELOCITY);

            // In decelerating portion
            } else if (time < end) {
                return new State(MAX_VELOCITY + (time - decelerationStart) * MAX_DECEL, MAX_DECEL, accelerationDist + (decelerationStart - constVelocityStart) * MAX_VELOCITY + MAX_VELOCITY * (time - decelerationStart) + MAX_DECEL * Math.pow((time - decelerationStart), 2) / 2);
            } else {
                return new State(0,0,distance);

            }
        }
    }

    public record State(double velocity, double acceleration, double distance) {}

}
