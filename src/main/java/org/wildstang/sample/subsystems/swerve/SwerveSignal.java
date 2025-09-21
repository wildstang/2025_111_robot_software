package org.wildstang.sample.subsystems.swerve;

public class SwerveSignal {
    private double verticalSpeed = 0;
    private double horizontalSpeed = 0;
    private double rotationSpeed = 0;
    private double rotationTarget = 0;
    private boolean rotLocked = false;

    public SwerveSignal(){
        this(0,0,0);
    }
    public SwerveSignal(double vert, double hori, double rot){
        this.verticalSpeed = vert;
        this.horizontalSpeed = hori;
        this.rotationSpeed = rot;
    }
    public void setTranslation(double vert, double hori){
        this.verticalSpeed = vert;
        this.horizontalSpeed = hori;
    }
    public void setFreeRotation(double rotation){
        rotLocked = false;
        rotationSpeed = rotation;
    }
    public void setRotLocked(double rotation){
        rotLocked = true;
        rotationTarget = rotation;
    }
    public double getVertical(){ 
        return verticalSpeed;
    }
    public double getHorizontal(){ 
        return horizontalSpeed;
    }
    public double getRotation(){
        return rotationSpeed;
    }
    public double getRotTarget(){
        return rotationTarget;
    }
    public boolean isRotLocked(){
        return rotLocked;
    }
}
