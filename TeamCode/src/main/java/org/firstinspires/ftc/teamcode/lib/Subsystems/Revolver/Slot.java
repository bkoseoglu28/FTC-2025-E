package org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;

public class Slot {
    double angle;
    Revolver.colors color;
    boolean IsthereBall;
    public Slot(double angle){
        this.color= Revolver.colors.UNKNOWN;
        IsthereBall=false;
        this.angle =angle;
    }
    public double getAngle(){
        return angle;
    }
    public Revolver.colors getColor(){
        return color;
    }
    public boolean IsthereBall(){
        return IsthereBall;
    }
    public void setColor(Revolver.colors cl){
        this.color = cl;
    }
    public void setIsthereBall(boolean lt){
        this.IsthereBall = lt;
    }

}
