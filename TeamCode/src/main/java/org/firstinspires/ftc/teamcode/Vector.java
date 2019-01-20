package org.firstinspires.ftc.teamcode;

public class Vector {
    public double x;
    public double y;

    public Vector(double x, double y)
    {
        this.x = x;
        this.y = y;
    }
    //Counterclockwise is positive, clockwise is negative
    //must be in radians
    public void rotate(double θ) {
        double x0 = this.x;
        double y0 = this.y;

        this.x = x0*Math.cos(θ)-y0*Math.sin(θ);
        this.y = x0*Math.sin(θ)+y0*Math.cos(θ);
    }
}
