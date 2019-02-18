package org.firstinspires.ftc.teamcode;

public class Vector {
    public double x;
    public double y;
    public double r;
    public double theta;

    public  enum  VectorType {
        CARTESIAN, POLAR
    }

    public Vector(double x, double y)
    {
        this.x = x;
        this.y = y;
        this.r = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
        this.theta = Math.atan(y/x);
    }

    public Vector(double a, double b,VectorType vType)
    {
        this.x = vType == VectorType.CARTESIAN ? a : a*Math.cos(b);
        this.y = vType == VectorType.CARTESIAN ? b : a*Math.sin(b);
        this.r = vType == VectorType.CARTESIAN ? Math.sqrt(Math.pow(a,2)+Math.pow(b,2)) : a;
        this.theta = vType == VectorType.CARTESIAN ? Math.atan(b/a) : b;
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
