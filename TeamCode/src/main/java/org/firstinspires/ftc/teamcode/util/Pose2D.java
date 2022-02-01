package org.firstinspires.ftc.teamcode.util;

public class Pose2D {
    public double x;
    public double y;
    public double heading;

    public static Pose2D zero = new Pose2D(0,0,0);

    public Pose2D (double _x, double _y, double _heading) {
        x = _x;
        y = _y;
        heading = _heading;
    }

    public Pose2D signedDifference(Pose2D _other) {
        return new Pose2D(x- _other.x, y- _other.y, heading - _other.heading);
    }

    public Pose2D unsignedDifference(Pose2D _other) {
        Pose2D signed = signedDifference(_other);

        return new Pose2D(Math.abs(signed.x), Math.abs(signed.y), Math.abs(signed.heading));
    }

    public Pose2D add(Pose2D _other) {
        return new Pose2D(x + _other.x, y + _other.y, heading + _other.heading);
    }

    public Pose2D mult(double _factor) {
        return new Pose2D(x * _factor, y * _factor, heading * _factor);
    }

    public Pose2D mult(Pose2D _factor) {
        return new Pose2D(x *_factor.x, y * _factor.y, heading * _factor.heading);
    }

    @Override
    public String toString() {
        return "Pose2D{" +
                "x=" + x +
                ", y=" + y +
                ", heading=" + heading +
                '}';
    }
}
