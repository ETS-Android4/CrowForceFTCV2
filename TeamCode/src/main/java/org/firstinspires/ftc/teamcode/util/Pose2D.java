package org.firstinspires.ftc.teamcode.util;

public class Pose2D {
    public double x;
    public double y;
    public double heading;

    public Pose2D (double _x, double _y, double _heading) {
        x = _x;
        y = _y;
        heading = _heading;
    }

    public Pose2D absDifference(Pose2D _other) {
        return new Pose2D(x- _other.x, y- _other.y, heading - _other.heading);
    }
}
