package org.firstinspires.ftc.teamcode.util;

public class TranslateEncoders {
    public static double tpi;

    public static enum DIRECTIONS {
        UP(1, 0),
        RIGHT(0, 1),
        DOWN(-1, 0),
        LEFT(0, -1);

        public double x;
        public double y;

        private DIRECTIONS(double _x, double _y) {
            this.x = _x;
            this.y = _y;
        }
    }

    public static void translate(double _distance, DIRECTIONS _direction) {

    }

    public static void setTicksPerInch(double _tpi) {
        tpi = _tpi;
    }

}