package org.firstinspires.ftc.teamcode;

public class Wait {
    public static void waitTime(double ms) {
        long start = System.nanoTime();

        while ((System.nanoTime() - start) / 1000000 < ms) {

        }
    }
}