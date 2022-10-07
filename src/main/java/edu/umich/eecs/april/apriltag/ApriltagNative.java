package edu.umich.eecs.april.apriltag;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class ApriltagNative {
    static {
        System.loadLibrary("apriltag");
        native_init();
    }

    public static native void native_init();

    public static native void yuv_to_rgb(byte[] src, int width, int height, Long dst_ptr);

    public static native void apriltag_init(String tagFamily, int errorBits, double decimateFactor,
                                            double blurSigma, int nthreads);

    public static native ArrayList<ApriltagDetection> apriltag_detect_yuv(byte[] src, int width, int height);
}
