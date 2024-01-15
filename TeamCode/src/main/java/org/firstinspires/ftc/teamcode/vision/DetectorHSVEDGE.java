package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectorHSVEDGE extends OpenCvPipeline {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    public Scalar lower = new Scalar(16, 70, 0);
    public Scalar upper = new Scalar(28, 255, 255);
    public int minsize = 200;
    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */

    private Mat blurredIn = new Mat();
    private Mat HSVmat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat edges = new Mat();
    private Mat gray = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private Mat maskedInputMat = new Mat();
    private Scalar color = new Scalar(0, 120, 0);
    private Mat element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(20,20));
    private Telemetry telemetry = null;
    private int detections = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.GaussianBlur(input, blurredIn, new Size(5, 5), 0);

        Imgproc.cvtColor(blurredIn, HSVmat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(HSVmat, lower, upper, binaryMat);

        Imgproc.erode(binaryMat, binaryMat, element1);
        Imgproc.dilate(binaryMat, binaryMat, element1);

        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        Imgproc.cvtColor(maskedInputMat, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(gray, gray, 0, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(gray, gray, 0,255,Imgproc.THRESH_OTSU);

        contours.clear();
        Imgproc.findContours(gray, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        detections = contours.size();
        detections = 0;
        for (int i = 0; i < contours.size(); i++) {
            Mat contour = contours.get(i);
            Rect rect = Imgproc.boundingRect(contour);
            if(Imgproc.contourArea(contour) > minsize){
                Imgproc.rectangle (maskedInputMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), color, 2);
                int distance = (rect.x + (rect.width/2)) - (input.width()/2);
                if(distance < -370){
                    detections = 1;
                }else if(distance > 150){
                    detections = 3;
                }else{
                    detections = 2;
                }
                Imgproc.putText(maskedInputMat, "Dist: " + distance, new Point(10, 50),Imgproc.FONT_HERSHEY_SIMPLEX,1,color,3);
            }

        }


        return maskedInputMat;
    }
    public int getAnalysis()
    {
        return detections;
    }

}