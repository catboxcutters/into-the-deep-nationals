package drive.writtenCode.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

public class pipeline extends OpenCvPipeline {
    private volatile double rectangleAngle = 0;  // Make it volatile for thread safety

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define blue color range
        Scalar lowerBlue = new Scalar(100, 150, 10);
        Scalar upperBlue = new Scalar(140, 255, 255);

        // Mask the blue color
        Mat mask = new Mat();
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Define a minimum area threshold
        double minArea = 500;

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) < minArea) {
                continue;
            }

            // Approximate the contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            // Check if the approximated polygon has 4 vertices
            if (approxCurve.total() == 4) {
                // Get the minimum area rectangle
                RotatedRect rect = Imgproc.minAreaRect(contour2f);

                // Store the angle of the rectangle
                rectangleAngle = rect.angle;
                if (rectangleAngle < -45) {
                    rectangleAngle += 90;
                }

                // Draw the rectangle
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                MatOfPoint box = new MatOfPoint(boxPoints);
                List<MatOfPoint> boxList = new ArrayList<>();
                boxList.add(box);
                Imgproc.drawContours(input, boxList, -1, new Scalar(0, 255, 0), 2);

                // Draw the center point
                Imgproc.circle(input, rect.center, 5, new Scalar(255, 0, 0), -1);
            }
        }

        // Release Mats to free memory
        hsv.release();
        mask.release();
        hierarchy.release();

        return input;  // Return the processed frame
    }

    public double getRectangleAngle() {
        return rectangleAngle;
    }
}
