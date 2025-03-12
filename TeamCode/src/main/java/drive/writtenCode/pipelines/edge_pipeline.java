package drive.writtenCode.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class edge_pipeline extends OpenCvPipeline {

    private volatile double orientationAngle = 0;  // Angle of the detected object
    private boolean showMaskedImage = false;      // Toggle between original and masked image
    private boolean detectBlue = true;           // Toggle to switch between blue and yellow detection

    // Reuse `Mat` objects to avoid creating new ones in each frame
    private final Mat hsv = new Mat();
    private Mat cropped = new Mat();
    private final Mat colorMask = new Mat();
    private final Mat edges = new Mat();
    private final Mat lines = new Mat();

    // ROI dimensions (percentages of the image)
    private final double roiTopPercent = 0.25;   // Top edge of the ROI (25% from the top)
    private final double roiBottomPercent = 0.75; // Bottom edge of the ROI (75% from the top)
    private final double roiLeftPercent = 0.25;  // Left edge of the ROI (25% from the left)
    private final double roiRightPercent = 0.75; // Right edge of the ROI (75% from the left)

    @Override
    public Mat processFrame(Mat input) {
        try {
            // Calculate ROI bounds
            int roiTop = (int) (input.rows() * roiTopPercent);
            int roiBottom = (int) (input.rows() * roiBottomPercent);
            int roiLeft = (int) (input.cols() * roiLeftPercent);
            int roiRight = (int) (input.cols() * roiRightPercent);

            // Define the ROI as a rectangle
            Rect roi = new Rect(roiLeft, roiTop, roiRight - roiLeft, roiBottom - roiTop);

            // Crop the input image to the ROI
            cropped = input.submat(roi);

            // Convert the cropped region to HSV color space
            Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_RGB2HSV);

            // Define blue and yellow color ranges
            Scalar lowerBlue = new Scalar(90, 100, 50);
            Scalar upperBlue = new Scalar(150, 255, 255);
            Scalar lowerYellow = new Scalar(20, 50, 150);
            Scalar upperYellow = new Scalar(30, 255, 255);

            // Mask the selected color based on the toggle
            if (detectBlue) {
                Core.inRange(hsv, lowerBlue, upperBlue, colorMask);
            } else {
                Core.inRange(hsv, lowerYellow, upperYellow, colorMask);
            }

            // Perform Canny edge detection on the selected color mask
            Imgproc.Canny(colorMask, edges, 50, 150);

            // Use Hough Line Transform to detect lines
            Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 100, 50, 10);

            LineInfo longestLine = null;
            double maxLength = 0;

            if (lines.rows() > 0) {
                for (int i = 0; i < lines.rows(); i++) {
                    double[] line = lines.get(i, 0);
                    double x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

                    // Calculate the length of the line
                    double length = Math.hypot(x2 - x1, y2 - y1);

                    if (length > maxLength) {
                        maxLength = length;
                        double angle = Math.atan2(y2 - y1, x2 - x1) * 180 / Math.PI;
                        if (angle < 0) {
                            angle += 180; // Normalize angle to [0, 180]
                        }
                        longestLine = new LineInfo(new Point(x1 + roiLeft, y1 + roiTop), new Point(x2 + roiLeft, y2 + roiTop), angle);
                    }
                }

                if (longestLine != null) {
                    orientationAngle = longestLine.angle;

                    // Draw the longest line on the masked image (if enabled) or input image
                    Imgproc.line(showMaskedImage ? colorMask : input, longestLine.start, longestLine.end, new Scalar(255, 0, 0), 2); // Blue line

                    if (!showMaskedImage) {
                        Imgproc.putText(input, "Angle: " + String.format("%.2f", orientationAngle) + " degrees",
                                new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
                    }
                }
            }

            // Return the appropriate frame
            return showMaskedImage ? colorMask : input;

        } finally {
            // Ensure Mats that are no longer used are cleaned up
            hsv.release();
            cropped.release();
            colorMask.release();
            edges.release();
            lines.release();
        }
    }

    public double getOrientationAngle() {
        return orientationAngle;
    }

    public void setShowMaskedImage(boolean showMaskedImage) {
        this.showMaskedImage = showMaskedImage;
    }

    public boolean isDetectingBlue() {
        return detectBlue;
    }

    public void setDetectBlue(boolean detectBlue) {
        this.detectBlue = detectBlue;
    }


    // Helper class to store line information
    private static class LineInfo {
        Point start;
        Point end;
        double angle;

        LineInfo(Point start, Point end, double angle) {
            this.start = start;
            this.end = end;
            this.angle = angle;
        }
    }
}
