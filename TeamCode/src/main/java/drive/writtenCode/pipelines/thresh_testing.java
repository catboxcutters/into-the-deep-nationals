package drive.writtenCode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class thresh_testing extends OpenCvPipeline {
    // HSV Thresholds for yellow
    private static final Scalar YELLOW_LOW = new Scalar(20, 80, 80);
    private static final Scalar YELLOW_HIGH = new Scalar(35, 255, 255);

    private Mat hsvImage = new Mat();
    private Mat yellowMask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Apply threshold to create a mask
        Core.inRange(hsvImage, YELLOW_LOW, YELLOW_HIGH, yellowMask);

        return yellowMask; // Return the binary mask
    }
}
