package drive.writtenCode.pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class MihneaDetection extends OpenCvPipeline {
    Telemetry telemetry;
    public MihneaDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    private static final Scalar LOWER_YELLOW = new Scalar(20, 140, 180); //20 120 180
    private static final Scalar UPPER_YELLOW = new Scalar(35, 255, 255);

    private static final int CONTOUR_THRESH_LOW = 500;
    private static final int CONTOUR_THRESH_HIGH = 4000;

    private int WIDTH = 640;
    private int HEIGHT = 480;
    private int ORIGINAL_WIDTH = 1920;
    private int ORIGINAL_HEIGHT = 1200;

    private Point ClosestPoint = new Point(-1, -1);
    private double MinimumDistance = Double.MAX_VALUE;
    private double closestAngle = 0;

    @Override
    public Mat processFrame(Mat input) {

        Mat hsv = new Mat();
        Mat colorImg = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> samples= new ArrayList<>();
        List<Double> angles= new ArrayList<>();
        double aspect_ratio = (double) ORIGINAL_WIDTH /ORIGINAL_HEIGHT;
        HEIGHT = (int) (WIDTH/aspect_ratio);
        Size image_size= new Size(WIDTH,480);
        Imgproc.resize(input, input, image_size);
        Point img_center = new Point((double) WIDTH /2, (double) 480 /2+10);

        // Convert input to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        for(int i=1;i<=2;i++)
        {
            Imgproc.GaussianBlur(hsv,hsv, new Size(3,3),0);
        }
        Core.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW, colorImg);


        // Morphological operations to clean up the mask
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(colorImg, colorImg, Imgproc.MORPH_CLOSE, kernel, new Point(-1, -1), 2);
        Imgproc.morphologyEx(colorImg, colorImg, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), 2);

        // Convert input to grayscale for Canny edge detection
//        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(colorImg, edges, 50, 150);
//        return edges;
        // Dilate and open edges to match Python behavior
//        Imgproc.dilate(edges, edges, kernel, new Point(-1, -1), 1);
//        Imgproc.morphologyEx(edges, edges, Imgproc.MORPH_OPEN, kernel);

        // Find contours
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        contours.clear();
        for (MatOfPoint sample : samples) {
            sample.release();
        }
        samples.clear();
        angles.clear();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double minDist = Double.MAX_VALUE;
        Point closestPoint = new Point(-1,-1);
        Double closestOrientation = 0.0;
        for (MatOfPoint contour : contours) {
            double angle=0;
            if (contour.rows() < 5) {
                continue;
            }
            MatOfPoint box;
            Mat mask = Mat.zeros(input.size(), CvType.CV_8UC1);
            RotatedRect rectangle;
            try {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                rectangle=rect;
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                box = new MatOfPoint(boxPoints);
                angle = rect.angle; // Store the correct orientation angle
                if(rect.size.width < rect.size.height){
                    angle=rect.angle-180;
                }else{
                    angle=rect.angle-90;
                }
                angle=-angle;
                List<MatOfPoint> boxList = new ArrayList<>();
                boxList.add(box);

                Imgproc.drawContours(mask, boxList, -1, new Scalar(255), Core.FILLED);
            } catch (Exception e) {
                mask.release();
                continue;
            }

            Mat maskedImg = new Mat();
            Core.bitwise_and(colorImg, colorImg, maskedImg, mask);
            Scalar vals = Core.mean(maskedImg);
            double area = Imgproc.contourArea(contour);
            if (area == 0) {
                mask.release();
                maskedImg.release();
                continue;
            }
            double satVal = (vals.val[0] * input.rows() * input.cols()) / area;
            satVal /= 255.0;

            if (vals.val[0] > 0.5 && filterByRatio(box)) {
                samples.add(box);
                angles.add(angle);
                Point center = rectangle.center;
                double distance = Math.sqrt((center.x - img_center.x)*(center.x - img_center.x) + (center.y - img_center.y)*(center.y - img_center.y));
                if (minDist > distance && center.y<300) {
                 minDist = distance;
                 closestPoint = center;
                 closestOrientation = angle;
                 }

            Imgproc.circle(input, center, 5, new Scalar(0, 0, 255), -1);
            }
            mask.release();
            maskedImg.release();
        }

        Imgproc.drawContours(input, samples, -1, new Scalar(0, 255, 0), 2);

        Imgproc.circle(input, img_center, 2, new Scalar(255,255,255),-1);
        Imgproc.circle(input, new Point(100,295), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,260), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,227), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,195), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,163), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,133), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,103), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,73), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,43), 2, new Scalar(0,0,0),-1);
        Imgproc.circle(input, new Point(100,13), 2, new Scalar(0,0,0),-1);

        if (closestPoint != null) {
            Imgproc.circle(input, closestPoint, 6, new Scalar(0, 0, 0), -1);
        }
        closestAngle=closestOrientation;
        ClosestPoint=closestPoint;
        MinimumDistance = minDist;
        telemetry.addData("angle: ", closestOrientation);
        telemetry.addData("x: ", closestPoint.x);
        telemetry.addData("y: ", closestPoint.y);
        telemetry.update();

        kernel.release();
        hsv.release();
        colorImg.release();
        edges.release();
        hierarchy.release();
        return input;
    }

    public static boolean filterByRatio(MatOfPoint sample) {
        Point[] points = sample.toArray();
        if (points.length < 4) {
            return false;
        }

        double width = Math.sqrt((points[0].x - points[1].x)*(points[0].x - points[1].x) + (points[0].y - points[1].y)*(points[0].y - points[1].y));
        double length = Math.sqrt((points[1].x - points[2].x)*(points[1].x - points[2].x) + (points[1].y - points[2].y)*(points[1].y - points[2].y));
        double ratio = length / width;
        double ratioInverted = width / length;
        double area = length * width;

        final double MIN_AREA = 1000;
        final double MAX_AREA = 4000;
        if (area < MIN_AREA || area > MAX_AREA) {
            return false;
        }

        final double MIN_RATIO = 1.3;
        final double MAX_RATIO = 1000;
        return (MIN_RATIO < ratio && ratio < MAX_RATIO) || (MIN_RATIO < ratioInverted && ratioInverted < MAX_RATIO);
    }

    public double getClosestAngle() {
        return closestAngle;
    }

    public Point getClosestPoint() {
        return ClosestPoint;
    }
    public double getMinimumDistance(){return MinimumDistance;}
}
