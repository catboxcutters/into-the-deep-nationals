    package drive.writtenCode.pipelines;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.opencv.core.Core;
    import org.opencv.core.CvType;
    import org.opencv.core.Mat;
    import org.opencv.core.MatOfPoint;
    import org.opencv.core.MatOfPoint2f;
    import org.opencv.imgproc.Moments;
    import org.opencv.core.Point;
    import org.opencv.core.RotatedRect;
    import org.opencv.core.Scalar;
    import org.opencv.core.Size;
    import org.opencv.imgproc.Imgproc;
    import org.openftc.easyopencv.OpenCvPipeline;
    import org.opencv.utils.Converters;

    import java.util.ArrayList;
    import java.util.Arrays;
    import java.util.Collections;
    import java.util.List;

    public class ShapeDetectionPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        public ShapeDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }
        // Dimensiunile imaginii procesate
        private static final int WIDTH = 640;
        private static final int HEIGHT = 480;

        // Praguri HSV (valorile pentru galben, conform codului tău)
        private static final Scalar YELLOW_LOW = new Scalar(20, 80, 80);
        private static final Scalar YELLOW_HIGH = new Scalar(35, 255, 255);

        // Alte praguri (albastru și roșu)
        private static final Scalar BLUE_LOW = new Scalar(100, 60, 60);
        private static final Scalar BLUE_HIGH = new Scalar(140, 255, 255);
        private static final Scalar RED_TOP_LOW = new Scalar(170, 70, 50);
        private static final Scalar RED_TOP_HIGH = new Scalar(180, 255, 255);
        private static final Scalar RED_BOTTOM_LOW = new Scalar(0, 70, 50);
        private static final Scalar RED_BOTTOM_HIGH = new Scalar(10, 255, 255);

        // Punctele sursă pentru transformarea perspectivică
        private static final Point[] SRC_POINTS = new Point[]{
                new Point(262, 80),   // stânga sus
                new Point(395, 80),   // dreapta sus
                new Point(519, 278),  // dreapta jos
                new Point(139, 275)    // stânga jos
        };

        // Punctele destinație: se mapează la colțurile imaginii de dimensiune WIDTH x HEIGHT
        private static final Point[] DST_POINTS = new Point[]{
                new Point(0, 0),
                new Point(WIDTH, 0),
                new Point(WIDTH, HEIGHT),
                new Point(0, HEIGHT)
        };

        private Mat perspectiveTransform = null;
        private boolean isPerspectiveComputed = false;

        // Variabile pentru a stoca informațiile despre forma "centermost"
        private Point centermostCoordinate = null;
        private double centermostOrientation = Double.NaN;

        // Variabilă pentru telemetrie
        private String shapeTelemetry = "No shape detected";

        /**
         * Returnează coordonata (x, y) a formei cele mai apropiate de centrul imaginii.
         */
        public Point getCentermostCoordinate() {
            return centermostCoordinate;
        }

        /**
         * Returnează orientarea (în grade) a formei celei mai apropiate de centrul imaginii.
         * Dacă nu s-a determinat orientarea, se returnează Double.NaN.
         */
        public double getCentermostOrientation() {
            return centermostOrientation;
        }

        /**
         * Returnează un string cu informațiile despre forma centermost detectată,
         * pentru a fi afișate în telemetrie.
         */
        public String getTelemetryData() {
            return shapeTelemetry;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Redimensionează cadrul de intrare la dimensiunile dorite
            Mat resizedImage = new Mat();
            Imgproc.resize(input, resizedImage, new Size(WIDTH, HEIGHT));

//            Point imageCenter = new Point(WIDTH / 2.0, HEIGHT / 2.0);
            Point imageCenter = new Point(562,278);
            // Calculează matricea de transformare perspectivică o singură dată
            if (!isPerspectiveComputed) {
                List<Point> srcList = Arrays.asList(SRC_POINTS);
                List<Point> dstList = Arrays.asList(DST_POINTS);
                Mat srcMat = Converters.vector_Point2f_to_Mat(srcList);
                Mat dstMat = Converters.vector_Point2f_to_Mat(dstList);
                perspectiveTransform = Imgproc.getPerspectiveTransform(srcMat, dstMat);
                isPerspectiveComputed = true;
            }

            // Aplică transformarea perspectivică
            Mat transformedImage = new Mat();
            Imgproc.warpPerspective(resizedImage, transformedImage, perspectiveTransform, new Size(WIDTH, HEIGHT));

            // Aplică Gaussian Blur de 4 ori pentru netezire
            for (int i = 0; i < 4; i++) {
                Imgproc.GaussianBlur(transformedImage, transformedImage, new Size(5, 5), 0);
            }

            // Convertește imaginea în spațiul de culoare HSV
            Mat hsvImage = new Mat();
            Imgproc.cvtColor(transformedImage, hsvImage, Imgproc.COLOR_RGB2HSV);

            // Aplică threshold-ul pentru galben
            Mat yellowThreshold = new Mat();
            Core.inRange(hsvImage, YELLOW_LOW, YELLOW_HIGH, yellowThreshold);

            // Găsește contururile din masca de threshold pentru galben
            List<MatOfPoint> yellowContours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowThreshold, yellowContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    //        telemetry.addData("yellow contours found", yellowContours.size());
            // Construiește o mască (imagine alb-negru) din contururile găsite
            Mat mask = Mat.zeros(transformedImage.size(), CvType.CV_8UC1);
            Imgproc.drawContours(mask, yellowContours, -1, new Scalar(255), Imgproc.FILLED);

            // Detectează muchiile folosind algoritmul Canny
            Mat edges = new Mat();
            Imgproc.Canny(transformedImage, edges, 50, 150);

            // Crează o mască pentru zgomotul de la margini
            Mat emptyMask = Mat.zeros(transformedImage.size(), CvType.CV_8UC1);
            List<MatOfPoint> edgeContours = new ArrayList<>();
            Imgproc.findContours(edges, edgeContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : edgeContours) {
                if (contour.toArray().length < 10) {
                    Imgproc.drawContours(emptyMask, Collections.singletonList(contour), -1, new Scalar(255), 2);
                }
            }
            Core.bitwise_not(emptyMask, emptyMask);
            Mat processedEdges = new Mat();
            Core.bitwise_and(edges, emptyMask, processedEdges);

            // Aplică transformarea HoughLinesP pentru a detecta linii
            Mat lines = new Mat();
            Imgproc.HoughLinesP(processedEdges, lines, 1, Math.PI / 180, 10, 10, 20);

            // Creează două matrici pentru afișarea liniilor
            Mat hough = transformedImage.clone();
            Mat rawHough = Mat.zeros(transformedImage.size(), CvType.CV_8UC1);

            // Desenează liniile detectate
            if (lines.rows() > 0) {
                for (int i = 0; i < lines.rows(); i++) {
                    double[] l = lines.get(i, 0);
                    Point pt1 = new Point(l[0], l[1]);
                    Point pt2 = new Point(l[2], l[3]);
                    Imgproc.line(hough, pt1, pt2, new Scalar(255), 10, Imgproc.LINE_AA);
                    Imgproc.line(rawHough, pt1, pt2, new Scalar(255), 10, Imgproc.LINE_AA);
                }
            }

            // Găsește contururile pe baza liniilor din rawHough (folosit pentru detecția formelor)
            Imgproc.threshold(rawHough, rawHough, 128, 255, Imgproc.THRESH_BINARY);
            List<MatOfPoint> shapeContours = new ArrayList<>();

            Imgproc.findContours(rawHough, shapeContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    //        telemetry.addData("rawHough type: " , rawHough.type() + ", channels: " + rawHough.channels());
    //        telemetry.addData("Non-zero pixels in rawHough: " , Core.countNonZero(rawHough));
    //        telemetry.addData("Number of contours found in rawHough: " , shapeContours.size());
            // Listă pentru stocarea informațiilor despre forme: coordonate și orientare
            List<ShapeInfo> shapeInfos = new ArrayList<>();
            int i=0;
            for (MatOfPoint contour : shapeContours) {
                // Aproximarea conturului
                i++;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxCurve, 0.01 * Imgproc.arcLength(contour2f, true), true);

                double area = Imgproc.contourArea(contour);
    //            telemetry.addData("Contour ", i + " area: " + area);
                if (area < 100){
    //                telemetry.addData("Skipping contour " , i + " because area is too small.");
                    continue;}

                // Calculează momentele pentru a determina centrul formei
                Moments M = Imgproc.moments(contour);
                if (M.get_m00() == 0) {
    //                telemetry.addData("skipping contour ", i + " because moments are zero.");
                    continue;
                }
                    int cx = (int) (M.get_m10() / M.get_m00());
                int cy = (int) (M.get_m01() / M.get_m00());

                if (cy < 0 || cy >= mask.rows() || cx < 0 || cx >= mask.cols()) {
    //                telemetry.addData("Skipping contour ", i + " because center is out of bounds.");
                    continue;
                }

                // Verifică dacă punctul central se află în regiunea dorită (din mască)
                // Verifică dacă o mică zonă din jurul centrului conține galben
                boolean isInsideYellowRegion = false;
                int kernelSize = 1; // Defines a 3x3 region (change to 2 for 5x5)

                for (int dx = -kernelSize; dx <= kernelSize; dx++) {
                    for (int dy = -kernelSize; dy <= kernelSize; dy++) {
                        int nx = cx + dx;
                        int ny = cy + dy;

                        // Asigură că punctul este în interiorul imaginii
                        if (nx >= 0 && nx < mask.cols() && ny >= 0 && ny < mask.rows()) {
                            double[] maskVal = mask.get(ny, nx);
                            if (maskVal != null && maskVal[0] > 0) {
                                isInsideYellowRegion = true;
                                break;
                            }
                        }
                    }
                    if (isInsideYellowRegion) break;
                }

                if (!isInsideYellowRegion) {
                    continue;
                }



                // Determină orientarea: dacă conturul are cel puțin 5 puncte, se poate folosi fitEllipse
                double orientation;
                if (contour.toArray().length >= 5) {
                    RotatedRect ellipse = Imgproc.fitEllipse(contour2f);
                    orientation = ellipse.angle;
                } else {
                    orientation = Double.NaN;
                }

                shapeInfos.add(new ShapeInfo(cx, cy, orientation));
                // Pentru vizualizare: desenează un cerc mic la centrul formei

                Imgproc.circle(rawHough, new Point(cx, cy), 3, new Scalar(255), 1);
            }

            // Determină forma cea mai apropiată de centrul imaginii

            Imgproc.circle(rawHough, imageCenter, 3, new Scalar(255), 1);
            if (!shapeInfos.isEmpty()) {
                ShapeInfo centermost = shapeInfos.get(0);
                double minDistance = distance(centermost, imageCenter);
                for (ShapeInfo info : shapeInfos) {
                    double d = distance(info, imageCenter);
                    if (d < minDistance) {
                        minDistance = d;
                        centermost = info;
                    }
                }
                centermostCoordinate = new Point(centermost.x, centermost.y);
                centermostOrientation = centermost.orientation;

                // Actualizează informațiile pentru telemetrie
                shapeTelemetry = String.format("Centermost Coordinate: (%d, %d), Orientation: %.2f°",
                        centermost.x, centermost.y, centermost.orientation);

                // Desenează un dreptunghi în jurul formei centermost pentru vizualizare
                int rectSize = 10;
                Point topLeft = new Point(centermost.x - rectSize, centermost.y - rectSize);
                Point bottomRight = new Point(centermost.x + rectSize, centermost.y + rectSize);
                Imgproc.rectangle(rawHough, topLeft, bottomRight, new Scalar(255), 2);
            } else {
                shapeTelemetry = "No shape detected";
            }
    //        telemetry.update();
            // Se returnează imaginea finală (rawHough) pentru afișare
            return rawHough;
        }

        // Metodă auxiliară pentru calculul distanței euclidiene
        private double distance(ShapeInfo info, Point center) {
            return Math.sqrt(Math.pow(info.x - center.x, 2) + Math.pow(info.y - center.y, 2));
        }

        // Clasă internă pentru stocarea informațiilor despre o formă
        private static class ShapeInfo {
            int x, y;
            double orientation;

            ShapeInfo(int x, int y, double orientation) {
                this.x = x;
                this.y = y;
                this.orientation = orientation;
            }
        }
    }
