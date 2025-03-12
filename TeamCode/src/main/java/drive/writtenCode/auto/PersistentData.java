package drive.writtenCode.auto;

import org.opencv.core.Point;

public class PersistentData {
    public static int[] incrementsX={0,1,1,1,1};
    public static int[] incrementsSlides={0,1,1,1,1};
    public static double[] subX = {0,76,76,76,76};
    public static int[] slidesPosition = {0,0,0,0,0};
    public static int[] rotatePosition = {0,0,0,0,0};
    public static Point detected_center = new Point(0,0);
    public static double detected_angle = 0.0;
}
