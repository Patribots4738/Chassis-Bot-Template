package calc;

import calc.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class PatriBoxController extends XboxController {
    private boolean POVPressedUp = false;
    private boolean POVPressedDown = false;
    private boolean POVPressedLeft = false;
    private boolean POVPressedRight = false;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public PatriBoxController(int port) {
        super(port);
    }

    @Override
    public double getLeftY() {
        return toCircle(
                MathUtil.applyDeadband(
                        super.getLeftX(), OIConstants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(
                        super.getLeftY(), OIConstants.CONTROLLER_DEADBAND)).getY();
    }

    @Override
    public double getLeftX() {
        return toCircle(
                MathUtil.applyDeadband(
                        super.getLeftX(), OIConstants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(
                        super.getLeftY(), OIConstants.CONTROLLER_DEADBAND)).getX();
    }

    @Override
    public double getRightY() {
        return toCircle(
                MathUtil.applyDeadband(
                        super.getRightX(), OIConstants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(
                        super.getRightY(), OIConstants.CONTROLLER_DEADBAND)).getY();
    }

    @Override
    public double getRightX() {
        return toCircle(
                MathUtil.applyDeadband(
                        super.getRightX(), OIConstants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(
                        super.getRightY(), OIConstants.CONTROLLER_DEADBAND)).getX();
    }

    // All calculations can be referenced here https://www.desmos.com/calculator/e07raajzh5
    public static Translation2d toCircle(double x, double y) {

        Translation2d intercept;
        Translation2d output;

        double slope = y / x;

        if (slope == 0 || Double.isNaN(slope) || Double.isInfinite(slope)) {

            output = new Translation2d(x, y);
            return output;

        } else if (0 > slope && slope >= -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(-1, -slope);

        } else if (-OIConstants.CONTROLLER_CORNER_SLOPE_1 < slope && slope < -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope - 1);
            double intersectionY = (intersectionX + 1.7);

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (slope < -OIConstants.CONTROLLER_CORNER_SLOPE_1 || slope > OIConstants.CONTROLLER_CORNER_SLOPE_1) {

            intercept = new Translation2d(1 / slope, 1);

        } else if (OIConstants.CONTROLLER_CORNER_SLOPE_1 > slope && slope > OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope + 1);
            double intersectionY = -intersectionX + 1.7;

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (0 < slope && slope <= OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(1, slope);

        } else {

            intercept = new Translation2d(0, 0);

            System.out.println("Error... Slope = " + slope);

        }

        double distance = getDistance(x, y, intercept.getX(), intercept.getY());

        distance = (distance > 1) ? 1 : distance;

        output = new Translation2d(Math.signum(x) * distance, new Rotation2d(Math.atan(y / x)));

        return output;
    }

    public static double getDistance(double x1, double y1, double x2, double y2) {
        return
                Math.sqrt(
                        (Math.pow(x1, 2) + Math.pow(y1, 2)) /
                                (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

    public int getPOVPressed() {

        int pressedPOV = -1;

        if (super.getPOV() == 0) {
            if (!POVPressedUp) {
                POVPressedUp = true;
                pressedPOV = 0;
            }
        } else {
            POVPressedUp = false;
        }

        if (super.getPOV() == 180) {
            if (!POVPressedDown) {
                POVPressedDown = true;
                pressedPOV = 180;
            }
        } else {
            POVPressedDown = false;
        }

        if (super.getPOV() == 270) {
            if (!POVPressedLeft) {
                POVPressedLeft = true;
                pressedPOV = 270;
            }
        } else {
            POVPressedLeft = false;
        }

        if (super.getPOV() == 90) {
            if (!POVPressedRight) {
                POVPressedRight = true;
                pressedPOV = 90;
            }
        } else {
            POVPressedRight = false;
        }

        return pressedPOV;
    }
}
