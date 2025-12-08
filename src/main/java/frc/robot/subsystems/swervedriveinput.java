package frc.robot.subsystems;

public class swervedriveinput {
   public void drive (double x1, double y1, double x2){
    public final double L= ;
    public final double W= ;
    double r = Math.sqrt((L*L) + (W*W));
    y1 *=-1;
    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    double backRightSpeed= Math.sqrt((a * a) + (d * d));
    double backLeftSpeed= Math.sqrt((a * a) + (c * c));
    double frontRightSpeed= Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed= Math.sqrt((b * b) + (c * c));
    double backRightAngle= Math.atan2(a, d) * 180 / Math.PI;
    double backLeftAngle= Math.atan2(a, c) * 180 / Math.PI;
    double frontRightAngle= Math.atan2(b, d) * 180 / Math.PI;
    double frontLeftAngle= Math.atan2(b, c) * 180 / Math.PI;
   } 
}
