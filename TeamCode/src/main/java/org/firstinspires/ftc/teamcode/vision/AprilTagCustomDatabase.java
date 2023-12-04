package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagCustomDatabase {

    public static AprilTagLibrary getCenterStageLibrary(){
        return new AprilTagLibrary.Builder()
                .addTag(10,"B1",5, new VectorF((float) -72.00, 41.50F, 4.00F),
                        DistanceUnit.INCH, Quaternion.identityQuaternion())
                .addTag(9,"B1s",2, new VectorF((float) -72.00, 36.00F, 4.00F),
                        DistanceUnit.INCH, Quaternion.identityQuaternion())
                .addTag(8,"E1s",2, new VectorF((float) -72.00, -36.00F, 4.00F),
                        DistanceUnit.INCH, Quaternion.identityQuaternion())
                .addTag(7,"E1s",5, new VectorF((float) -72.00, -41.50F, 4.00F),
                        DistanceUnit.INCH, Quaternion.identityQuaternion())
                .build();
    }

}
