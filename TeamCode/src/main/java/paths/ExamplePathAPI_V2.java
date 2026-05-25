package paths;

import paths.heading.InterpolationStyle;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * This is a test path to demonstrate the capabilities of the new .addPath() method in
 * the PathBuilder
 * We can delete one of these 2 ExamplePathAPI classes later once we decide how we will
 * go about the PathBuilder modifications
 */
public class ExamplePathAPI_V2 {

    public Path testPath() {
        return new PathBuilder(new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.RADIANS))
                .newPath(
                        // 1 & 2. THE B-SPLINE
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.BSPLINE,
                                new Pose(Distance.Units.MILLIMETERS, 600, 0),
                                new Pose(15, 10),
                                new Pose(25, 20, Math.toRadians(90))
                        ),

                        // 2. POINT TURN: Stationary rotation.
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.TURN,
                                new Pose(0, 0, Angle.fromDeg(135).getRad())
                        ),

                        // 3 & 4. STRAIGHT LINE WITH IN-LINE OVERRIDE INSIDE THE STEP
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.LINETO,
                                InterpolationStyle.TANGENT_FORWARD, // Style override passed directly here!
                                new Pose(0, 0, Math.toRadians(180))
                        ),

                        // 5. ADVANCED LAMBDA OVERRIDE INSIDE THE STEP
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.BSPLINE,
                                s -> new Angle(s * (6 * Math.PI)),  // Lambda function passed directly here!
                                new Pose(10, 10),
                                new Pose(20, 0, 0)
                        ),

                        // 6. FAILSAFE DEMONSTRATION: Missing headings!
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.LINETO,
                                new Pose(Distance.Units.INCHES, 30, 0)
                        )
                ) // All paths, overrides, and turns are closed right here! One single monolithic block.

                /*
                 * Look how easy it became! All the paths go in one .newPath() block
                 * Importantly, this unifies all the .bSplineTo(), .turnTo() and .lineTo() methods using an enum which
                 * in my opinion at least is way cleaner :)
                 */

                // 7. COMPILE: Locks the path and calculates all the Look-Up Tables.
                .build();
    }
}