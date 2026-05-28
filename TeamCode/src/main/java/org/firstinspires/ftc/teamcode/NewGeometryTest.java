package org.firstinspires.ftc.teamcode;

import geometry.Angle;
import geometry.ArcPose;
import geometry.Dist;
import geometry.Vector;
import geometry.Pose;

import util.PoseFactory;
import util.AngleUnit;
import util.DistUnit;

public class NewGeometryTest {
    // Angles are stored internally in radians, but can be created and accessed in any unit
    // 0 is facing the positive x-axis and angles increase counterclockwise (like the unit circle)
    // Normalization methods wrap to [0, 2pi], angles are not wrapped by default
    // Units (AngleUnit): DEG, RAD
    public void angleDemo() {
        // Constructors & Factories
        Angle a1 = Angle.of(180, AngleUnit.DEG);
        Angle aDeg = Angle.fromDeg(400.0);
        Angle aRad = Angle.fromRad(Math.PI / 4);
        Angle negativeAngle = Angle.fromDeg(-90.0);

        // Getters
        double asDegreesUnit = a1.get(AngleUnit.DEG);
        double getRadians = a1.getRad();
        double getDegrees = a1.getDeg();

        // Arithmetic
        Angle aSum = a1.plus(aDeg);
        Angle aDiff = a1.minus(aDeg);

        // Other Operations
        Angle mirroredX = aRad.mirrorX();
        Angle mirroredY = aRad.mirrorY();
        Angle normalizedAngle = aDeg.normalized();
        Angle copiedAngle = a1.copy();
        String angleString = a1.toString();

        // Static utility methods
        double normalizedRad = Angle.normalize(3 * Math.PI);
    }

    // Distances are stored internally in inches, but can be created and accessed in any unit
    // Units (DistUnit): IN, FT, MM, CM, M
    public void distDemo() {
        // Constructors & Factories
        Dist d1 = Dist.of(12.0, DistUnit.IN);
        Dist d2 = Dist.fromIn(5.0);
        Dist dFeet = Dist.fromFt(2.5);
        Dist dMm = Dist.fromMm(100.0);
        Dist dCm = Dist.fromCm(15.0);
        Dist dMeters = Dist.fromM(1.2);
        Dist negativeDist = Dist.fromIn(-10.0);

        // Getters
        double asInches = d1.get(DistUnit.IN);
        double getInches = d1.getIn();
        double getFeet = d1.getFt();
        double getMm = d1.getMm();
        double getCm = d1.getCm();
        double getMeters = d1.getM();

        // Arithmetic
        Dist sum = d1.plus(d2);
        Dist diff = d1.minus(d2);
        Dist timesScalar = d1.times(2.0);
        Dist timesDist = d1.times(d2);
        Dist divScalar = d1.div(2.0);
        Dist divDist = d1.div(d2);
        Dist absoluteDist = negativeDist.abs();
        Dist hypotenuse = d1.hypot(dFeet);

        // Other Operations
        Dist mirroredDist = d1.mirror();
        Dist copiedDist = dFeet.copy();
        String distString = dFeet.toString();
    }

    // Vectors represent a 2D position or direction using two Dist objects
    public void vectorDemo() {
        // Constructors & Factories
        Vector v1 = new Vector(Dist.fromIn(3.0), Dist.fromIn(4.0));
        Vector v2 = Vector.of(1.0, 2.0, DistUnit.IN);
        Vector vNegative = Vector.of(-30.0, -45.0, DistUnit.MM);
        Vector polar = Vector.fromPolar(Dist.fromFt(5.0), Angle.fromDeg(60.0));
        Vector zeroVector = Vector.zero();

        // Getters
        Dist xDist = v1.getX();
        Dist yDist = v1.getY();
        double xDouble = v1.getX(DistUnit.IN);
        double yDouble = v1.getY(DistUnit.MM);
        Dist magnitude = v1.getMag();
        Dist magnitudeSq = v1.getMagSq();
        Angle theta = v1.getTheta();

        // Arithmetic
        Vector vSum = v1.plus(v2);
        Vector vDiff = v1.minus(v2);
        Vector vTimes = v1.times(2.0);
        Vector vDiv = v1.div(2.0);
        Vector vAbsolute = vNegative.abs();
        Dist dotProduct = v1.dot(v2);
        Dist crossProduct = v1.cross(v2);

        // Other Operations
        Dist distanceBetween = v1.distanceTo(v2);
        Vector rotatedVector = v1.rotate(Angle.fromDeg(90.0));
        Vector normalizedVec = v1.normalize();
        Vector mirrorXVector = v1.mirrorX();
        Vector mirrorYVector = v1.mirrorY();
        Vector reflectedVector = v1.reflect(v2);
        Vector copiedVector = v1.copy();
        String vectorString = v1.toString();
    }

    // Poses represent a 2D position along with a directional heading angle
    // PoseFactory should generally be used for creation to handle units cleanly
    public void poseDemo() {
        // Constructors & Factories
        PoseFactory poseFactory = new PoseFactory(DistUnit.CM, AngleUnit.RAD);
        PoseFactory poseFactoryAlt = new PoseFactory(DistUnit.CM, AngleUnit.RAD, PoseFactory.Mirror.X);

        // Factory Setters
        poseFactory.setMirror(PoseFactory.Mirror.NONE);
        poseFactory.setDistUnit(DistUnit.IN);
        poseFactory.setAngleUnit(AngleUnit.DEG);

        // Factory Getters
        PoseFactory.Mirror currentMirror = poseFactory.getMirror();
        DistUnit currentDistUnit = poseFactory.getDistUnit();
        AngleUnit currentAngleUnit = poseFactory.getAngleUnit();

        // Pose Creation
        Pose p1 = poseFactory.of(12.0, 24.0, 90.0); // X: 12in, Y: 24in, Heading: 90deg
        Pose p2 = poseFactory.of(5.0, 10.0); // Heading defaults to 0
        Pose zeroPose = Pose.zero(); // Direct creation: Vector(0,0), Angle(0)

        // ArcPose Creation
        ArcPose arcPoseFromFactory = poseFactory.arcPoseOf(10.0, 20.0, 5.0); // X: 10in, Y: 20in, Radius: 5in
        ArcPose rawArcPose = new ArcPose(Vector.of(5, 5, DistUnit.IN), Dist.fromIn(2.0));
        Dist arcRadius = rawArcPose.getRadius();

        // Common Poses
        Pose centerPose = Pose.Common.CENTER.get(); // (0, 0, 0)
        Pose topLeftPose = Pose.Common.TOP_LEFT.withHeading(180.0, AngleUnit.DEG); // (-70.5, 70.5, 180)
        Pose bottomRightPose = Pose.Common.BOTTOM_RIGHT.withHeading(Angle.fromDeg(90.0)); // (70.5, -70.5, 90)
        Vector centerPosRaw = Pose.Common.CENTER.getPosition(); // (0, 0) (Vector)

        // Getters
        Vector position = p1.getPos();

        Dist xDist = p1.getX();
        double xInches = p1.getX(DistUnit.IN);

        Dist yDist = p1.getY();
        double yMillimeters = p1.getY(DistUnit.MM);

        Angle heading = p1.getHeading();
        double headingRad = p1.getHeading(AngleUnit.RAD);

        // Arithmetic
        Pose pSum = p1.plus(p2);
        Pose pDiff = p1.minus(p2);

        // Other Operations
        Dist distanceBetweenPoses = p1.distanceTo(p2);
        Pose pMirroredX = p1.mirrorX();
        Pose pMirroredY = p1.mirrorY();
        Pose copiedPose = p1.copy();
        String poseString = p1.toString();

        // Factory mirror behavior
        poseFactory.setMirror(PoseFactory.Mirror.X);
        Pose autoMirroredPose = poseFactory.of(10.0, 10.0, 45.0); // (10in, -10in, 315deg)
    }
}