package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import paths.callbacks.Callback;
import paths.movements.Path;
import paths.movements.Turn;
import paths.movements.FollowerMovement;
import paths.geometry.BSpline;
import paths.geometry.PathSegment;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import util.Angle;
import util.Pose;
import util.ArcPose;
import util.Vector;

/**
 * A unified builder designed to construct either a continuous geometric {@link Path}
 * or a stationary point-{@link Turn} fluently using a single API interface.
 * <p>
 * This class captures movement configurations in any order and defers geometric
 * compilation and safety validation until {@link #build()} is called.
 * <p>
 * @author Sohum Arora 22985 Paraducks
 */
public class MovementBuilder {
    private final Pose startPose;
    private MovementType movementType;
    private Pose[] rawPoses = null;
    private Pose expectedEndPose;
    private InterpolationStyle currentStyle = InterpolationStyle.SMOOTH_START_TO_END;
    private HeadingInterpolator customInterpolator = null;
    private Angle targetHeading = null;
    private final List<DistanceCallbackData> distanceCallbacks = new ArrayList<>();
    private final List<AngularCallbackData> angularCallbacks = new ArrayList<>();
    private final List<String> buildWarnings = new ArrayList<>();

    /**
     * Initializes the builder with the starting location and heading of the robot.
     * @param startPose The initial Pose of the robot at the beginning of the movement.
     */
    public MovementBuilder(Pose startPose) {
        this.startPose = startPose;
    }

    /**
     * Initializes the builder and explicitly declares the movement type upfront.
     * @param startPose The initial Pose of the robot.
     * @param type The type of movement being built.
     */
    public MovementBuilder(Pose startPose, MovementType type) {
        this.startPose = startPose;
        this.movementType = type;
    }

    /**
     * Stores a sequence of control points to define a continuous Uniform Cubic B-Spline.
     * Sets the internal movement type to {@link MovementType#PATH}.
     */
    public MovementBuilder addControlPoints(Pose... poses) {
        if (this.movementType == MovementType.TURN) {
            throw new IllegalStateException("Cannot add control points to a movement of type TURN!");
        }
        if (this.rawPoses != null) {
            throw new IllegalStateException("Control points have already been added to this builder!");
        }
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }

        this.movementType = MovementType.PATH;
        this.rawPoses = poses;
        this.expectedEndPose = poses[poses.length - 1];

        return this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) interpolation with a different style.
     */
    public MovementBuilder interpolateWith(InterpolationStyle style) {
        switch (style) {
            case TANGENT_OPTIMAL:
            case TANGENT_FORWARD:
            case SMOOTH_START_TO_END:
                if (style == InterpolationStyle.SMOOTH_START_TO_END) {
                    addWarning("APEX WARNING: SMOOTH_START_TO_END is the default interpolator, there's no need to change it!");
                }
                this.currentStyle = style;
                break;
            default:
                throw new IllegalArgumentException("You need more parameters for: " + style.name() + "!");
        }
        return this;
    }

    /**
     * Overrides the default interpolation strategy with a custom function of distance percentage (s).
     */
    public MovementBuilder interpolateWith(Function<Double, Angle> function) {
        this.customInterpolator = new HeadingInterpolator(function);
        return this;
    }

    //********************** Turn methods *************************

    /**
     * Defines the target angle for a stationary point turn.
     * Sets the internal movement type to {@link MovementType#TURN}.
     */
    public MovementBuilder turnTo(Angle targetHeading) {
        if (this.movementType == MovementType.PATH) {
            throw new IllegalStateException("Cannot assign a point turn target to a designated PATH movement!");
        }
        this.movementType = MovementType.TURN;
        this.targetHeading = targetHeading;
        return this;
    }

    //****************** Callback Methods *******************

    /**
     * Attaches an executable callback based on physical distance percentage [0.0, 1.0].
     * (Only valid for {@link MovementType#PATH} movements).
     */
    public MovementBuilder addDistanceCallback(double s, Runnable action) {
        distanceCallbacks.add(new DistanceCallbackData(s, action));
        return this;
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     * Works seamlessly for both PATH and TURN sweeps.
     */
    public MovementBuilder addAngularCallback(Angle angle, Runnable action) {
        angularCallbacks.add(new AngularCallbackData(angle, action));
        return this;
    }
    /**
     * Compiles configuration parameters, calculates B-Spline curves, validates all
     * callback parameters against physical limits, and returns the finished movement.
     * * @return The fully constructed {@link FollowerMovement} execution container.
     */
    public FollowerMovement build() {
        if (this.movementType == null) {
            throw new IllegalStateException("Cannot build movement: Type is unspecified. Call addControlPoints() or turnTo() first!");
        }

        if (this.movementType == MovementType.PATH) {
            return buildPath();
        } else {
            return buildTurn();
        }
    }

    private Path buildPath() {
        if (rawPoses == null) {
            throw new IllegalStateException("Cannot build path: No control points were added!");
        }

        Path path = new Path();

        // Pass collected build warnings over to the final path object
        for (String warning : buildWarnings) {
            path.addWarning(warning);
        }

        // 1. Pre-process the points (Expand ArcPoses)
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are ignored! Only the final pose heading controls the end heading.");
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.toVec().subtract(arcPose.toVec());
                Vector vecToNext = nextPose.toVec().subtract(arcPose.toVec());

                double distToLast = vecToLast.getMagnitude();
                double distToNext = vecToNext.getMagnitude();

                if (radius > distToLast || radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds boundaries of adjacent control points.");
                }

                Vector p1Vec = arcPose.toVec().add(vecToLast.multiply(radius / distToLast));
                Vector p2Vec = arcPose.toVec().add(vecToNext.multiply(radius / distToNext));

                processedPoses.add(new Pose(p1Vec.getX(), p1Vec.getY(), arcPose.getHeading()));
                processedPoses.add(currentPose);
                processedPoses.add(new Pose(p2Vec.getX(), p2Vec.getY(), arcPose.getHeading()));
            } else {
                processedPoses.add(currentPose);
            }
        }
        processedPoses.add(rawPoses[rawPoses.length - 1]);

        // 2. Build the curve math
        Vector[] vectors = new Vector[processedPoses.size() + 1];
        vectors[0] = startPose.toVec();

        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i + 1] = processedPoses.get(i).toVec();
        }

        path.setParametricPath(new PathSegment(new BSpline(vectors)));

        // 3. Inject heading interpolation strategies
        if (customInterpolator != null) {
            path.setInterpolator(customInterpolator);
        } else {
            path.setInterpolator(buildSafeInterpolator(startPose, expectedEndPose, path));
        }

        // 4. Attach and calculate callbacks
        for (DistanceCallbackData data : distanceCallbacks) {
            path.addCallback(new Callback(data.s, data.action));
        }

        double startRad = startPose.getHeading();
        double endRad = expectedEndPose.getHeading();
        for (AngularCallbackData data : angularCallbacks) {
            validateAngularCallbackBounds(startRad, endRad, data.angle, "path's start and end headings");
            path.addCallback(new Callback(data.angle, data.action));
        }

        return path;
    }

    private Turn buildTurn() {
        if (targetHeading == null) {
            throw new IllegalStateException("Cannot build Turn: No target heading was specified! Use .turnTo().");
        }
        if (!distanceCallbacks.isEmpty()) {
            throw new IllegalArgumentException("Cannot attach physical distance callbacks to a stationary point-turn movement!");
        }

        Turn turn = new Turn(startPose, targetHeading);

        // Run unified boundary check for all angular callbacks inside the sweep
        double startRad = startPose.getHeading();
        double endRad = targetHeading.getRad();
        for (AngularCallbackData data : angularCallbacks) {
            validateAngularCallbackBounds(startRad, endRad, data.angle, "sweep range of this turn");
            turn.addCallback(new Callback(data.angle, data.action));
        }

        return turn;
    }


    private void validateAngularCallbackBounds(double startRad, double endRad, Angle targetAngle, String scopeMessage) {
        if (Double.isFinite(startRad) && Double.isFinite(endRad)) {
            double targetRad = targetAngle.getRad();

            double totalDiff = getShortestAngularDifference(startRad, endRad);
            double targetDiff = getShortestAngularDifference(startRad, targetRad);

            if (Math.abs(totalDiff) < 1e-6) {
                if (Math.abs(targetDiff) > 1e-6) {
                    throw new IllegalArgumentException("Angular callback out of bounds: The movement has no rotational distance.");
                }
            } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                throw new IllegalArgumentException("Angular callback is outside the " + scopeMessage + ".");
            }
        }
    }

    private HeadingInterpolator buildSafeInterpolator(Pose start, Pose end, Path pathObj) {
        if (currentStyle == InterpolationStyle.TANGENT_FORWARD) {
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        boolean missingHeading = !Double.isFinite(start.getHeading()) || !Double.isFinite(end.getHeading());

        if (missingHeading) {
            pathObj.addWarning("APEX WARNING: Segment missing start/end heading! Falling back to TANGENT_FORWARD. Use Pose(x, y, heading) to fix this.");
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        return new HeadingInterpolator(currentStyle, start.getHeadingComponent(), end.getHeadingComponent());
    }

    private double getShortestAngularDifference(double from, double to) {
        double diff = (to - from) % (2 * Math.PI);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        else if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    private void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) {
            buildWarnings.add(warning);
        }
    }
    private static class DistanceCallbackData {
        final double s;
        final Runnable action;

        DistanceCallbackData(double s, Runnable action) {
            this.s = s;
            this.action = action;
        }
    }
    private static class AngularCallbackData {
        final Angle angle;
        final Runnable action;

        AngularCallbackData(Angle angle, Runnable action) {
            this.angle = angle;
            this.action = action;
        }
    }
}