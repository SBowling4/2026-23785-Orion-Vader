package org.firstinspires.ftc.lib.orion.odometry;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.Matrix;
import org.firstinspires.ftc.lib.wpilib.math.Nat;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.lib.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N1;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N3;
import org.firstinspires.ftc.lib.wpilib.wpilibj.Timer;

public class PoseEstimator {

    private final Odometry odometry;
    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());

    private static final double kBufferDuration = 1.5;

    private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);

    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    private Pose2d m_poseEstimate;

    public PoseEstimator(
            Odometry odometry,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {

        this.odometry = odometry;
        this.m_poseEstimate = new Pose2d();

        for (int i = 0; i < 3; i++) {
            double std = stateStdDevs.get(i, 0);
            m_q.set(i, 0, std * std);
        }

        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    public final void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        double[] r = new double[3];

        for (int i = 0; i < 3; i++) {
            double std = visionMeasurementStdDevs.get(i, 0);
            r[i] = std * std;
        }

        for (int row = 0; row < 3; row++) {
            double q = m_q.get(row, 0);

            if (q == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                        row,
                        row,
                        q / (q + Math.sqrt(q * r[row]))
                );
            }
        }
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = odometry.getPoseWPILib();
    }

    public void resetTranslation(Translation2d translation) {
        odometry.resetPose(
                new Pose2d(
                        translation,
                        odometry.getPoseWPILib().getRotation()
                )
        );
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = odometry.getPoseWPILib();
    }

    public void resetRotation(Rotation2d rotation) {
        odometry.resetPose(
                new Pose2d(
                        odometry.getPoseWPILib().getTranslation(),
                        rotation
                )
        );
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = odometry.getPoseWPILib();
    }

    /**
     *
     * @return gets the estimated pose from the pose estimator in Orion coordinates
     */
    public Pose2d getEstimatedPosition() {
        Pose3d pose3d = new Pose3d(m_poseEstimate.getX(), m_poseEstimate.getY(), 0, new Rotation3d(0,0, m_poseEstimate.getRotation().getRadians()));

        return CoordinateSystemConverter.WPILibToOrion(pose3d);
    }

    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        double oldest = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        double newest = m_odometryPoseBuffer.getInternalBuffer().lastKey();

        timestampSeconds = MathUtil.clamp(timestampSeconds, oldest, newest);

        if (m_visionUpdates.isEmpty()
                || timestampSeconds < m_visionUpdates.firstKey()) {
            return m_odometryPoseBuffer.getSample(timestampSeconds);
        }

        double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        VisionUpdate visionUpdate = m_visionUpdates.get(floorTimestamp);

        Optional<Pose2d> odometryEstimate =
                m_odometryPoseBuffer.getSample(timestampSeconds);

        return odometryEstimate.map(
                odometryPose -> visionUpdate.compensate(odometryPose)
        );
    }

    private void cleanUpVisionUpdates() {
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }

        double oldestOdometryTimestamp =
                m_odometryPoseBuffer.getInternalBuffer().firstKey();

        if (m_visionUpdates.isEmpty()
                || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }

        double newestNeededVisionUpdateTimestamp =
                m_visionUpdates.floorKey(oldestOdometryTimestamp);

        m_visionUpdates
                .headMap(newestNeededVisionUpdateTimestamp, false)
                .clear();
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
                                     double timestampSeconds) {

        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
                || m_odometryPoseBuffer.getInternalBuffer().lastKey()
                - kBufferDuration > timestampSeconds) {
            return;
        }

        cleanUpVisionUpdates();

        Optional<Pose2d> odometrySample =
                m_odometryPoseBuffer.getSample(timestampSeconds);

        if (!odometrySample.isPresent()) {
            return;
        }

        Optional<Pose2d> visionSample = sampleAt(timestampSeconds);

        if (!visionSample.isPresent()) {
            return;
        }

        Twist2d twist =
                visionSample.get().log(visionRobotPoseMeters);

        Matrix<N3, N1> kTimesTwist =
                m_visionK.times(
                        VecBuilder.fill(twist.dx, twist.dy, twist.dtheta)
                );

        Twist2d scaledTwist = new Twist2d(
                kTimesTwist.get(0, 0),
                kTimesTwist.get(1, 0),
                kTimesTwist.get(2, 0)
        );

        VisionUpdate visionUpdate = new VisionUpdate(
                visionSample.get().exp(scaledTwist),
                odometrySample.get()
        );

        m_visionUpdates.put(timestampSeconds, visionUpdate);
        m_visionUpdates.tailMap(timestampSeconds, false).clear();

        m_poseEstimate =
                visionUpdate.compensate(odometry.getPoseWPILib());
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {

        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public Pose2d update() {
        return updateWithTime(Timer.getTimestampSeconds());
    }

    public Pose2d updateWithTime(double currentTimeSeconds) {
        Pose2d odometryEstimate = odometry.getPoseWPILib();

        m_odometryPoseBuffer.addSample(
                currentTimeSeconds,
                odometryEstimate
        );

        if (m_visionUpdates.isEmpty()) {
            m_poseEstimate = odometryEstimate;
        } else {
            VisionUpdate visionUpdate =
                    m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate =
                    visionUpdate.compensate(odometryEstimate);
        }

        return m_poseEstimate;
    }

    private static final class VisionUpdate {
        private final Pose2d visionPose;
        private final Pose2d odometryPose;

        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        public Pose2d compensate(Pose2d pose) {
            Transform2d delta = pose.minus(odometryPose);
            return visionPose.plus(delta);
        }
    }
}
