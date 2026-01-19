package org.firstinspires.ftc.lib.orion;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

public abstract class BaseOpMode extends OpMode {

    // ---- Static reference to current active opmode ----
    private static BaseOpMode activeOpMode;

    protected static BaseOpMode getActiveOpMode() {
        return activeOpMode;
    }

    // ---- Hub list for bulk caching ----
    protected List<LynxModule> hubs;

    // ---- Timing ----
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime opModeTimer = new ElapsedTime();

    private double avgLoopMs = 0.0;
    private long loopCount = 0;

    protected BaseOpMode(Alliance alliance) {
        Robot.alliance = alliance;
    }

    @Override
    public final void init() {
        Robot.sendHardwareMap(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        onInit();
    }

    @Override
    public final void init_loop() {
        onInitLoop();
    }

    @Override
    public final void start() {
        activeOpMode = this;
        opModeTimer.reset();
        loopTimer.reset();
        onStart();
    }

    @Override
    public final void loop() {

        // ---- Bulk cache clear ----
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // ---- Loop timing ----
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        loopCount++;
        avgLoopMs += (loopMs - avgLoopMs) / loopCount;

        // ---- User loop ----
        onLoop();

        // ---- Dashboard-only telemetry ----
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Robot/Loop Time (ms)", loopMs);
        packet.put("Robot/Avg Loop Time (ms)", avgLoopMs);
        packet.put("Robot/OpMode Time (s)", getOpModeTimeSeconds());
        packet.put("Robot/Alliance", Robot.alliance.toString());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public final void stop() {
        activeOpMode = null;
        onStop();
    }

    // ---- Hooks ----
    protected abstract void onInit();
    protected void onInitLoop() {}
    protected void onStart() {}
    protected abstract void onLoop();
    protected void onStop() {}

    // ---- Public helpers for child OpModes ----

    /**
     * Returns seconds since start() was called.
     * If no opmode is active, returns 0.
     */
    public static double getOpModeTimeSeconds() {
        return activeOpMode != null ? activeOpMode.opModeTimer.seconds() : 0.0;
    }

    /**
     * Returns the average loop time in milliseconds.
     * If no opmode is active, returns 0.
     */
    public static double getAverageLoopTimeMs() {
        return activeOpMode != null ? activeOpMode.avgLoopMs : 0.0;
    }
}
