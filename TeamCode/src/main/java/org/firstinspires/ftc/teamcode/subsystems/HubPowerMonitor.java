package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.config.TunableValues;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Samples Control Hub / Expansion Hub battery, 5V rail, and current, plus
 * OctoQuad uptime. Logs when voltage steps past a threshold or the OctoQuad
 * reboots, so a power brownout can be confirmed or ruled out.
 */
public class HubPowerMonitor {
    private static final double SAMPLE_PERIOD_SEC = 0.05;
    private static final double BASELINE_LOG_PERIOD_SEC = 1.0;
    private static final double INVALID_VIN_V = 1.0;

    private final HardwareMap hardwareMap;
    private final OctoQuadOdometry octoquad;
    private final ElapsedTime sampleTimer = new ElapsedTime();
    private final ElapsedTime baselineLogTimer = new ElapsedTime();
    private final List<HubReading> hubs = new ArrayList<>();
    private final Map<String, HubReading> lastByLabel = new HashMap<>();
    private final List<String> currentErrors = new ArrayList<>();

    private boolean haveLastSample;
    private boolean haveLastPack;
    private boolean octoWasUnhealthy;
    private double packAmps;
    private double lastPackAmps;
    private double packAmpsMax = Double.NEGATIVE_INFINITY;
    private double chVinMin = Double.POSITIVE_INFINITY;
    private double chVinMax = Double.NEGATIVE_INFINITY;
    private double ehVinMin = Double.POSITIVE_INFINITY;
    private double ehVinMax = Double.NEGATIVE_INFINITY;

    public HubPowerMonitor(HardwareMap hardwareMap, OctoQuadOdometry octoquad) {
        this.hardwareMap = hardwareMap;
        this.octoquad = octoquad;
    }

    public void update() {
        currentErrors.clear();
        boolean octoUnhealthy = octoquadHasErrors();
        boolean octoJustFailed = octoUnhealthy && !octoWasUnhealthy;
        boolean force = octoJustFailed || octoquad.hasUptimeReset();
        if (!force && haveLastSample && sampleTimer.seconds() < SAMPLE_PERIOD_SEC) {
            octoWasUnhealthy = octoUnhealthy;
            return;
        }
        boolean firstSample = !haveLastSample;
        sampleHubs();
        sampleTimer.reset();
        checkOctoReboot();
        checkVoltageEvents(octoJustFailed);
        if (firstSample) {
            RobotLog.i("[POWER] %s", snapshotLine("start"));
            baselineLogTimer.reset();
        } else if (baselineLogTimer.seconds() >= BASELINE_LOG_PERIOD_SEC) {
            RobotLog.i("[POWER] %s", snapshotLine("ok"));
            baselineLogTimer.reset();
        }
        octoWasUnhealthy = octoUnhealthy;
    }

    public String getTelemetryLine() {
        if (hubs.isEmpty()) {
            return "NO HUBS";
        }
        StringBuilder line = new StringBuilder();
        line.append(String.format(Locale.US, "pack=%.2fA", packAmps));
        for (HubReading hub : hubs) {
            line.append(" | ").append(hub.shortLine());
        }
        line.append(" | OCTO ").append(octoquad.getUptimeStatus());
        return line.toString();
    }

    public List<String> getCurrentErrors() {
        return currentErrors;
    }

    public void logSummary() {
        RobotLog.i("[POWER] SUMMARY CH vin min=%.2f max=%.2f | EH vin min=%.2f max=%.2f | pack max=%.2fA | %s",
                finiteOrZero(chVinMin), finiteOrZero(chVinMax),
                finiteOrZero(ehVinMin), finiteOrZero(ehVinMax),
                finiteOrZero(packAmpsMax),
                snapshotLine("end"));
    }

    private void sampleHubs() {
        List<HubReading> next = new ArrayList<>();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            next.add(readHub(hub));
        }
        for (HubReading hub : next) {
            if (!hub.ok || hub.vin < INVALID_VIN_V) {
                continue;
            }
            if ("CH".equals(hub.label)) {
                chVinMin = Math.min(chVinMin, hub.vin);
                chVinMax = Math.max(chVinMax, hub.vin);
            } else {
                ehVinMin = Math.min(ehVinMin, hub.vin);
                ehVinMax = Math.max(ehVinMax, hub.vin);
            }
        }
        hubs.clear();
        hubs.addAll(next);
        packAmps = 0;
        for (HubReading hub : hubs) {
            if (hub.ok) {
                packAmps += hub.amps;
            }
        }
        packAmpsMax = Math.max(packAmpsMax, packAmps);
        haveLastSample = true;
    }

    private HubReading readHub(LynxModule hub) {
        HubReading reading = new HubReading();
        reading.label = hub.isParent() ? "CH" : "EH";
        try {
            reading.vin = hub.getInputVoltage(VoltageUnit.VOLTS);
            reading.v5 = hub.getAuxiliaryVoltage(VoltageUnit.VOLTS);
            reading.amps = hub.getCurrent(CurrentUnit.AMPS);
            reading.i2cMa = hub.getI2cBusCurrent(CurrentUnit.MILLIAMPS);
            reading.ok = true;
        } catch (RuntimeException e) {
            reading.ok = false;
        }
        return reading;
    }

    private void checkOctoReboot() {
        if (octoquad.consumeUptimeReset()) {
            currentErrors.add("octoquad rebooted");
            RobotLog.w("[POWER] REBOOT %s", snapshotLine("uptime reset"));
        }
    }

    private void checkVoltageEvents(boolean octoJustFailed) {
        for (HubReading hub : hubs) {
            if (!hub.ok) {
                continue;
            }
            if (hub.vin >= INVALID_VIN_V && hub.vin < TunableValues.POWER_VIN_LOW_V) {
                currentErrors.add(hub.label + " vin low");
            }
            if (hub.v5 >= 1.0 && hub.v5 < TunableValues.POWER_V5_LOW_V) {
                currentErrors.add(hub.label + " 5V low");
            }
            HubReading previous = lastByLabel.get(hub.label);
            if (previous != null && previous.ok) {
                logStep(hub.label + " vin", previous.vin, hub.vin, TunableValues.POWER_VIN_STEP_V);
                logStep(hub.label + " 5V", previous.v5, hub.v5, TunableValues.POWER_V5_STEP_V);
                logStep(hub.label + " A", previous.amps, hub.amps, TunableValues.POWER_CURRENT_STEP_A);
            }
            lastByLabel.put(hub.label, hub);
        }
        if (packAmps >= TunableValues.POWER_PACK_HIGH_A) {
            currentErrors.add("pack A high");
        }
        if (haveLastPack) {
            logStep("pack A", lastPackAmps, packAmps, TunableValues.POWER_CURRENT_STEP_A);
        }
        lastPackAmps = packAmps;
        haveLastPack = true;
        if (octoJustFailed) {
            RobotLog.w("[POWER] OCTO %s", snapshotLine("octoquad error"));
        }
    }

    private void logStep(String field, double previous, double current, double step) {
        double delta = current - previous;
        if (Math.abs(delta) < step) {
            return;
        }
        String kind = delta < 0 ? "DROP" : "RISE";
        RobotLog.w("[POWER] %s %s %.2f->%.2f d=%+.2f | %s",
                kind, field, previous, current, delta, snapshotLine(kind));
    }

    private boolean octoquadHasErrors() {
        List<String> errors = octoquad.getCurrentHardwareErrors();
        for (String error : errors) {
            if (error.toLowerCase(Locale.US).contains("octoquad")) {
                return true;
            }
        }
        return false;
    }

    private String snapshotLine(String reason) {
        StringBuilder line = new StringBuilder(reason);
        line.append(String.format(Locale.US, " | pack=%.2fA", packAmps));
        for (HubReading hub : hubs) {
            line.append(" | ").append(hub.detailLine());
        }
        line.append(" | octo ").append(octoquad.getUptimeStatus());
        List<String> octoErrors = octoquad.getCurrentHardwareErrors();
        if (!octoErrors.isEmpty()) {
            line.append(" errors=").append(String.join(",", octoErrors));
        }
        return line.toString();
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static final class HubReading {
        String label = "?";
        double vin;
        double v5;
        double amps;
        double i2cMa;
        boolean ok;

        String shortLine() {
            if (!ok) {
                return label + " READ FAIL";
            }
            return String.format(Locale.US, " %s %.2fV 5V=%.2f A=%.2f i2c=%.0fmA",
                    label, vin, v5, amps, i2cMa).trim();
        }

        String detailLine() {
            if (!ok) {
                return label + " READ FAIL";
            }
            return String.format(Locale.US, "%s vin=%.2f v5=%.2f A=%.2f i2c=%.0fmA",
                    label, vin, v5, amps, i2cMa);
        }
    }
}
