package frc.robot.subsystems.background;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import autobahn.client.NamedCallback;
import frc.robot.Robot;
import frc.robot.constants.PiConstants;
import proto.status.StateLoggingOuterClass.StateLogging;
import proto.status.PiStatusOuterClass.LogMessage;
import proto.status.PiStatusOuterClass.PiStatus;
import proto.status.StateLoggingOuterClass.DataEntry;
import proto.status.StateLoggingOuterClass.DataType;
import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PiLogToAKit extends SubsystemBase {

    private static PiLogToAKit self;
    private final String piAkitTopic = PiConstants.AutobahnConfig.piTechnicalLogTopic + "/akit";
    private final String piStatusTopic = PiConstants.AutobahnConfig.piTechnicalLogTopic + "/status";

    public static PiLogToAKit GetInstance() {
        if (self == null) {
            self = new PiLogToAKit();
        }

        return self;
    }

    public PiLogToAKit() {
        super();

        Robot.communication.subscribe(piAkitTopic,
                NamedCallback.FromConsumer(this::subscriptionAkit));

        Robot.communication.subscribe(piStatusTopic,
                NamedCallback.FromConsumer(this::subscriptionStats));

        Robot.communication.subscribe(PiConstants.AutobahnConfig.piTechnicalLogTopic,
                NamedCallback.FromConsumer(this::logSubscription));
    }

    private void logSubscription(byte[] data) {
        try {
            LogMessage piLog = LogMessage.parseFrom(data);
            String name = piLog.getPiName() == null ? "unknown-pi" : piLog.getPiName();
            Logger.recordOutput(name + "/piLog/message", piLog.getMessage());
            Logger.recordOutput(name + "/piLog/type", piLog.getType().toString());
            Logger.recordOutput(name + "/piLog/prefix", piLog.getPrefix());
            Logger.recordOutput(name + "/piLog/color", piLog.getColor());
        } catch (InvalidProtocolBufferException e) {
            e.printStackTrace();
        }
    }

    private void subscriptionStats(byte[] data) {
        try {
            PiStatus piStatus = PiStatus.parseFrom(data);
            Logger.recordOutput(piStatus.getPiName() + "/piStats/cpuUsage", piStatus.getCpuUsageTotal());
            Logger.recordOutput(piStatus.getPiName() + "/piStats/memoryUsage", piStatus.getMemoryUsage());
            Logger.recordOutput(piStatus.getPiName() + "/piStats/diskUsage", piStatus.getDiskUsage());
            Logger.recordOutput(piStatus.getPiName() + "/piStats/networkUsageIn", piStatus.getNetUsageIn());
            Logger.recordOutput(piStatus.getPiName() + "/piStats/networkUsageOut", piStatus.getNetUsageOut());
        } catch (InvalidProtocolBufferException e) {
            e.printStackTrace();
        }
    }

    private void subscriptionAkit(byte[] data) {
        try {
            StateLogging stateLogging = StateLogging.parseFrom(data);
            for (DataEntry entry : stateLogging.getEntriesList()) {
                String name = stateLogging.getName();
                DataType type = entry.getType();
                String entryName = name + stateLogging.getTopic();

                switch (type) {
                    case BOOL:
                        mskrBool(entryName, entry.getBoolValuesList());
                        break;
                    case INT:
                        mskrInt(entryName, entry.getIntValuesList());
                        break;
                    case FLOAT:
                        mskrFloat(entryName, entry.getFloatValuesList());
                        break;
                    case STRING:
                        mskrString(entryName, entry.getStringValuesList());
                        break;
                    default:
                        // Unknown or unsupported type, do nothing
                        break;
                }
            }
        } catch (InvalidProtocolBufferException e) {
            e.printStackTrace();
        }
    }

    // Very jankey solution but haven't found an alternative yet
    private void mskrBool(String name, List<Boolean> bools) {
        if (bools.size() == 1) {
            Logger.recordOutput(name, bools.get(0));
        } else if (bools.size() > 1) {
            boolean[] arr = new boolean[bools.size()];
            for (int i = 0; i < bools.size(); i++)
                arr[i] = bools.get(i) != null ? bools.get(i) : false;
            Logger.recordOutput(name, arr);
        }
    }

    private void mskrInt(String name, List<Integer> ints) {
        if (ints.size() == 1) {
            Logger.recordOutput(name, ints.get(0));
        } else if (ints.size() > 1) {
            int[] arr = new int[ints.size()];
            for (int i = 0; i < ints.size(); i++)
                arr[i] = ints.get(i);
            Logger.recordOutput(name, arr);
        }
    }

    private void mskrFloat(String name, List<Float> floats) {
        if (floats.size() == 1) {
            Logger.recordOutput(name, floats.get(0));
        } else if (floats.size() > 1) {
            double[] arr = new double[floats.size()];
            for (int i = 0; i < floats.size(); i++)
                arr[i] = floats.get(i);
            Logger.recordOutput(name, arr);
        }
    }

    private void mskrString(String name, List<String> strings) {
        if (strings.size() == 1) {
            Logger.recordOutput(name, strings.get(0));
        } else if (strings.size() > 1) {
            Logger.recordOutput(name, strings.toArray(new String[0]));
        }
    }
}
