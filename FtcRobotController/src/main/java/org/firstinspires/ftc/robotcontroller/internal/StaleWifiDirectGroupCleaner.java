package org.firstinspires.ftc.robotcontroller.internal;

import android.content.Context;
import android.net.wifi.p2p.WifiP2pGroup;
import android.net.wifi.p2p.WifiP2pManager;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.atomic.AtomicBoolean;

public class StaleWifiDirectGroupCleaner {
    public static final String TAG = "StaleWifiDirectGroupCleaner";

    /** Give up waiting on the framework rather than hang startup. */
    private static final long TIMEOUT_MS = 4000;

    /**
     * Let the p2p stack settle after a teardown before we let the SDK create a new
     * group.
     */
    private static final long SETTLE_MS = 750;

    private static final AtomicBoolean hasRunThisProcess = new AtomicBoolean(false);

    public interface Continuation {
        void proceed();
    }

    /**
     * Clears a stale Wi-Fi Direct group, then runs {@code continuation}.
     *
     * When there is nothing to do, {@code continuation} runs synchronously, so
     * callers that did not
     * need the cleanup behave exactly as they did before. Otherwise it runs on the
     * main thread once
     * the teardown finishes or {@link #TIMEOUT_MS} elapses, whichever comes first.
     * It is invoked
     * exactly once either way.
     */
    public static void clearThenRun(Context context, final Continuation continuation) {
        if (!hasRunThisProcess.compareAndSet(false, true)) {
            continuation.proceed();
            return;
        }

        final WifiP2pManager manager = (WifiP2pManager) context.getSystemService(Context.WIFI_P2P_SERVICE);
        if (manager == null) {
            RobotLog.vv(TAG, "no WifiP2pManager; skipping stale group check");
            continuation.proceed();
            return;
        }

        final WifiP2pManager.Channel channel = manager.initialize(context, Looper.getMainLooper(), null);
        if (channel == null) {
            RobotLog.vv(TAG, "could not initialize a Wi-Fi Direct channel; skipping stale group check");
            continuation.proceed();
            return;
        }

        final Handler handler = new Handler(Looper.getMainLooper());
        final AtomicBoolean finished = new AtomicBoolean(false);

        final Runnable proceedOnce = new Runnable() {
            @Override
            public void run() {
                if (finished.compareAndSet(false, true)) {
                    closeChannel(channel);
                    continuation.proceed();
                }
            }
        };

        // The framework is not obliged to ever call us back, so bound the wait.
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (!finished.get())
                    RobotLog.ww(TAG, "timed out waiting on Wi-Fi Direct group info; continuing");
                proceedOnce.run();
            }
        }, TIMEOUT_MS);

        try {
            manager.requestGroupInfo(channel, new WifiP2pManager.GroupInfoListener() {
                @Override
                public void onGroupInfoAvailable(WifiP2pGroup group) {
                    if (group == null) {
                        RobotLog.vv(TAG, "no leftover Wi-Fi Direct group");
                        proceedOnce.run();
                        return;
                    }

                    if (!group.isGroupOwner()) {
                        // Someone else owns this group, so it is not ours to remove.
                        RobotLog.vv(TAG, "leftover group '%s' is owned by a peer; leaving it alone",
                                group.getNetworkName());
                        proceedOnce.run();
                        return;
                    }

                    RobotLog.vv(TAG, "removing stale Wi-Fi Direct group '%s' left over from a previous run",
                            group.getNetworkName());
                    manager.removeGroup(channel, new WifiP2pManager.ActionListener() {
                        @Override
                        public void onSuccess() {
                            RobotLog.vv(TAG, "stale Wi-Fi Direct group removed");
                            handler.postDelayed(proceedOnce, SETTLE_MS);
                        }

                        @Override
                        public void onFailure(int reason) {
                            RobotLog.ww(TAG, "could not remove stale Wi-Fi Direct group: %s",
                                    failureReasonToString(reason));
                            proceedOnce.run();
                        }
                    });
                }
            });
        } catch (SecurityException e) {
            // requestGroupInfo() needs ACCESS_FINE_LOCATION. The permission validator
            // normally has
            // it by now, but never block startup if it does not.
            RobotLog.ww(TAG, e, "not permitted to inspect Wi-Fi Direct groups; skipping stale group check");
            proceedOnce.run();
        }
    }

    private static void closeChannel(WifiP2pManager.Channel channel) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
            try {
                channel.close();
            } catch (RuntimeException e) {
                RobotLog.vv(TAG, "ignoring failure to close temporary Wi-Fi Direct channel: %s", e.getMessage());
            }
        }
    }

    private static String failureReasonToString(int reason) {
        switch (reason) {
            case WifiP2pManager.P2P_UNSUPPORTED:
                return "P2P_UNSUPPORTED";
            case WifiP2pManager.ERROR:
                return "ERROR";
            case WifiP2pManager.BUSY:
                return "BUSY";
            default:
                return "UNKNOWN (reason " + reason + ")";
        }
    }

    private StaleWifiDirectGroupCleaner() {
    }
}
