package io.github.trongio.swivel

import android.content.Context
import android.os.SystemClock
import android.util.Log
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.core.Delegate
import com.google.mediapipe.tasks.vision.core.ImageProcessingOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.facelandmarker.FaceLandmarker
import androidx.camera.core.ImageProxy
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2

/** Adaptive low-pass: steady when still, responsive when moving (degrees). */
class OneEuro(private val minCutoff: Double = 0.8, private val beta: Double = 0.02) {
    private var x: Double? = null
    private var dx = 0.0
    private fun alpha(cutoff: Double, dt: Double) = 1.0 / (1.0 + 1.0 / (2 * PI * cutoff * dt))
    fun reset() { x = null; dx = 0.0 }
    fun filter(v: Double, dt: Double): Double {
        val prev = x ?: run { x = v; return v }
        dx += alpha(1.0, dt) * ((v - prev) / dt - dx)
        val out = prev + alpha(minCutoff + beta * abs(dx), dt) * (v - prev)
        x = out
        return out
    }
}

data class Pose(val yaw: Double, val pitch: Double, val roll: Double, val valid: Boolean)

/**
 * Streams poses to Swivel on the PC over TCP. With USB, `adb reverse tcp:4244 tcp:4244`
 * (Swivel runs it) makes 127.0.0.1:4244 on the phone reach the PC. Packet = 7
 * little-endian doubles: x, y, z, yaw, pitch, roll, valid (same as swivel_cam.py).
 */
class Streamer(private val host: String = "127.0.0.1", private val port: Int = 4244) {
    @Volatile var connected = false
        private set
    @Volatile private var latest: Pose? = null
    @Volatile private var running = true
    private val lock = Object()

    fun send(p: Pose) {
        latest = p
        synchronized(lock) { lock.notifyAll() }
    }

    fun stop() {
        running = false
        synchronized(lock) { lock.notifyAll() }
    }

    fun start() = Thread {
        val buf = ByteBuffer.allocate(56).order(ByteOrder.LITTLE_ENDIAN)
        while (running) {
            try {
                Socket().use { s ->
                    s.tcpNoDelay = true
                    s.connect(InetSocketAddress(host, port), 1000)
                    connected = true
                    val out: OutputStream = s.getOutputStream()
                    while (running) {
                        val p = synchronized(lock) {
                            var v = latest
                            while (v == null && running) { lock.wait(500); v = latest }
                            latest = null
                            v
                        } ?: continue
                        buf.clear()
                        buf.putDouble(0.0).putDouble(0.0).putDouble(0.0)
                        buf.putDouble(p.yaw).putDouble(p.pitch).putDouble(p.roll)
                        buf.putDouble(if (p.valid) 1.0 else 0.0)
                        out.write(buf.array())
                        out.flush()
                    }
                }
            } catch (e: Exception) {
                connected = false
                Thread.sleep(1000)
            }
        }
        connected = false
    }.apply { isDaemon = true; start() }
}

/** MediaPipe Face Landmarker on the GPU, live-stream mode. */
class HeadTracker(context: Context, private val onPose: (Pose, Long) -> Unit) {
    private val filters = Array(3) { OneEuro() }
    private var lastT = 0L
    private val landmarker: FaceLandmarker

    init {
        fun build(delegate: Delegate): FaceLandmarker {
            val opts = FaceLandmarker.FaceLandmarkerOptions.builder()
                .setBaseOptions(BaseOptions.builder().setModelAssetPath("face_landmarker.task").setDelegate(delegate).build())
                .setRunningMode(RunningMode.LIVE_STREAM)
                .setNumFaces(1)
                .setOutputFacialTransformationMatrixes(true)
                .setResultListener { result, _ ->
                    val now = SystemClock.uptimeMillis()
                    val mats = result.facialTransformationMatrixes()
                    if (mats.isPresent && mats.get().isNotEmpty()) {
                        val m = mats.get()[0]  // 4x4, column-major; Swivel's calibration learns signs/scale
                        fun at(r: Int, c: Int) = m[c * 4 + r].toDouble()
                        val yaw = Math.toDegrees(atan2(at(0, 2), at(2, 2)))
                        val pitch = Math.toDegrees(asin((-at(1, 2)).coerceIn(-1.0, 1.0)))
                        val roll = Math.toDegrees(atan2(at(1, 0), at(1, 1)))
                        val dt = if (lastT == 0L) 1.0 / 30 else ((now - lastT) / 1000.0).coerceIn(0.005, 0.5)
                        lastT = now
                        onPose(Pose(filters[0].filter(yaw, dt), filters[1].filter(pitch, dt),
                            filters[2].filter(roll, dt), true), now)
                    } else {
                        filters.forEach { it.reset() }
                        lastT = 0L
                        onPose(Pose(0.0, 0.0, 0.0, false), now)
                    }
                }
                .setErrorListener { e -> Log.e("Swivel", "landmarker", e) }
                .build()
            return FaceLandmarker.createFromOptions(context, opts)
        }
        landmarker = try { build(Delegate.GPU) } catch (e: Exception) {
            Log.w("Swivel", "GPU delegate failed, using CPU", e); build(Delegate.CPU)
        }
    }

    fun process(image: ImageProxy) {
        try {
            val bmp = image.toBitmap()
            val opts = ImageProcessingOptions.builder().setRotationDegrees(image.imageInfo.rotationDegrees).build()
            landmarker.detectAsync(BitmapImageBuilder(bmp).build(), opts, SystemClock.uptimeMillis())
        } finally {
            image.close()
        }
    }

    fun close() = landmarker.close()
}
