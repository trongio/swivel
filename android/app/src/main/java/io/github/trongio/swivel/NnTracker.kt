package io.github.trongio.swivel

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.os.SystemClock
import androidx.camera.core.ImageProxy
import java.nio.FloatBuffer
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * OpenTrack's neural-net head tracker on the phone (port of tracker-neuralnet/model_adapters.cpp
 * and Swivel's swivel_cam.py): head localizer finds the head, the pose model reads its rotation
 * from a 129x129 grayscale crop. Robust to ~65 degrees of turn, unlike face-landmark fitting.
 */
class NnTracker(context: Context, private val onPose: (Pose, Long) -> Unit) {
    private val env = OrtEnvironment.getEnvironment()
    private val loc: OrtSession
    private val pose: OrtSession
    private val filters = Array(3) { OneEuro() }
    private var lastT = 0L
    private var box: FloatArray? = null  // x, y, w, h in frame pixels
    var accel = ""
        private set
    private val useMirror = false  // averaging with the mirrored crop: steadier, but doubles inference time

    // working frame: long side 640 px, either orientation (phone upright or sideways)
    private var fw = 640
    private var fh = 360
    private var gray = FloatArray(fw * fh)
    private var pixels = IntArray(fw * fh)

    init {
        fun session(name: String): OrtSession {
            val bytes = context.assets.open(name).use { it.readBytes() }
            // Qualcomm NPU (QNN HTP) first, then NNAPI, then XNNPACK on the CPU.
            try {
                val qnn = OrtSession.SessionOptions().apply {
                    addQnn(mapOf("backend_path" to "libQnnHtp.so", "enable_htp_fp16_precision" to "1",
                        "htp_performance_mode" to "burst"))
                }
                return env.createSession(bytes, qnn).also { accel = "NPU" }
            } catch (e: Exception) { android.util.Log.w("Swivel", "QNN unavailable for $name", e) }
            try {
                val nn = OrtSession.SessionOptions().apply { addNnapi() }
                return env.createSession(bytes, nn).also { accel = "NNAPI" }
            } catch (_: Exception) {}
            val opts = OrtSession.SessionOptions().apply {
                setIntraOpNumThreads(4)
                try { addXnnpack(mapOf("intra_op_num_threads" to "4")) } catch (_: Exception) {}
            }
            accel = "CPU"
            return env.createSession(bytes, opts)
        }
        loc = session("head-localizer.onnx")
        pose = session("head-pose-0.5-big.onnx")
    }

    fun process(image: ImageProxy) {
        try {
            val rot = image.imageInfo.rotationDegrees
            val src = image.toBitmap()
            val k = 640f / max(src.width, src.height)
            val sw = (src.width * k).toInt(); val sh = (src.height * k).toInt()
            val scaled = Bitmap.createScaledBitmap(src, sw, sh, true)
            val frame = if (rot == 0) scaled else Bitmap.createBitmap(scaled, 0, 0, sw, sh,
                Matrix().apply { postRotate(rot.toFloat()) }, true)
            if (frame.width != fw || frame.height != fh) {  // orientation changed
                fw = frame.width; fh = frame.height
                gray = FloatArray(fw * fh); pixels = IntArray(fw * fh)
                box = null
            }
            frame.getPixels(pixels, 0, fw, 0, 0, fw, fh)
            for (i in pixels.indices) {
                val p = pixels[i]
                gray[i] = (0.299f * ((p shr 16) and 255) + 0.587f * ((p shr 8) and 255) + 0.114f * (p and 255))
            }
            val now = SystemClock.uptimeMillis()
            val ang = track()
            if (ang != null) {
                val dt = if (lastT == 0L) 1.0 / 30 else ((now - lastT) / 1000.0).coerceIn(0.005, 0.5)
                lastT = now
                onPose(Pose(filters[0].filter(ang[0], dt), filters[1].filter(ang[1], dt),
                    filters[2].filter(ang[2], dt), true), now)
            } else {
                filters.forEach { it.reset() }
                lastT = 0L
                onPose(Pose(0.0, 0.0, 0.0, false), now)
            }
        } finally {
            image.close()
        }
    }

    /** Bilinear sample of the grayscale frame. */
    private fun sample(x: Float, y: Float): Float {
        val cx = x.coerceIn(0f, fw - 1.001f)
        val cy = y.coerceIn(0f, fh - 1.001f)
        val x0 = cx.toInt(); val y0 = cy.toInt()
        val ax = cx - x0; val ay = cy - y0
        val i = y0 * fw + x0
        return (gray[i] * (1 - ax) + gray[i + 1] * ax) * (1 - ay) + (gray[i + fw] * (1 - ax) + gray[i + fw + 1] * ax) * ay
    }

    private fun localize(): FloatArray? {
        val w = 288; val h = 224
        val input = FloatArray(w * h)
        val sx = fw.toFloat() / w; val sy = fh.toFloat() / h
        for (y in 0 until h) for (x in 0 until w) input[y * w + x] = sample((x + 0.5f) * sx, (y + 0.5f) * sy) / 255f - 0.5f
        OnnxTensor.createTensor(env, FloatBuffer.wrap(input), longArrayOf(1, 1, h.toLong(), w.toLong())).use { t ->
            loc.run(mapOf("x" to t)).use { res ->
                @Suppress("UNCHECKED_CAST")
                val r = (res[0].value as Array<FloatArray>)[0]
                if (1f / (1f + exp(-r[0])) < 0.5f) return null
                fun un(v: Float) = 0.5f * (v + 1f)
                val x0 = un(r[1]) * fw; val y0 = un(r[2]) * fh; val x1 = un(r[3]) * fw; val y1 = un(r[4]) * fh
                return floatArrayOf(x0, y0, x1 - x0, y1 - y0)
            }
        }
    }

    private class PoseOut(val quat: FloatArray, val box: FloatArray, val unc: Float)

    private fun runPose(crop: FloatArray): PoseOut {
        OnnxTensor.createTensor(env, FloatBuffer.wrap(crop), longArrayOf(1, 1, 129, 129)).use { t ->
            pose.run(mapOf("x" to t)).use { res ->
                @Suppress("UNCHECKED_CAST")
                val q = (res.get("quat").get().value as Array<FloatArray>)[0]
                @Suppress("UNCHECKED_CAST")
                val b = (res.get("box").get().value as Array<FloatArray>)[0]
                var unc = 0f
                res.get("rotaxis_scales_tril").ifPresent { v ->
                    @Suppress("UNCHECKED_CAST")
                    val m = (v.value as Array<Array<FloatArray>>)[0]
                    for (row in m) for (c in row) unc = max(unc, kotlin.math.abs(c))
                }
                return PoseOut(q.copyOf(), b.copyOf(), unc)
            }
        }
    }

    private fun rotvec(q: FloatArray): DoubleArray {
        var w = q[3].toDouble(); var x = q[0].toDouble(); var y = q[1].toDouble(); var z = q[2].toDouble()
        if (w < 0) { w = -w; x = -x; y = -y; z = -z }
        val n = sqrt(x * x + y * y + z * z)
        if (n < 1e-9) return doubleArrayOf(0.0, 0.0, 0.0)
        val ang = Math.toDegrees(2 * atan2(n, w))
        return doubleArrayOf(x / n * ang, y / n * ang, z / n * ang)
    }

    /** Returns yaw, pitch, roll (degrees) or null when no head is found. */
    private fun track(): DoubleArray? {
        val b = box ?: localize()?.also { box = it } ?: return null
        val size = max(b[2], b[3])
        if (size < 16f) { box = null; return null }
        val cx = min(max(b[0] + 0.5f * b[2], 0f), fw.toFloat())
        val cy = min(max(b[1] + 0.5f * b[3], 0f), fh.toFloat())
        // 129x129 crop and its mirror image
        val crop = FloatArray(129 * 129)
        val step = size / 129f
        for (y in 0 until 129) for (x in 0 until 129)
            crop[y * 129 + x] = sample(cx - size / 2 + (x + 0.5f) * step, cy - size / 2 + (y + 0.5f) * step)
        // brightness normalisation (90th percentile), as in OpenTrack
        val sorted = crop.copyOf().also { it.sort() }
        val bright = max(5, sorted[(sorted.size * 0.9).toInt()].toInt())
        val alpha = if (bright < 127) 0.9f * 0.5f / bright else 1f / 255f
        for (i in crop.indices) crop[i] = crop[i] * alpha - 0.5f
        val mirror = if (useMirror) FloatArray(crop.size).also { mr ->
            for (y in 0 until 129) for (x in 0 until 129) mr[y * 129 + x] = crop[y * 129 + (128 - x)]
        } else crop

        val a = runPose(crop)
        val m = if (useMirror) runPose(mirror) else null
        val half = 0.5f * size
        val nb = floatArrayOf(cx + half * a.box[0], cy + half * a.box[1], half * (a.box[2] - a.box[0]), half * (a.box[3] - a.box[1]))
        box = if (nb[2] > 8 && nb[3] > 8 && a.unc < 0.5f) {
            val old = box
            if (old == null) nb else FloatArray(4) { old[it] + 0.35f * (nb[it] - old[it]) }
        } else null
        val r = rotvec(a.quat)
        if (m == null) return doubleArrayOf(r[1], r[0], r[2])
        val rm = rotvec(m.quat)
        // mirroring negates rotation about the vertical (y) and viewing (z) axes
        val rx = (r[0] + rm[0]) / 2; val ry = (r[1] - rm[1]) / 2; val rz = (r[2] - rm[2]) / 2
        return doubleArrayOf(ry, rx, rz)
    }

    fun close() { loc.close(); pose.close() }
}
