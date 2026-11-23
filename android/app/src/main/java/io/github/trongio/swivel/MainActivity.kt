package io.github.trongio.swivel

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Size
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import kotlinx.coroutines.delay
import java.util.concurrent.Executors

class MainActivity : ComponentActivity() {
    private val analysisExecutor = Executors.newSingleThreadExecutor()
    private val streamer = Streamer()
    private var tracker: NnTracker? = null

    private var pose by mutableStateOf(Pose(0.0, 0.0, 0.0, false))
    private var fps by mutableStateOf(0.0)
    private var frames = 0
    private var fpsT = System.currentTimeMillis()
    private var hasCamera by mutableStateOf(false)

    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { ok ->
        hasCamera = ok
        if (ok) startCamera()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        streamer.start()
        tracker = NnTracker(this) { p, _ ->
            streamer.send(p)
            pose = p
            frames++
            val now = System.currentTimeMillis()
            if (now - fpsT >= 1000) { fps = frames * 1000.0 / (now - fpsT); frames = 0; fpsT = now }
        }
        hasCamera = ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
        setContent { Screen() }
        if (hasCamera) startCamera() else askCamera.launch(Manifest.permission.CAMERA)
    }

    private lateinit var previewView: PreviewView

    private fun startCamera() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            val provider = future.get()
            val res = ResolutionSelector.Builder()
                .setResolutionStrategy(ResolutionStrategy(Size(640, 480), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
                .build()
            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(res)
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
            analysis.setAnalyzer(analysisExecutor) { img -> tracker?.process(img) ?: img.close() }
            val preview = Preview.Builder().build()
            if (::previewView.isInitialized) preview.surfaceProvider = previewView.surfaceProvider
            provider.unbindAll()
            provider.bindToLifecycle(this, CameraSelector.DEFAULT_FRONT_CAMERA, preview, analysis)
        }, ContextCompat.getMainExecutor(this))
    }

    override fun onDestroy() {
        streamer.stop()
        tracker?.close()
        analysisExecutor.shutdown()
        super.onDestroy()
    }

    @androidx.compose.runtime.Composable
    private fun Screen() {
        var connected by remember { mutableStateOf(false) }
        LaunchedEffect(Unit) { while (true) { connected = streamer.connected; delay(300) } }
        MaterialTheme(colorScheme = darkColorScheme()) {
            Row(Modifier.fillMaxSize().background(Color(0xFF1E1E1E)).padding(16.dp),
                horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                Box(Modifier.fillMaxHeight().width(360.dp)) {
                    AndroidView(factory = { ctx ->
                        PreviewView(ctx).also { previewView = it; if (hasCamera) startCamera() }
                    }, modifier = Modifier.fillMaxSize())
                }
                Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
                    Text("Swivel Cam", color = Color.White, fontSize = 26.sp)
                    Text(if (connected) "Connected to Swivel" else "Waiting for Swivel on the PC (USB)…",
                        color = if (connected) Color(0xFF57E389) else Color(0xFFF6D32D), fontSize = 18.sp)
                    Text(if (!hasCamera) "Camera permission needed" else if (pose.valid) "Face: tracking" else "Face: not found",
                        color = Color.White, fontSize = 18.sp)
                    Text("yaw %+.1f°   pitch %+.1f°   roll %+.1f°".format(pose.yaw, pose.pitch, pose.roll),
                        color = Color(0xFFDEDDDA), fontSize = 18.sp)
                    Text("%.0f fps · %s".format(fps, tracker?.accel ?: ""), color = Color(0xFF9A9996), fontSize = 16.sp)
                    Text("Put the phone under your monitor, facing you. Keep this app open.",
                        color = Color(0xFF9A9996), fontSize = 14.sp)
                }
            }
        }
    }
}
