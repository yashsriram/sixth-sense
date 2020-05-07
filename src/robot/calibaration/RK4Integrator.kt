package robot.calibaration

import extensions.plus
import extensions.plusAssign
import extensions.times
import extensions.timesAssign
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix3
import kotlin.math.cos
import kotlin.math.sin

class RK4Integrator {
    companion object {
        private fun getChangeInPose(pose: FMatrix3, control: FMatrix2): FMatrix3 {
            val changeInPose = FMatrix3()
            changeInPose.a1 = control.a1 * cos(pose.a3)
            changeInPose.a2 = control.a1 * sin(pose.a3)
            changeInPose.a3 = control.a2
            return changeInPose
        }

        fun updatePose(pose: FMatrix3, control: FMatrix2, dt: Float): FMatrix3 {
            // Run the dynamics via RK4
            val k1 = getChangeInPose(pose, control)
            val x2 = pose + k1 * (0.5f * dt)
            val k2 = getChangeInPose(x2, control)
            val x3 = pose + k2 * (0.5f * dt)
            val k3 = getChangeInPose(x3, control)
            val x4 = pose + k3 * dt
            val k4 = getChangeInPose(x4, control)
            val avgChangeInPose = FMatrix3()
            avgChangeInPose += k1
            avgChangeInPose += k2 * 2f
            avgChangeInPose += k3 * 2f
            avgChangeInPose += k4
            avgChangeInPose *= dt / 6f
            return pose + avgChangeInPose
        }

    }
}