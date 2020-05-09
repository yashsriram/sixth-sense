package extensions

import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.EigenOps_FDRM
import org.ejml.dense.row.decomposition.eig.SwitchingEigenDecomposition_FDRM
import processing.core.PApplet

fun PApplet.circleXZ(x: Float, z: Float, radius: Float) {
    beginShape()
    val resolution = 20
    for (i in 1..resolution) {
        val theta = 2 * PApplet.PI / (resolution - 1) * i
        vertex(x + radius * PApplet.cos(theta), 0f, z + radius * PApplet.sin(theta))
    }
    endShape(PApplet.CLOSE)
}

fun PApplet.covarianceXZ(x_T: FMatrixRMaj, sigma_T: FMatrixRMaj) {
    val sigma_T_copy = FMatrixRMaj(sigma_T)
    val decomposer = SwitchingEigenDecomposition_FDRM(2, true, 1e-6f)
    val success = decomposer.decompose(sigma_T_copy)
    if (!success) {
        throw IllegalStateException("Can't find eigen vectors/values of matrix")
    }
    val eigenValue1 = decomposer.getEigenvalue(0)
    val eigenValue2 = decomposer.getEigenvalue(1)
    val eigenVectors = EigenOps_FDRM.createMatrixV(decomposer)
    val ellipseTheta = PApplet.atan2(eigenVectors[1, 0], eigenVectors[0, 0])
    val sinEllipseTheta = PApplet.sin(ellipseTheta)
    val cosEllipseTheta = PApplet.cos(ellipseTheta)
    val rot = FMatrixRMaj(
            arrayOf(
                    floatArrayOf(cosEllipseTheta, -sinEllipseTheta),
                    floatArrayOf(sinEllipseTheta, cosEllipseTheta)
            )
    )
    val ellipseResolution = 20
    val ellipse = mutableListOf<FMatrixRMaj>()
    for (i in 0 until ellipseResolution) {
        // Only ellipseResolution - 1 points as the first and last points are the same (completing the loop)
        val theta = 2 * PApplet.PI * i / (ellipseResolution - 1)

        // Scale by major / minor axis, then rotate and offset
        // 5.991 (=chi2inv(.95,2)) is the 95% confidence scaling bound for a 2D covariance ellipse
        // 2f * gives 99% conficence interval
        val pointOnEllipse = FMatrixRMaj(
                arrayOf(
                        floatArrayOf(2f * PApplet.sqrt((5.991 * eigenValue1.real).toFloat()) * PApplet.cos(theta)),
                        floatArrayOf(2f * PApplet.sqrt((5.991 * eigenValue2.real).toFloat()) * PApplet.sin(theta))
                )
        )
        ellipse.add(x_T + rot * pointOnEllipse)
    }
    stroke(1f, 0f, 0f)
    for (i in 1 until ellipse.size) {
        val prevState = ellipse[i - 1]
        val currState = ellipse[i]
        line(prevState[0], 0f, prevState[1], currState[0], 0f, currState[1])
    }
}

fun PApplet.pathXZ(points: MutableList<FMatrix2>) {
    for (i in 1 until points.size) {
        val prevState = points[i - 1]
        val currState = points[i]
        line(prevState.a1, 0f, prevState.a2, currState.a1, 0f, currState.a2)
    }
}
