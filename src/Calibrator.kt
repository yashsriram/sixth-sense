import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import processing.core.PApplet
import robot.calibaration.RK4Integrator
import simulator.Simulator
import kotlin.collections.ArrayList
class Calibrator: PApplet() {

    private var sim: Simulator? = null

    override fun setup() {
        val sceneName = "data/apartment.scn"
        sim = Simulator(this, sceneName)
        calibrate()
    }

    private fun calibrate(){
        println("Calibrating bot position covariance")
        Simulator.GHOST_MODE = true;
        val noise = FMatrixRMaj(100,3)
        val dt = 0.01f;
        val velocity = 100f
        // collect samples

        println("Collecting noise samples")
        for(x in 0..99){

            // move the bot forward
            val baselinePose = sim!!.truePose

            // rotate and move the bot
            sim!!.applyControl(FMatrix2(velocity, 0f))
            Thread.sleep(500);
            val pose = sim!!.truePose
            val estimatedPose = RK4Integrator.updatePose(baselinePose, FMatrix2(velocity, 0f), dt, PApplet.parseInt(dt * 500f));
            noise.add(x,0, estimatedPose.a1 - pose.a1)
            noise.add(x,1, estimatedPose.a2 - pose.a2)
            noise.add(x,2, estimatedPose.a3 - pose.a3)
        }

        // get column means
        println("Finding Mean")
        val mean: MutableList<Float> = ArrayList()
        val nCols = noise.numCols-1
        val nRows = noise.numRows-1
        for(x in 0..nCols){
            mean.add(noise.columnWiseMean(x));
        }

        // calculate covariance
        println("Finding Covariance")
        val covariance: FMatrixRMaj = FMatrixRMaj(3,3)
        for(i in 0..nCols){
            for(j in 0..nCols){
                var variance = 0f
                for(row in 0..nRows){
                    variance += (noise.get(row, i) - mean[i])*(noise.get(row, j) - mean[j])
                }
                variance /= noise.numRows;
                covariance.set(i, j, variance);
            }
        }

        // print covariance
        for(i in 0..2){
            for(j in 0..2){
                PApplet.print(covariance.get(i, j), ",")
            }
            PApplet.println()
        }
    }

    override fun draw() {
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("Calibrator")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}