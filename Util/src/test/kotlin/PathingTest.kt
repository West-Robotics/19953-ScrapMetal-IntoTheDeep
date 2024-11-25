import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.Spline
import com.scrapmetal.util.control.pathing.closestPoint
import com.scrapmetal.util.control.pathing.closestPointBruteforce
import kotlin.random.Random
import kotlin.test.Test
import kotlin.time.measureTimedValue

class PathingTest {
    val spline = Spline(
        start = Vector2d(0.0, 0.0),
        v0 = Vector2d(26.0, 10.0),
        end = Vector2d(24.0, 2.0),
        v1 = Vector2d(0.0, 40.0),
    )

    @Test
    fun testClosestT() {
        repeat (1000) {
            val pos = Vector2d(
                Random.nextDouble(-20.0, 50.0),
                Random.nextDouble(-20.0, 20.0)
            )
            println("pos = $pos")
            val (efficientClosestPoint, efficientTime) = measureTimedValue { closestPoint(pos, spline) }
            println("efficient = $efficientClosestPoint")
            println("efficient time = $efficientTime")
            val (bruteForceClosestPoint, bruteForceTime) = measureTimedValue { closestPointBruteforce(pos, spline) }
            println("brute force = $bruteForceClosestPoint")
            println("brute force time = $bruteForceTime")
            println("")

            assert((efficientClosestPoint - bruteForceClosestPoint).norm() < 0.05)
        }
    }

}