import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.Constant
import com.scrapmetal.util.control.pathing.Linear
import com.scrapmetal.util.control.pathing.Spline
import com.scrapmetal.util.control.pathing.SplinePoint
import com.scrapmetal.util.control.pathing.SubMovement
import com.scrapmetal.util.control.pathing.Tangent
import com.scrapmetal.util.control.pathing.to
import com.scrapmetal.util.control.pathing.withEffort
import com.scrapmetal.util.control.pathing.withHeading
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.time.measureTimedValue

class PathingTest {
    @Test
    fun testClosestT() {
        val spline = Spline(
            start = Vector2d(0.0, 0.0),
            startTangent = Rotation2d(68.0)*Vector2d(28.0, 0.0),
            end = Vector2d(24.0, 2.0),
            endTangent = Rotation2d(90.0)*Vector2d(40.0, 00.0),
        )
        repeat (1000) {
            val pos = Vector2d(
                Random.nextDouble(-20.0, 50.0),
                Random.nextDouble(-20.0, 20.0)
            )
            println("pos = $pos")
            val (efficientClosestPoint, efficientTime) = measureTimedValue { spline.closestPoint(pos) }
            println("efficient = $efficientClosestPoint")
            println("efficient time = $efficientTime")
            val (bruteForceClosestPoint, bruteForceTime) = measureTimedValue { spline.closestPointBruteforce(pos) }
            println("brute force = $bruteForceClosestPoint")
            println("brute force time = $bruteForceTime")
            println("")

            assert((efficientClosestPoint - bruteForceClosestPoint).norm() < 0.05)
        }
    }

    @Test
    fun testMovementBuilder() {
        val movement = SplinePoint(0.0, 0.0, 28.0, 68.0) to
                SplinePoint(24.0, 2.0, 40.0, 90.0) withHeading Constant(10.0) to
                SplinePoint(1.0, 2.0, 3.0, 4.0) to
                SplinePoint(5.0, 6.0, 7.0, 8.0) withEffort 0.2 to
                SplinePoint(9.0, 10.0, 11.0, 12.0) withHeading Linear(10.0, 180.0) withEffort 0.5

        assertEquals(movement.submovements.size, 4)
        assertEquals(
            movement.submovements[0],
            SubMovement(
                Spline(Vector2d(0.0, 0.0), Rotation2d(68.0)*Vector2d(28.0, 0.0), Vector2d(24.0, 2.0), Rotation2d(90.0)*Vector2d(40.0, 0.0)),
                Constant(10.0),
                1.0,
            )
        )
        assertEquals(
            movement.submovements[1],
            SubMovement(
                Spline(Vector2d(24.0, 2.0), Rotation2d(90.0)*Vector2d(40.0, 0.0), Vector2d(1.0, 2.0), Rotation2d(4.0)*Vector2d(3.0, 0.0)),
                Tangent(Spline(Vector2d(24.0, 2.0), Rotation2d(90.0)*Vector2d(40.0, 0.0), Vector2d(1.0, 2.0), Rotation2d(4.0)*Vector2d(3.0, 0.0))),
                1.0,
            )
        )
        assertEquals(
            movement.submovements[2],
            SubMovement(
                Spline(Vector2d(1.0, 2.0), Rotation2d(4.0)*Vector2d(3.0, 0.0), Vector2d(5.0, 6.0), Rotation2d(8.0)*Vector2d(7.0, 0.0)),
                Tangent(Spline(Vector2d(1.0, 2.0), Rotation2d(4.0)*Vector2d(3.0, 0.0), Vector2d(5.0, 6.0), Rotation2d(8.0)*Vector2d(7.0, 0.0))),
                0.2,
            )
        )
        assertEquals(
            movement.submovements[3],
            SubMovement(
                Spline(Vector2d(5.0, 6.0), Rotation2d(8.0)*Vector2d(7.0, 0.0), Vector2d(9.0, 10.0), Rotation2d(12.0)*Vector2d(11.0, 0.0)),
                Linear(10.0, 180.0),
                0.5,
            )
        )
    }
}