package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pControl

/**
 * Guiding vector field follower that gives the (non-normalized) drivetrain effort to follow a
 * [Movement], given the error vector gain [kN]. The follower will switch to proportional-derivative
 * feedback within a certain [endDistance] from the end point of the [Movement], with gains of
 * position and heading, [kP], [kD], [kTheta], and [kOmega] respectively.
 */
class Follower(
    private val kN: Double,
    private val kP: Double,
    private val kD: Double,
    private val kTheta: Double,
    private val kOmega: Double,
    private val endDistance: Double,
) {
    lateinit var movement: Movement

    fun follow(newMovement: Movement) {
        movement = newMovement
    }

    fun update(pose: Pose2d, velocity: Pose2d): Pose2d {
        val closest = movement(pose.position)
        val error = closest.pose.position - pose.position
        // standard GVF equation for desired motion:
        //   normalize the unit tangent of closest point + scaled error vector
        val mD = (closest.derivative.position.unit + error * kN).unit
        // project mD onto the unit tangent, getting the tangent component of mD
        val mD_tau = closest.derivative.position.unit * (mD dot closest.derivative.position.unit)
        // WARNING: ??????

        // remove the tangent component and replace it with a version scaled by the effort specified
        //   in the submovement
        // we only scale the tangent component because we want to keep the full corrective power, we
        //   just don't want to travel along the path as quickly
        // however, if we're within the end distance, switch to a PD controller
        val translationEffort = if (
            !atEnd(pose.position, endDistance)
        ) {
            mD - mD_tau + mD_tau*closest.derivative.position.norm
        } else {
            val fieldFrameEffort = pControl(kP, movement.endPoint, pose.position) +
                pControl(kD, Vector2d(0.0, 0.0), velocity.position)
            val robotFrameEffort = pose.heading.inverse * fieldFrameEffort
            robotFrameEffort
        }

        // PD control on heading
        val rotationEffort = Rotation2d(
            pControl(
                kTheta,
                closest.pose.heading.theta,
                pose.heading.theta,
                wraparound = true,
            ) + pControl(
                kOmega,
                0.0,
                // TODO: try this out
                // closest.derivative.heading.theta,
                velocity.heading.theta,
            )
        )

        return Pose2d(translationEffort, rotationEffort)
        // TODO: Deal with units of closest derivative heading and velocity heading
        // TODO: Deal with heading derivative being a function of the translational velocity
        //   projection onto the tangent, as well as path length
        // TODO: Implement coast to stop at endpoint
    }

    fun atEnd(pos: Vector2d, threshold: Double) = movement.isLast() && (movement.endPoint - pos).norm < threshold
}