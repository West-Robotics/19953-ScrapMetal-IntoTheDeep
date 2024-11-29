package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.pControl

/**
 * Guiding vector field follower that gives the (non-normalized) drivetrain effort to follow a
 * [Movement], given the error vector gain [kN], and the gains for proportional feedback of heading
 * and heading velocity, [kTheta] and [kOmega] respectively.
 */
class Follower(val kN: Double, val kTheta: Double, val kOmega: Double) {
    fun follow(movement: Movement, pose: Pose2d, velocity: Pose2d): Pose2d {
        val closest = movement(pose.position)
        val error = closest.pose.position - pose.position
        // standard GVF equation for desired motion:
        //   normalize the unit tangent of closest point + scaled error vector
        val mD = (closest.derivative.position.unit() + error * kN).unit()
        // project mD onto the unit tangent, getting the tangent component of mD
        val mD_tau = closest.derivative.position.unit() * (mD dot closest.derivative.position.unit())
        // remove the tangent component and replace it with a version scaled by the effort specified
        //   in the submovement
        // we only scale the tangent component because we want to keep the full corrective power, we
        //   just don't want to travel along the path as quickly
        val translationEffort = mD - mD_tau + mD_tau*closest.derivative.position.norm()

        // proportional control on both heading and heading velocity
        val rotationEffort = Rotation2d(
            pControl(
                kTheta,
                closest.pose.heading.theta,
                pose.heading.theta,
                wraparound = true
            ) + pControl(
                kOmega,
                closest.derivative.heading.theta,
                velocity.heading.theta,
            )
        )
        TODO("Deal with units of closest derivative heading and velocity heading")
        TODO("Deal with heading derivative being a function of the translational velocity " +
                "projection onto the tangent, as well as path length")
        TODO("Implement coast to stop at endpoint")

        return Pose2d(translationEffort, rotationEffort)
    }
}