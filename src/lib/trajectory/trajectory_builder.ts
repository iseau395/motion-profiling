import { type HermiteWaypoint } from "./hermite_functions";
import type { Path } from "./path";
import { Trajectory, type TrajectoryPose } from "./trajectory";
import { Vec2 } from "./vec2";

export interface TrajectoryConstraints {
    drivetrain_rpm: number,
    wheel_diameter: number,

    max_wheel_accel: number,

    track_width: number,
    mass_lb: number,

    friction_coefficient: number,
}

const GRAVITATIONAL_ACCEL = 9.81;

export class TrajectoryBuilder
{
    readonly constraints: TrajectoryConstraints;

    constructor(constraints: TrajectoryConstraints)
    {
        this.constraints = constraints;
    }

    private static get_wheel_max_force(wheel_velocity: number, wheel_max_rpm: number, wheel_diameter: number)
    {
        const wheel_rotational_velocity = wheel_velocity / (wheel_diameter / 2);

        const percent_of_max_speed = (wheel_rotational_velocity * (60 / (2 * Math.PI))) / wheel_max_rpm;

        const wheel_radius_m = (wheel_diameter / 2) * 0.0254;

        // // account for vendor-set max speed
        // if (percent_of_max_speed < 1)
        // {
            // account for vendor-set max torque
            return ((100 / wheel_max_rpm) * Math.min(
                2.1,
                -3.275 * percent_of_max_speed + 3.965
            )) / wheel_radius_m;
        // }
        // else
        // {
        //     return 0;
        // }
    }

    private static compute_velocity_max(constraints: TrajectoryConstraints, previous: TrajectoryPose, next: TrajectoryPose, points_per_inch: number, max_speed: number)
    {
        const curvature_avg = (previous.curvature + next.curvature) / 2 || 0.00000001;
        const curvature_avg_unsigned = Math.abs(curvature_avg);

        const max_wheel_speed = (constraints.drivetrain_rpm * Math.PI * constraints.wheel_diameter) / 60;

        const mass_kg = constraints.mass_lb * 0.453592;
        // const track_width_m = constraints.track_width * 0.0254;
        // const curve_radius_m = (1 / curvature_avg_unsigned) * 0.0254;

        // const higher_wheel_velocity = Math.max(previous.left_drive_velocity, previous.right_drive_velocity);
        // const max_outer_wheel_force = 3 * this.get_wheel_max_force(higher_wheel_velocity, constraints.drivetrain_rpm, constraints.wheel_diameter);

        // const max_drive_force = Math.min(
        //     (12 * max_outer_wheel_force * curve_radius_m) / (track_width_m + 6 * curve_radius_m),
        //     constraints.friction_coefficient * 9.81 * mass_kg
        // );
        // const max_accel_m = max_drive_force / mass_kg;
        // const max_accel = 1 * (max_accel_m * 39.37);
        
        // const displacement = 1 / points_per_inch;

        // const old_center_velocity = (previous.left_drive_velocity + previous.right_drive_velocity) / 2;
        // const center_velocity = Math.min(
        //     (next.left_drive_velocity + next.right_drive_velocity) / 2,
        //     Math.sqrt(old_center_velocity * old_center_velocity + 2 * max_accel * displacement),
        //     max_wheel_speed / (1 + curvature_avg_unsigned * (constraints.track_width / 2)),
        //     max_speed,
        // );
        
        const displacement = 1 / points_per_inch;

        const left_conversion = (1 + curvature_avg * (constraints.track_width / 2));
        const right_conversion = (1 - curvature_avg * (constraints.track_width / 2));

        const left_displacement = displacement * left_conversion;
        const right_displacement = displacement * right_conversion;

        const max_left_velocity = previous.left_drive_velocity < max_wheel_speed ? Math.sign(left_displacement) * Math.min(
            max_wheel_speed,
            Math.sqrt(previous.left_drive_velocity * previous.left_drive_velocity + 2 * constraints.max_wheel_accel * Math.abs(left_displacement))
        ) : max_wheel_speed;
        const max_right_velocity =  previous.right_drive_velocity < max_wheel_speed ? Math.sign(right_displacement) * Math.min(
            max_wheel_speed,
            Math.sqrt(previous.right_drive_velocity * previous.right_drive_velocity + 2 * constraints.max_wheel_accel * Math.abs(right_displacement))
        ) : max_wheel_speed;

        const friction_max_speed = curvature_avg_unsigned != 0 ?
            39.3701 * Math.sqrt((constraints.friction_coefficient * mass_kg * 9.81) / (curvature_avg_unsigned * 39.3701))
            : Infinity;

        const center_velocity = Math.min(
            (next.left_drive_velocity + next.right_drive_velocity) / 2,
            (max_left_velocity / left_conversion) > 0 ? (max_left_velocity / left_conversion) : Infinity,
            (max_right_velocity / right_conversion) > 0 ? (max_right_velocity / right_conversion) : Infinity,
            friction_max_speed,
            max_speed
        );

        return [
            center_velocity * left_conversion, // left
            center_velocity * right_conversion  // right
        ];
        
    }

    build(path: Path, points_per_inch = 3): Trajectory
    {
        const trajectory = path.get_discretized(points_per_inch, true) as TrajectoryPose[];

        trajectory[0].left_drive_velocity = 0;
        trajectory[0].right_drive_velocity = 0;
        trajectory[0].time = 0;

        trajectory[trajectory.length-1].left_drive_velocity = 0;
        trajectory[trajectory.length-1].right_drive_velocity = 0;

        const displacement = 1 / points_per_inch;
        for (let i = 1; i < trajectory.length-1; i++)
        {
            const previous = trajectory[i-1];
            const next = trajectory[i];

            const max_speed = path.get_max_speed(next.position);

            const [left, right] = TrajectoryBuilder.compute_velocity_max(this.constraints, previous, next, points_per_inch, max_speed);

            next.left_drive_velocity = left;
            next.right_drive_velocity = right;
            
            // prevent cusps
            if (Math.abs(next.heading - previous.heading) > Math.max(2 * Math.abs(previous.curvature) * displacement, 0.01))
            {
                return new Trajectory([{
                    position: new Vec2(Infinity, Infinity),
                    curvature: Infinity,

                    left_drive_velocity: Infinity,
                    right_drive_velocity: Infinity,

                    heading: Infinity,

                    time: Infinity,
                }]);
            }
        }

        for (let i = trajectory.length - 1 - 1; i > 0; i--)
        {
            const previous = trajectory[i+1];
            const next = trajectory[i];

            const max_speed = path.get_max_speed(next.position);

            const [left, right] = TrajectoryBuilder.compute_velocity_max(this.constraints, previous, next, points_per_inch, max_speed);

            next.left_drive_velocity = left;
            next.right_drive_velocity = right;
        }

        for (let i = 1; i < trajectory.length; i++)
        {
            const previous = trajectory[i-1];
            const next = trajectory[i];

            const next_left_velocity_sqr = next.left_drive_velocity * next.left_drive_velocity;
            const last_left_velocity_sqr = previous.left_drive_velocity * previous.left_drive_velocity;

            const left_accel_exceeds = next_left_velocity_sqr > last_left_velocity_sqr + 2 * this.constraints.max_wheel_accel * displacement;

            const next_right_velocity_sqr = next.right_drive_velocity * next.right_drive_velocity;
            const last_right_velocity_sqr = previous.right_drive_velocity * previous.right_drive_velocity;

            const right_accel_exceeds = next_right_velocity_sqr > last_right_velocity_sqr + 2 * this.constraints.max_wheel_accel * displacement;

            if (left_accel_exceeds || right_accel_exceeds)
            {
                const max_speed = path.get_max_speed(next.position);

                const [left, right] = TrajectoryBuilder.compute_velocity_max(this.constraints, previous, next, points_per_inch, max_speed);

                next.left_drive_velocity = left;
                next.right_drive_velocity = right;
            }

            const previous_velocity = (previous.left_drive_velocity + previous.right_drive_velocity) / 2;
            const next_velocity = (next.left_drive_velocity + next.right_drive_velocity) / 2;
            const avg_velocity = (previous_velocity + next_velocity) / 2;

            next.time = previous.time + displacement / Math.abs(avg_velocity);
        }
        
        return new Trajectory(trajectory);
    }
}