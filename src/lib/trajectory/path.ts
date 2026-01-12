import { delay } from "$lib";
import { lerp_hermite, lerp_hermite_double_prime, lerp_hermite_prime, type HermiteWaypoint } from "./hermite_functions";
import { layered_optimize, type OptimizablePolarVec2Data, type OptimizableVariableData, type OptimizableVec2Data } from "./optimizer";
import type { Trajectory, TrajectoryPose } from "./trajectory";
import { TrajectoryBuilder, type TrajectoryConstraints } from "./trajectory_builder";
import { Vec2 } from "./vec2";

export type SpeedCallback = (pos: Vec2) => number;

export interface HermiteControl extends HermiteWaypoint
{
    position_lock: OptimizableVec2Data,
    derivative_lock: OptimizablePolarVec2Data,
    derivative_2_lock: OptimizablePolarVec2Data,
}

export enum ControlLockMode
{
    Unlocked,
    DirectionLocked,
    Locked
}

export function default_polar_optimizer_data(mode: ControlLockMode, max_magnitude = 120, min_magnitude = 10): OptimizablePolarVec2Data
{
    const data: OptimizablePolarVec2Data = {};

    if (mode == ControlLockMode.Unlocked || mode == ControlLockMode.DirectionLocked)
    {
        data.magnitude = {
            range_min: min_magnitude,
            range_max: max_magnitude,
            tolerance: 5,
            per_layer: 6
        }
    }

    if (mode == ControlLockMode.Unlocked)
    {
        data.direction = {
            range_min: 0,
            range_max: Math.PI * 2,
            tolerance: Math.PI / 64,
            per_layer: 8
        }
    }


    return data;
}

function fill_circle(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, style: string)
{
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.arc(x, y, r, 0, 2 * Math.PI);

    ctx.fillStyle = style;
    ctx.fill();
}

export interface PathInstant
{
    position: Vec2,
    curvature: number,
    heading: number,
}

export class Path
{
    private readonly spline = new Array<HermiteControl>();
    private readonly speed_callbacks = new Array<SpeedCallback>();

    private static calculate_hermite(points_per_inch: number, initial_waypoint: HermiteWaypoint, final_waypoint: HermiteWaypoint, prefill_velocity: true | false = false): TrajectoryPose[] | PathInstant[]
    {
        const states = new Array<TrajectoryPose | PathInstant>();

        const inches_per_point = 1 / points_per_inch;

        let t = 0
        let dt = 0;
        while (t < 1) {
            t = Math.min(1, t + dt);

            const next = lerp_hermite(t, initial_waypoint, final_waypoint);
            const next_prime = lerp_hermite_prime(t, initial_waypoint, final_waypoint);
            const next_double_prime = lerp_hermite_double_prime(t, initial_waypoint, final_waypoint);

            const prime_magnitude = Math.sqrt(next_prime.x * next_prime.x + next_prime.y * next_prime.y);

            const curvature = (next_prime.x * next_double_prime.y - next_double_prime.x * next_prime.y)
                                /
                                (prime_magnitude * prime_magnitude * prime_magnitude);

            if (prefill_velocity == true)
            {
                states.push({
                    position: next,
                    curvature: curvature,
                    
                    heading: Math.atan2(next_prime.y, next_prime.x),

                    left_drive_velocity: Infinity,
                    right_drive_velocity: Infinity,

                    time: Infinity,
                });
            }
            else
            { 
                states.push({
                    position: next,
                    curvature: curvature,
                    
                    heading: Math.atan2(next_prime.y, next_prime.x)
                });
            }
                          
            dt = inches_per_point / prime_magnitude;
        }

        return states;
    }

    get_discretized(points_per_inch: number, prefill_velocity: boolean): TrajectoryPose[] | PathInstant[]
    {
        const path = new Array<TrajectoryPose | PathInstant>();
        
        for (let i = 0; i < this.spline.length-1; i++) {
            path.pop();

            const segment = Path.calculate_hermite(points_per_inch, this.spline[i], this.spline[i+1], prefill_velocity);

            path.push(...segment);
        }

        return path;
    }

    get_max_speed(position: Vec2)
    {
        let min = Infinity;

        for (const callback of this.speed_callbacks)
        {
            min = Math.min(min ,callback(position));
        }

        return min;
    }
        
    append_control(...control: HermiteControl[])
    {
        this.spline.push(...control);
    }

    append_speed_callback(...callback: SpeedCallback[])
    {
        this.speed_callbacks.push(...callback);
    }

    private last_trajectory: Trajectory | undefined;
    async optimize(layers: number, constraints: TrajectoryConstraints, points_per_inch = 3)
    {
        function default_data(x: number): OptimizableVariableData
        {
            return {
                range_min: x,
                range_max: x,
                per_layer: 1,
                tolerance: Infinity,
            }
        }

        for (const [i, control_set] of this.spline.entries())
        {
            const trajectory_builder = new TrajectoryBuilder(constraints);
            
            const path = new Path();
            if (i-1 >= 0)
            {
                path.append_control(this.spline[i-1]);
            }
            path.append_control(control_set);
            if (i+1 <= this.spline.length-1)
            {
                path.append_control(this.spline[i+1]);
            }
            path.append_speed_callback(...this.speed_callbacks);

            const loss_function = async (position: Vec2, derivative: Vec2, derivative_2: Vec2) =>
            {
                this.spline[i].position = position;
                this.spline[i].derivative = derivative;
                this.spline[i].derivative_2 = derivative_2;

                this.last_trajectory = trajectory_builder.build(path, points_per_inch);

                const error = this.last_trajectory.total_time();

                return !isNaN(error) ? error : Infinity;
            }

            const initial_derivative_direction = Math.atan2(control_set.derivative.y, control_set.derivative.x);
            const initial_derivative_magnitude = new Vec2(0, 0).distance_to(control_set.derivative);
            const initial_derivative_2_direction = Math.atan2(control_set.derivative_2.y, control_set.derivative_2.x);
            const initial_derivative_2_magnitude = new Vec2(0, 0).distance_to(control_set.derivative_2);

            const variable_data = {
                position_x: {...(control_set.position_lock.x ?? default_data(control_set.position.x))},
                position_y: {...(control_set.position_lock.y ?? default_data(control_set.position.y))},

                derivative_dir: {...(control_set.derivative_lock.direction ?? default_data(initial_derivative_direction))},
                derivative_mag: {...(control_set.derivative_lock.magnitude ?? default_data(initial_derivative_magnitude))},

                derivative_2_dir: {...(control_set.derivative_2_lock.direction ?? default_data(initial_derivative_2_direction))},
                derivative_2_mag: {...(control_set.derivative_2_lock.magnitude ?? default_data(initial_derivative_2_magnitude))},
            }

            const optimal = await layered_optimize(layers, async (v: { [K in keyof typeof variable_data]: number }) => {
                return await loss_function(
                    new Vec2(v.position_x, v.position_y),
                    Vec2.from_polar(v.derivative_dir, v.derivative_mag),
                    Vec2.from_polar(v.derivative_2_dir, v.derivative_2_mag)
                );
            }, variable_data);

            
            this.spline[i].position = new Vec2(optimal.position_x, optimal.position_y);
            this.spline[i].derivative = Vec2.from_polar(optimal.derivative_dir, optimal.derivative_mag);
            this.spline[i].derivative_2 = Vec2.from_polar(optimal.derivative_2_dir, optimal.derivative_2_mag);
            
            // await delay(0);
        }
    }
    

    draw(ctx: CanvasRenderingContext2D, max_speed?: number) {
        if (this.last_trajectory && max_speed)
            this.last_trajectory.draw(ctx, max_speed);

        for (const handle_group of this.spline)
        {
            const blue = "100, 100, 255";
            const red = "255, 100, 100";
            const purple = "200, 100, 255";

            ctx.beginPath();
            ctx.moveTo(handle_group.position.x, handle_group.position.y);
            ctx.lineTo(handle_group.position.x + handle_group.derivative.x, handle_group.position.y + handle_group.derivative.y);
            ctx.strokeStyle = `rgba(${red}, 0.5)`;
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(handle_group.position.x, handle_group.position.y);
            ctx.lineTo(handle_group.position.x + handle_group.derivative_2.x, handle_group.position.y + handle_group.derivative_2.y);
            ctx.strokeStyle = `rgba(${purple}, 0.5)`;
            ctx.stroke();

            fill_circle(ctx, handle_group.position.x, handle_group.position.y, 1.5, `rgba(${blue}, 0.5)`);
            fill_circle(ctx, handle_group.position.x, handle_group.position.y, .75, `rgba(${blue}, 1)`);

            fill_circle(ctx, handle_group.position.x + handle_group.derivative.x, handle_group.position.y + handle_group.derivative.y, 1.5, `rgba(${red}, 0.5)`);
            fill_circle(ctx, handle_group.position.x + handle_group.derivative.x, handle_group.position.y + handle_group.derivative.y, .75, `rgba(${red}, 1)`);

            fill_circle(ctx, handle_group.position.x + handle_group.derivative_2.x, handle_group.position.y + handle_group.derivative_2.y, 1.5, `rgba(${purple}, 0.5)`);
            fill_circle(ctx, handle_group.position.x + handle_group.derivative_2.x, handle_group.position.y + handle_group.derivative_2.y, .75, `rgba(${purple}, 1)`);
        }
    }
}