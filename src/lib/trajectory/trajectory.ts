import { Vec2 } from "./vec2";

export interface TrajectoryPose {
    position: Vec2,
    curvature: number,

    left_drive_velocity: number,
    right_drive_velocity: number,

    heading: number,

    time: number,
}

export class Trajectory {
    readonly states: TrajectoryPose[];

    private lerp(a: number, b: number, t: number)
    {
        return a + t * (b - a);
    }

    constructor(poses?: TrajectoryPose[]) {
        this.states = poses ?? [];
    }

    get_state(time: number): TrajectoryPose | undefined
    {
        for (const [state, last_state] of this.states.map((v, i, l) => [v, l[i-1]]))
        {
            if (state.time > time)
            {
                const t = (time - last_state.time) / (state.time - last_state.time);

                return {
                    position: new Vec2(
                        this.lerp(last_state.position.x, state.position.x, t),
                        this.lerp(last_state.position.y, state.position.y, t)
                    ),
                    curvature: this.lerp(last_state.curvature, state.curvature, t),

                    left_drive_velocity: this.lerp(last_state.left_drive_velocity, state.left_drive_velocity, t),
                    right_drive_velocity: this.lerp(last_state.right_drive_velocity, state.right_drive_velocity, t),

                    heading: this.lerp(last_state.heading, state.heading, t),

                    time
                }
            }
        }

        return undefined;
    }

    total_time()
    {
        return this.states.at(-1)?.time!;
    }

    draw(ctx: CanvasRenderingContext2D, max_speed?: number) {
        ctx.lineWidth = 0.5;

        for (let i = 0; i < this.states.length - 1; i++) {
            const current = this.states[i];
            const next = this.states[i + 1];

            ctx.beginPath();
            ctx.moveTo(current.position.x, current.position.y);
            ctx.lineTo(next.position.x, next.position.y);

            if (max_speed)
            {
                const current_velocity = (current.left_drive_velocity + current.right_drive_velocity) / 2;
                const next_velocity = (next.left_drive_velocity + next.right_drive_velocity) / 2;
                const avg_velocity = (current_velocity + next_velocity) / 2;
                ctx.strokeStyle = `hsl(${(avg_velocity / max_speed) * 120}, ${100}%, ${50}%)`;
                
                if (isNaN(avg_velocity) || !avg_velocity) {
                    ctx.strokeStyle = "black";
                }
            }
            else
            {
                ctx.strokeStyle = "#555555";
            }

            ctx.stroke();
        }
    }
}