export class Robot
{
    constructor(public x: number, public y: number, public theta: number, public readonly track_width: number) {};
    
    move(velocity_left: number, velocity_right: number, delta_time: number)
    {
        let local_delta_x: number;
        let local_delta_y: number;
        let delta_theta: number;

        if (velocity_left != velocity_right)
        {
            const rotational_velocity = (velocity_left - velocity_right) / this.track_width;

            let curve_radius = (velocity_left + velocity_right) / (2 * rotational_velocity);
            if (!isFinite(curve_radius) || isNaN(curve_radius))
                curve_radius = 10e6;

            delta_theta = rotational_velocity * delta_time;

            local_delta_x = curve_radius - curve_radius * Math.cos(delta_theta);
            local_delta_y = curve_radius * Math.sin(delta_theta);
        }
        else
        {
            delta_theta = 0;
            
            local_delta_x = 0;
            local_delta_y = velocity_left * delta_time;
        }


        const global_delta_x = local_delta_x * Math.cos(this.theta + Math.PI / 2) + local_delta_y * Math.cos(this.theta);
        const global_delta_y = local_delta_x * Math.sin(this.theta + Math.PI / 2) + local_delta_y * Math.sin(this.theta);

        this.x += global_delta_x;
        this.y += global_delta_y;
        this.theta += delta_theta;
    }

    draw(ctx: CanvasRenderingContext2D)
    {
        const radius = this.track_width * Math.SQRT1_2;

        ctx.beginPath();
        ctx.moveTo(this.x + radius * Math.cos(this.theta + 1*Math.PI/4), this.y + radius * Math.sin(this.theta + 1*Math.PI/4));
        ctx.lineTo(this.x + radius * Math.cos(this.theta + 3*Math.PI/4), this.y + radius * Math.sin(this.theta + 3*Math.PI/4));
        ctx.lineTo(this.x + radius * Math.cos(this.theta + 5*Math.PI/4), this.y + radius * Math.sin(this.theta + 5*Math.PI/4));
        ctx.lineTo(this.x + radius * Math.cos(this.theta + 7*Math.PI/4), this.y + radius * Math.sin(this.theta + 7*Math.PI/4));
        ctx.closePath();

        ctx.fillStyle = "rgba(66, 66, 66, 0.5)";
        ctx.fill();
    }
}