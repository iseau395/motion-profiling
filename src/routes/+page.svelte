<script lang="ts">
    import AnimatedCanvas from "$lib/AnimatedCanvas.svelte";
    import { InputSystem } from "$lib/input";
    import { width, height } from "$lib/constants";
    import { TrajectoryBuilder } from "$lib/trajectory/trajectory_builder";
    import { Vec2 } from "$lib/trajectory/vec2";
    import { ControlLockMode, default_polar_optimizer_data, Path } from "$lib/trajectory/path";
    import { Robot } from "$lib/robot";

    let canvas: HTMLCanvasElement | undefined = $state(undefined);
    let input = $state(new InputSystem());

    const max_wheel_speed = (450 * Math.PI * 3.25) / 60;
    const max_wheel_accel = max_wheel_speed / .35;
    const constraints = {
        drivetrain_rpm: 450,
        wheel_diameter: 3.25,

        max_wheel_accel,

        track_width: 11,
        mass_lb: 16,

        friction_coefficient: .7,
    }

    const path = new Path();
    // path.append_control({
    //     position: new Vec2(72 - 24, 72 - 24),
    //     derivative: new Vec2(0, 72),
    //     derivative_2: new Vec2(0, 0),

    //     position_lock: {},
    //     derivative_lock: default_polar_optimizer_data(ControlLockMode.DirectionLocked, 120, 50),
    //     derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    // });
    // path.append_control({
    //     position: new Vec2(72, 72 + 24),
    //     derivative: new Vec2(0, 72),
    //     derivative_2: new Vec2(0, 0),

    //     position_lock: {},
    //     derivative_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 120, 20),
    //     derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    // });
    // path.append_control({
    //     position: new Vec2(72+48, 72 + 24),
    //     derivative: new Vec2(0, -72),
    //     derivative_2: new Vec2(0, 0),

    //     position_lock: {},
    //     derivative_lock: default_polar_optimizer_data(ControlLockMode.DirectionLocked, 120, 30),
    //     derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    // });

    path.append_control({
        position: new Vec2(144 - 24, 72),
        derivative: new Vec2(0, 40),
        derivative_2: new Vec2(0, 0),

        position_lock: {},
        derivative_lock: default_polar_optimizer_data(ControlLockMode.DirectionLocked, 120, 30),
        derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    });
    path.append_control({
        position: new Vec2(144 - 24, 144 - 24),
        derivative: new Vec2(-5, 10),
        derivative_2: new Vec2(-10, -10),

        position_lock: {},
        derivative_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 120),
        derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    });
    path.append_control({
        position: new Vec2(144 - 24, 144 - 12),
        derivative: new Vec2(20, 0),
        derivative_2: new Vec2(0, -80),

        position_lock: {},
        derivative_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 120),
        derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    });
    path.append_control({
        position: new Vec2(144 - 12, 144 - 24),
        derivative: new Vec2(0, -20),
        derivative_2: new Vec2(0, 0),

        position_lock: {},
        derivative_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 120),
        derivative_2_lock: default_polar_optimizer_data(ControlLockMode.Unlocked, 240, 0)
    });

    const trajectory_builder = new TrajectoryBuilder(constraints);

    let trajectory = trajectory_builder.build(path);
    console.log(trajectory.total_time());

    let is_optimizing = false;
    async function optimize() {
        is_optimizing = true;
        console.time();
        for (let i = 0; i < 2; i++)
        {
            await path.optimize(5, constraints, 3);
        }
        console.timeEnd();
        
        trajectory = trajectory_builder.build(path, 3);
        console.log(trajectory.total_time());
        console.log(trajectory.states.map(v => `${v.time}, ${v.left_drive_velocity}, ${v.right_drive_velocity}`).join("\n"));

        is_optimizing = false;
    }


    const robot = new Robot(trajectory.states[0].position.x, trajectory.states[0].position.y, trajectory.states[0].heading, 11);

    let start_time = Date.now() / 1000;
    let last_time = start_time;
    let v_left = 0
    let v_right = 0
    function render(ctx: CanvasRenderingContext2D)
    {
        const now = Date.now() / 1000;
        const delta_time = (now - last_time);
        last_time = now;

        ctx.fillStyle = "#909090";
        ctx.fillRect(0, 0, width, height);

        const left = Number(input.is_key_down("q")) - Number(input.is_key_down("a"));
        const right = Number(input.is_key_down("e")) - Number(input.is_key_down("d"));
        const current_trajectory_state = trajectory.get_state(now - start_time);
        
        if (!is_optimizing && current_trajectory_state)
        {
            robot.move(current_trajectory_state?.left_drive_velocity, current_trajectory_state?.right_drive_velocity, delta_time);
        }
        else
        {
            v_left += (left || -Math.sign(v_left)) * max_wheel_accel * delta_time;
            v_right += (right || -Math.sign(v_right)) * max_wheel_accel * delta_time;

            v_left = Math.max(Math.min(v_left, max_wheel_speed), -max_wheel_speed);
            v_right = Math.max(Math.min(v_right, max_wheel_speed), -max_wheel_speed);

            robot.move(v_left, v_right, delta_time);
        }

        trajectory.draw(ctx, !is_optimizing ? max_wheel_speed : undefined);
        path.draw(ctx, is_optimizing ? max_wheel_speed : undefined);
        robot.draw(ctx);
    }

    function reset()
    {
        start_time = Date.now() / 1000;
        robot.x = trajectory.states[0].position.x;
        robot.y = trajectory.states[0].position.y;
        robot.theta = trajectory.states[0].heading;
    }
</script>

<AnimatedCanvas {render} bind:input bind:canvas {width} {height} scale={5} /><br/>
<button onclick={optimize}>re-optimize</button>
<button onclick={reset}>re-drive</button>