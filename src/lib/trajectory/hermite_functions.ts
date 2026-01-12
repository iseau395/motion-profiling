import { Vec2 } from "./vec2";

export interface HermiteWaypoint {
    position: Vec2,
    derivative: Vec2,
    derivative_2: Vec2,
}

// hermite functions

function h0(t: number)
{
    return 1 - 10 * (t * t * t) + 15 * (t * t * t * t) - 6 * (t * t * t * t * t);
}

function h1(t: number)
{
    return t - 6 * (t * t * t) + 8 * (t * t * t * t) - 3 * (t * t * t * t * t);
}

function h2(t: number)
{
    return 0.5 * (t * t) - 1.5 * (t * t * t) + 1.5 * (t * t * t * t) - 0.5 * (t * t * t * t * t);
}

function h3(t: number)
{
    return 0.5 * (t * t * t) - (t * t * t * t) + 0.5 * (t * t * t * t * t);
}

function h4(t: number)
{
    return -4 * (t * t * t) + 7 * (t * t * t * t) - 3 * (t * t * t * t * t);
}

function h5(t: number)
{
    return 10 * (t * t * t) - 15 * (t * t * t * t) + 6 * (t * t * t * t * t);
}

// derivative hermite functions

function h0_prime(t: number)
{
    return -30 * (t * t) + 60 * (t * t * t) - 30 * (t * t * t * t);
}

function h1_prime(t: number)
{
    return 1 - 18 * (t * t) + 32 * (t * t * t) - 15 * (t * t * t * t);
}

function h2_prime(t: number)
{
    return t - 4.5 * (t * t) + 6 * (t * t * t) - 2.5 * (t * t * t * t);
}

function h3_prime(t: number)
{
    return 1.5 * (t * t) - 4 * (t * t * t) + 2.5 * (t * t * t * t);
}

function h4_prime(t: number)
{
    return -12 * (t * t) + 28 * (t * t * t) - 15 * (t * t * t * t);
}

function h5_prime(t: number)
{
    return 30 * (t * t) - 60 * (t * t * t) + 30 * (t * t * t * t);
}

// double derivative hermite functions

function h0_double_prime(t: number)
{
    return -60 * t + 180 * (t * t) - 120 * (t * t * t);
}

function h1_double_prime(t: number)
{
    return -36 * t + 96 * (t * t) - 60 * (t * t * t);
}

function h2_double_prime(t: number)
{
    return 1 - 9 * t + 18 * (t * t) - 10 * (t * t * t);
}

function h3_double_prime(t: number)
{
    return 3 * t - 12 * (t * t) + 10 * (t * t * t);
}

function h4_double_prime(t: number)
{
    return -24 * t + 84 * (t * t) - 60 * (t * t * t);
}

function h5_double_prime(t: number)
{
    return 60 * t - 180 * (t * t) + 120 * (t * t * t);
}

export function lerp_hermite(t: number, initial_waypoint: HermiteWaypoint, final_waypoint: HermiteWaypoint) {
    return new Vec2(
        initial_waypoint.position.x * h0(t) +
        initial_waypoint.derivative.x * h1(t) +
        initial_waypoint.derivative_2.x * h2(t) +
        final_waypoint.derivative_2.x * h3(t) +
        final_waypoint.derivative.x * h4(t) +
        final_waypoint.position.x * h5(t),

        initial_waypoint.position.y * h0(t) +
        initial_waypoint.derivative.y * h1(t) +
        initial_waypoint.derivative_2.y * h2(t) +
        final_waypoint.derivative_2.y * h3(t) +
        final_waypoint.derivative.y * h4(t) +
        final_waypoint.position.y * h5(t)
    );
}

export function lerp_hermite_prime(t: number, initial_waypoint: HermiteWaypoint, final_waypoint: HermiteWaypoint) {
    return new Vec2(
        initial_waypoint.position.x * h0_prime(t) +
        initial_waypoint.derivative.x * h1_prime(t) +
        initial_waypoint.derivative_2.x * h2_prime(t) +
        final_waypoint.derivative_2.x * h3_prime(t) +
        final_waypoint.derivative.x * h4_prime(t) +
        final_waypoint.position.x * h5_prime(t),

        initial_waypoint.position.y * h0_prime(t) +
        initial_waypoint.derivative.y * h1_prime(t) +
        initial_waypoint.derivative_2.y * h2_prime(t) +
        final_waypoint.derivative_2.y * h3_prime(t) +
        final_waypoint.derivative.y * h4_prime(t) +
        final_waypoint.position.y * h5_prime(t)
    );
}

export function lerp_hermite_double_prime(t: number, initial_waypoint: HermiteWaypoint, final_waypoint: HermiteWaypoint) {
    return new Vec2(
        initial_waypoint.position.x * h0_double_prime(t) +
        initial_waypoint.derivative.x * h1_double_prime(t) +
        initial_waypoint.derivative_2.x * h2_double_prime(t) +
        final_waypoint.derivative_2.x * h3_double_prime(t) +
        final_waypoint.derivative.x * h4_double_prime(t) +
        final_waypoint.position.x * h5_double_prime(t),

        initial_waypoint.position.y * h0_double_prime(t) +
        initial_waypoint.derivative.y * h1_double_prime(t) +
        initial_waypoint.derivative_2.y * h2_double_prime(t) +
        final_waypoint.derivative_2.y * h3_double_prime(t) +
        final_waypoint.derivative.y * h4_double_prime(t) +
        final_waypoint.position.y * h5_double_prime(t)
    );
}