export class Vec2 {
    x: number;
    y: number;

    static from_polar(theta: number, radius: number) {
        return new Vec2(
            Math.cos(theta) * radius,
            Math.sin(theta) * radius
        )
    }

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }

    distance_to_squared(other: Vec2) {
        return (this.x - other.x) * (this.x - other.x) + (this.y - other.y) * (this.y - other.y);
    }

    distance_to(other: Vec2) {
        return Math.sqrt(this.distance_to_squared(other));
    }
}