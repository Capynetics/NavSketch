import type { ParametersState } from "../../types/parameters";

export class Lidar {
  senseEnvironment(state: ParametersState): number[] {
    const lidar = state.lidar;
    if (!lidar.enabled) {
      return [];
    }

    const beamCount = Math.max(1, Math.floor(lidar.resolution));
    const maxRange = Math.max(0, lidar.range);
    const fovRad = (lidar.fieldOfView * Math.PI) / 180;

    const rx = state.robot.currentPose.x;
    const ry = state.robot.currentPose.y;
    const heading = state.robot.currentPose.theta;

    const startAngle = heading - fovRad / 2;
    const isFullCircle = Math.abs(fovRad) >= 2 * Math.PI;
    const step = beamCount > 1 ? fovRad / (isFullCircle ? beamCount : beamCount - 1) : 0;

    const ranges: number[] = [];
    for (let i = 0; i < beamCount; i += 1) {
      const angle = startAngle + i * step;
      const dx = Math.cos(angle);
      const dy = Math.sin(angle);

      let closest = maxRange;
      for (const obstacle of state.obstacles) {
        let hit: number | null = null;

        if (obstacle.type === "circle") {
          const radius = obstacle.geometry.radius ?? 0;
          hit = this.rayCircleDistance(
            rx,
            ry,
            dx,
            dy,
            obstacle.pose.x,
            obstacle.pose.y,
            radius
          );
        } else if (obstacle.type === "rectangle") {
          const width = obstacle.geometry.width ?? 0;
          const height = obstacle.geometry.height ?? 0;
          hit = this.rayRectangleDistance(
            rx,
            ry,
            dx,
            dy,
            obstacle.pose.x,
            obstacle.pose.y,
            obstacle.pose.theta,
            width,
            height
          );
        }

        if (hit !== null && hit >= 0 && hit < closest) {
          closest = hit;
        }
      }

      ranges.push(closest);
    }

    return ranges;
  }

  private rayCircleDistance(
    ox: number,
    oy: number,
    dx: number,
    dy: number,
    cx: number,
    cy: number,
    radius: number
  ): number | null {
    if (radius <= 0) {
      return null;
    }

    const lx = ox - cx;
    const ly = oy - cy;
    const b = 2 * (dx * lx + dy * ly);
    const c = lx * lx + ly * ly - radius * radius;
    const disc = b * b - 4 * c;
    if (disc < 0) {
      return null;
    }

    const sqrtDisc = Math.sqrt(disc);
    const t1 = (-b - sqrtDisc) / 2;
    const t2 = (-b + sqrtDisc) / 2;

    if (t1 >= 0) {
      return t1;
    }
    if (t2 >= 0) {
      return t2;
    }
    return null;
  }

  private rayRectangleDistance(
    ox: number,
    oy: number,
    dx: number,
    dy: number,
    cx: number,
    cy: number,
    theta: number,
    width: number,
    height: number
  ): number | null {
    if (width <= 0 || height <= 0) {
      return null;
    }

    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);

    const relX = ox - cx;
    const relY = oy - cy;
    const localOx = cosT * relX + sinT * relY;
    const localOy = -sinT * relX + cosT * relY;
    const localDx = cosT * dx + sinT * dy;
    const localDy = -sinT * dx + cosT * dy;

    return this.rayAabbDistance(
      localOx,
      localOy,
      localDx,
      localDy,
      -width / 2,
      width / 2,
      -height / 2,
      height / 2
    );
  }

  private rayAabbDistance(
    ox: number,
    oy: number,
    dx: number,
    dy: number,
    minX: number,
    maxX: number,
    minY: number,
    maxY: number
  ): number | null {
    const eps = 1e-9;
    let tMin = -Infinity;
    let tMax = Infinity;

    if (Math.abs(dx) < eps) {
      if (ox < minX || ox > maxX) {
        return null;
      }
    } else {
      const tx1 = (minX - ox) / dx;
      const tx2 = (maxX - ox) / dx;
      tMin = Math.max(tMin, Math.min(tx1, tx2));
      tMax = Math.min(tMax, Math.max(tx1, tx2));
    }

    if (Math.abs(dy) < eps) {
      if (oy < minY || oy > maxY) {
        return null;
      }
    } else {
      const ty1 = (minY - oy) / dy;
      const ty2 = (maxY - oy) / dy;
      tMin = Math.max(tMin, Math.min(ty1, ty2));
      tMax = Math.min(tMax, Math.max(ty1, ty2));
    }

    if (tMax < tMin || tMax < 0) {
      return null;
    }

    return tMin >= 0 ? tMin : tMax;
  }
}