import type p5 from "p5";
import type { ParametersState } from "../types/parameters";
import { Lidar } from "./sensors/lidar";


export class Simulation {
  current_state: ParametersState;
  sensor_ranges: number[];
  ux: number;
  uy: number;
  p: p5;
  private lidar: Lidar;

  constructor(current_state: ParametersState, p: p5) {
    this.current_state = current_state;
    this.sensor_ranges = [];
    this.ux = 0;
    this.uy = 0;
    this.p = p;
    this.lidar = new Lidar();
  }

  updateState(nextState: ParametersState) {
    this.current_state = nextState;
  }

  move_towards_goal() {
        let dx = this.current_state.goal.x - this.current_state.robot.currentPose.x;
        let dy = this.current_state.goal.y - this.current_state.robot.currentPose.y;
        let distance = Math.sqrt(dx * dx + dy * dy);
      if (distance <= 1e-9) {
        this.ux = 0;
        this.uy = 0;
        return;
      }
        this.ux = dx / distance;
        this.uy = dy / distance;
  }

  follow_wall() {
    let ranges = this.sensor_ranges;
    let minRange = Math.min(...ranges);
    let index = ranges.indexOf(minRange);
    console.log(`Closest obstacle at index ${index} with range ${minRange}`);
    let angleToObstacle = (index / ranges.length) * 2 * Math.PI;
    console.log(`Angle to closest obstacle: ${angleToObstacle} radians`);
    // Correct the angle to be relative to the robot's heading and world coordinates
    angleToObstacle = angleToObstacle + Math.PI + this.current_state.robot.currentPose.theta + 0.1;

    let dx = Math.cos(angleToObstacle + Math.PI / 2);
    let dy = Math.sin(angleToObstacle + Math.PI / 2);
    let distance = Math.sqrt(dx * dx + dy * dy);
    this.ux = dx / distance;
    this.uy = dy / distance;
    this.p.stroke(255, 0, 0);
    this.p.strokeWeight(10);
    this.p.line(
      this.current_state.robot.currentPose.x * 100,
      this.current_state.robot.currentPose.y * 100,
      this.current_state.robot.currentPose.x * 100 + Math.cos(angleToObstacle) * 100,
      this.current_state.robot.currentPose.y * 100 + Math.sin(angleToObstacle) * 100,
    );
  }

  path_to_goal_is_clear(): boolean {
    const ranges = this.sensor_ranges;
    if (ranges.length === 0) {
      return true;
    }

    const startIndex = Math.max(0, Math.floor(ranges.length * 0.25));
    const endIndex = Math.max(startIndex + 1, Math.floor(ranges.length * 0.75));
    const frontRanges = ranges.slice(startIndex, endIndex);
    const minFrontRange = Math.min(...frontRanges);

    return minFrontRange >= this.current_state.robot.radius + 0.25;
  }

  sensor_read() {
    this.sensor_ranges = this.lidar.senseEnvironment(this.current_state);

    return this.sensor_ranges;
  }

  calculate_next_step(p: p5) {
    this.p = p;
    switch (this.current_state.planner.algorithm) {
      case "bug0":
        if (this.path_to_goal_is_clear()) {
          this.move_towards_goal();
        } else {
          this.follow_wall();
        }
        break;
      case "astar":
        this.ux = 0;
        this.uy = 0;
        break;
      case "dijkstra":
        this.ux = 0;
        this.uy = 0;
        break;
      default:
        this.ux = 0;
        this.uy = 0;
        break;
    }
  }

  move_to_next_step() {
    const dx = this.current_state.goal.x - this.current_state.robot.currentPose.x;
    const dy = this.current_state.goal.y - this.current_state.robot.currentPose.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    const speed = 2; // m/s

    const dt = this.current_state.simulation.timestep;
    if (distance < speed * dt) {
      this.current_state.robot.currentPose.x = this.current_state.goal.x;
      this.current_state.robot.currentPose.y = this.current_state.goal.y;
      if (this.ux !== 0 || this.uy !== 0) {
        this.current_state.robot.currentPose.theta = Math.atan2(this.uy, this.ux);
      }
      this.current_state.simulation.running = false;
    }else{
      this.current_state.robot.currentPose.x += this.ux * speed * dt;
      this.current_state.robot.currentPose.y += this.uy * speed * dt;
      if (this.ux !== 0 || this.uy !== 0) {
        this.current_state.robot.currentPose.theta = Math.atan2(this.uy, this.ux);
      }
    }
  }
};
