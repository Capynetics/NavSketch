import type p5 from "p5";
import type { ParametersState } from "../types/parameters";
import { Lidar } from "./sensors/lidar";


export class Simulation {
  current_state: ParametersState;
  sensor_ranges: number[];
  ux: number;
  uy: number;
  utheta: number;
  p: p5;
  private lidar: Lidar;
  private obstacle_encountered: boolean;
  private entrance_x: number;
  private entrance_y: number;
  private min_distance_to_goal: number;
  private left_entrance: boolean;
  private go_to_min_distance: boolean;
  is_following_wall: boolean;
  is_bug1_following_wall: boolean;
  follow_direction: "left" | "right";
  min_distance_to_goal_point: { x: number; y: number };
  m_line_x: number;
  m_line_y: number;
  private bug2_started: boolean;

  get is_bug2_started(): boolean {
    return this.bug2_started;
  }

  constructor(current_state: ParametersState, p: p5) {
    this.current_state = current_state;
    this.sensor_ranges = [];
    this.ux = 0;
    this.uy = 0;
    this.utheta = 0;
    this.p = p;
    this.lidar = new Lidar();
    this.obstacle_encountered = false;
    this.entrance_x = 0;
    this.entrance_y = 0;
    this.min_distance_to_goal = Infinity;
    this.left_entrance = false;
    this.go_to_min_distance = false;
    this.is_following_wall = false;
    this.is_bug1_following_wall = false;
    this.min_distance_to_goal_point = { x: 0, y: 0 };
    this.m_line_x = 0; //starting point of the m-line
    this.m_line_y = 0; //starting point of the m-line
    this.bug2_started = false; // Flag to indicate if Bug2 has started
    this.follow_direction = "right"; // Default follow direction for Bug2
    
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
        this.is_following_wall = false;
        this.is_bug1_following_wall = false;
  }

  move_towards_x_y(x: number, y: number) {
    let dx = x - this.current_state.robot.currentPose.x;
    let dy = y - this.current_state.robot.currentPose.y;
    let distance = Math.sqrt(dx * dx + dy * dy);
    if (distance <= 1e-9) {
      this.ux = 0;
      this.uy = 0;
      return;
    }
    this.ux = dx / distance;
    this.uy = dy / distance;
  }

  follow_wall(direction: "left" | "right" = "right") {
    this.is_following_wall = true;
    let ranges = this.sensor_ranges;
    let minRange = Math.min(...ranges);
    let index = ranges.indexOf(minRange);
    let angleToObstacle = (index / ranges.length) * 2 * Math.PI;
    // Correct the angle to be relative to the robot's heading and world coordinates
    // The nudge sign is mirrored for "left" since it steers relative to the opposite turn direction
    const nudge = direction === "left" ? -0.05 : 0.05;
    if (minRange < this.current_state.robot.radius + 0.05) {
      angleToObstacle = angleToObstacle + Math.PI + this.current_state.robot.currentPose.theta + nudge;
    }else {
      angleToObstacle = angleToObstacle + Math.PI + this.current_state.robot.currentPose.theta - nudge;
    } 

    const turn = direction === "left" ? -Math.PI / 2 : Math.PI / 2;
    let dx = Math.cos(angleToObstacle + turn);
    let dy = Math.sin(angleToObstacle + turn);
    let distance = Math.sqrt(dx * dx + dy * dy);
    this.ux = dx / distance;
    this.uy = dy / distance;
    this.p.stroke(255, 0, 0);
    this.p.strokeWeight(10);
    //this.p.line(
    //  this.current_state.robot.currentPose.x * 100,
    //  this.current_state.robot.currentPose.y * 100,
    //  this.current_state.robot.currentPose.x * 100 + Math.cos(angleToObstacle) * 100,
    //  this.current_state.robot.currentPose.y * 100 + Math.sin(angleToObstacle) * 100,
    //);
  }

  path_to_goal_is_clear(): boolean {
    const ranges = this.sensor_ranges;
    if (ranges.length === 0) {
      return true;
    }

    const startIndex = Math.max(0, Math.floor(ranges.length * 0));
    const endIndex = Math.max(startIndex + 1, Math.floor(ranges.length * 1));
    const frontRanges = ranges.slice(startIndex, endIndex);
    const minFrontRange = Math.min(...frontRanges);

    return minFrontRange >= this.current_state.robot.radius + 0.05; // Add a small buffer to avoid collisions
  }

  sensor_read() {
    this.sensor_ranges = this.lidar.senseEnvironment(this.current_state);

    return this.sensor_ranges;
  } 

  get_discontinuities(ranges: number[]): number[] {
    const candidates: { index: number; strength: number }[] = [];
    const maximumSensorRange = this.current_state.lidar.range;
    const absoluteThreshold = 0.05;
    const relativeThreshold = 0.2;

    const isOutOfRange = (range: number) => range >= maximumSensorRange - 1e-9;

    for (let index = 0; index < ranges.length - 1; index += 1) {
      const currentRange = ranges[index];
      const nextRange = ranges[index + 1];
      const currentIsOutOfRange = isOutOfRange(currentRange);
      const nextIsOutOfRange = isOutOfRange(nextRange);

      // A finite-to-infinite transition is a real sensor discontinuity.
      if (currentIsOutOfRange !== nextIsOutOfRange) {
        const finiteRange = currentIsOutOfRange ? nextRange : currentRange;
        candidates.push({
          index,
          strength: maximumSensorRange - finiteRange,
        });
        continue;
      }

      if (currentIsOutOfRange) {
        continue;
      }

      const distanceJump = Math.abs(currentRange - nextRange);
      const threshold = Math.max(
        absoluteThreshold,
        relativeThreshold * Math.min(currentRange, nextRange)
      );

      if (distanceJump > threshold) {
        candidates.push({ index, strength: distanceJump });
      }
    }

    // Keep the strongest detection from each run of neighboring jumps.
    const discontinuityIndices: number[] = [];
    for (const candidate of candidates) {
      const previousIndex = discontinuityIndices[discontinuityIndices.length - 1];
      const previousCandidate = candidates.find(({ index }) => index === previousIndex);

      if (previousIndex !== undefined && candidate.index === previousIndex + 1) {
        if (previousCandidate && candidate.strength > previousCandidate.strength) {
          discontinuityIndices[discontinuityIndices.length - 1] = candidate.index;
        }
      } else {
        discontinuityIndices.push(candidate.index);
      }
    }

    return discontinuityIndices;
  }

  get_discontinuity_lines(){
    const discontinuity_lines: { start: number; end: number }[] = [];
    const discontinuities = this.get_discontinuities(this.sensor_ranges);
    const ranges = this.sensor_ranges;

    if (discontinuities.length === 0) {
      return discontinuity_lines;
    }

    for (let i = 0; i < discontinuities.length; i++) {
      const nextIndex = discontinuities[i] + 1;
      if (nextIndex >= ranges.length) {
        continue;
      }
      if (ranges[discontinuities[i]] > ranges[nextIndex]) {
        if(discontinuities[i + 1] !== undefined) {
          discontinuity_lines.push({ start: discontinuities[i], end: discontinuities[i + 1]});
        }else {
          discontinuity_lines.push({ start: discontinuities[i], end: discontinuities[0]});
        }
      }
    }

    for(let i = 0; i < discontinuity_lines.length; i++) {

      if (discontinuity_lines[i].start < discontinuity_lines[i].end) {
        if (ranges[Math.floor((discontinuity_lines[i].start + discontinuity_lines[i].end) / 2)] == this.current_state.lidar.range) {
          //remove discontinuity_lines[i] from the array
          discontinuity_lines.splice(i, 1);
          i--;
        }
      }else {
        if (ranges[Math.floor((discontinuity_lines[i].start + discontinuity_lines[i].end + ranges.length) / 2) % ranges.length] == this.current_state.lidar.range) {
          //remove discontinuity_lines[i] from the array
          discontinuity_lines.splice(i, 1);
          i--;
        }
      }
    }

    return discontinuity_lines;
  }

  left_or_right(discontinuityIndex: number): "left" | "right" {
    const robotPose = this.current_state.robot.currentPose;
    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const startAngle = robotPose.theta - fieldOfView / 2;
    const beamCount = this.sensor_ranges.length;
    const isFullCircle = Math.abs(fieldOfView) >= 2 * Math.PI;
    const angleStep = beamCount > 1
      ? fieldOfView / (isFullCircle ? beamCount : beamCount - 1)
      : 0;
    const angle = startAngle + discontinuityIndex * angleStep;
    const goalAngle = Math.atan2(
      this.current_state.goal.y - robotPose.y,
      this.current_state.goal.x - robotPose.x
    );

    const relativeAngle = Math.atan2(
      Math.sin(angle - goalAngle),
      Math.cos(angle - goalAngle)
    );

    return relativeAngle <= 0 ? "left" : "right";
  }

  calculate_discontinuity_heuristic(discontinuityIndex: number): number {
    const nextIndex = discontinuityIndex + 1;
    if (
      discontinuityIndex < 0 ||
      nextIndex >= this.sensor_ranges.length
    ) {
      return Infinity;
    }

    const currentRange = this.sensor_ranges[discontinuityIndex];
    const nextRange = this.sensor_ranges[nextIndex];
    const discontinuityRange = Math.min(currentRange, nextRange);
    if (!Number.isFinite(discontinuityRange)) {
      return Infinity;
    }

    const robotPose = this.current_state.robot.currentPose;
    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const startAngle = robotPose.theta - fieldOfView / 2;
    const beamCount = this.sensor_ranges.length;
    const angleStep = beamCount > 1 ? fieldOfView / (beamCount - 1) : 0;
    const angle = startAngle + discontinuityIndex * angleStep;

    const discontinuityX = robotPose.x + discontinuityRange * Math.cos(angle);
    const discontinuityY = robotPose.y + discontinuityRange * Math.sin(angle);

    const distanceToDiscontinuity = Math.hypot(
      discontinuityX - robotPose.x,
      discontinuityY - robotPose.y
    );
    const distanceToGoal = Math.hypot(
      this.current_state.goal.x - discontinuityX,
      this.current_state.goal.y - discontinuityY
    );

    return distanceToDiscontinuity + distanceToGoal;
  }

  private drawDiscontinuities() {
    const discontinuities = this.get_discontinuities(this.sensor_ranges);
    const beamCount = this.sensor_ranges.length;
    if (beamCount === 0) {
      return;
    }

    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const startAngle = this.current_state.robot.currentPose.theta - fieldOfView / 2;
    const step = beamCount > 1 ? fieldOfView / (beamCount - 1) : 0;

    this.p.stroke(0, 255, 0);
    this.p.strokeWeight(10);
    for (const index of discontinuities) {
      if (index + 1 >= this.sensor_ranges.length) {
        continue;
      }
      const angle = startAngle + index * step;
      const range = Math.min(this.sensor_ranges[index], this.sensor_ranges[index + 1]);
      const x = this.current_state.robot.currentPose.x + range * Math.cos(angle);
      const y = this.current_state.robot.currentPose.y + range * Math.sin(angle);
      this.p.point(x * 100, y * 100);
    }

  }

  private drawDiscontinuityLines() {
    const discontinuity_lines = this.get_discontinuity_lines();
    const beamCount = this.sensor_ranges.length;
    if (beamCount === 0 || discontinuity_lines.length === 0) {
      return;
    }

    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const startAngle = this.current_state.robot.currentPose.theta - fieldOfView / 2;
    const step = beamCount > 1 ? fieldOfView / (beamCount - 1) : 0;

    this.p.stroke(255, 165, 0); // Orange color
    this.p.strokeWeight(6);
    for (const line of discontinuity_lines) {
      const startIdx = line.start;
      const endIdx = line.end;

      // Calculate start point coordinates
      if (startIdx + 1 >= this.sensor_ranges.length) {
        continue;
      }
      const startAngleBeam = startAngle + startIdx * step;
      const startRange = Math.min(this.sensor_ranges[startIdx], this.sensor_ranges[startIdx + 1]);
      const startX = this.current_state.robot.currentPose.x + startRange * Math.cos(startAngleBeam);
      const startY = this.current_state.robot.currentPose.y + startRange * Math.sin(startAngleBeam);

      // Calculate end point coordinates
      if (endIdx + 1 >= this.sensor_ranges.length) {
        continue;
      }
      const endAngleBeam = startAngle + endIdx * step;
      const endRange = Math.min(this.sensor_ranges[endIdx], this.sensor_ranges[endIdx + 1]);
      const endX = this.current_state.robot.currentPose.x + endRange * Math.cos(endAngleBeam);
      const endY = this.current_state.robot.currentPose.y + endRange * Math.sin(endAngleBeam);

      // Draw line
      this.p.line(startX * 100, startY * 100, endX * 100, endY * 100);
    }
  }

  pointToLineDistance(
    x: number,
    y: number,
    xStart: number,
    yStart: number,
    xEnd: number,
    yEnd: number
  ): number {
    const dx = xEnd - xStart;
    const dy = yEnd - yStart;

    // If start and end are the same point, it's not really a segment
    if (dx === 0 && dy === 0) {
      return Math.hypot(x - xStart, y - yStart);
    }

    // Project point onto the line, parameterized as start + t * (end - start)
    let t = ((x - xStart) * dx + (y - yStart) * dy) / (dx * dx + dy * dy);

    // Clamp t to [0, 1] so the closest point stays within the segment
    t = Math.max(0, Math.min(1, t));

    // Closest point on the segment
    const closestX = xStart + t * dx;
    const closestY = yStart + t * dy;

    return Math.hypot(x - closestX, y - closestY);
  }

  lines_intersect(
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    x3: number,
    y3: number,
    x4: number,
    y4: number
  ): boolean {
    const orientation = (
      ax: number,
      ay: number,
      bx: number,
      by: number,
      cx: number,
      cy: number
    ) => (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const isOnSegment = (
      ax: number,
      ay: number,
      bx: number,
      by: number,
      cx: number,
      cy: number
    ) =>
      Math.min(ax, cx) <= bx &&
      bx <= Math.max(ax, cx) &&
      Math.min(ay, cy) <= by &&
      by <= Math.max(ay, cy);

    const orientation1 = orientation(x1, y1, x2, y2, x3, y3);
    const orientation2 = orientation(x1, y1, x2, y2, x4, y4);
    const orientation3 = orientation(x3, y3, x4, y4, x1, y1);
    const orientation4 = orientation(x3, y3, x4, y4, x2, y2);

    if (
      ((orientation1 > 0 && orientation2 < 0) || (orientation1 < 0 && orientation2 > 0)) &&
      ((orientation3 > 0 && orientation4 < 0) || (orientation3 < 0 && orientation4 > 0))
    ) {
      return true;
    }

    return (
      (orientation1 === 0 && isOnSegment(x1, y1, x3, y3, x2, y2)) ||
      (orientation2 === 0 && isOnSegment(x1, y1, x4, y4, x2, y2)) ||
      (orientation3 === 0 && isOnSegment(x3, y3, x1, y1, x4, y4)) ||
      (orientation4 === 0 && isOnSegment(x3, y3, x2, y2, x4, y4))
    );
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
      case "bug1":
        if (this.path_to_goal_is_clear() && !this.obstacle_encountered) {
          this.move_towards_goal();
        } else {
          if (!this.obstacle_encountered) {
            this.entrance_x = this.current_state.robot.currentPose.x;
            this.entrance_y = this.current_state.robot.currentPose.y;
            this.obstacle_encountered = true;
          }

          this.is_bug1_following_wall = true;

          let current_distance_to_goal = Math.sqrt(
            Math.pow(this.current_state.robot.currentPose.x - this.current_state.goal.x, 2) +
            Math.pow(this.current_state.robot.currentPose.y - this.current_state.goal.y, 2)
          );

          if (current_distance_to_goal < this.min_distance_to_goal) {
            this.min_distance_to_goal = current_distance_to_goal;
            this.min_distance_to_goal_point = {
              x: this.current_state.robot.currentPose.x,
              y: this.current_state.robot.currentPose.y,
            };
          }

          let current_distance_to_entrance = Math.sqrt(
            Math.pow(this.current_state.robot.currentPose.x - this.entrance_x, 2) +
            Math.pow(this.current_state.robot.currentPose.y - this.entrance_y, 2)
          );
          if (current_distance_to_entrance > 0.5) {
            this.left_entrance = true;
          }

          if (this.left_entrance && current_distance_to_entrance < 0.1) {
            this.go_to_min_distance = true;
          }

          if (this.go_to_min_distance && current_distance_to_goal <= this.min_distance_to_goal + 0.02) {
            this.obstacle_encountered = false;
            this.left_entrance = false;
            this.go_to_min_distance = false;
            this.min_distance_to_goal = Infinity;
            this.is_bug1_following_wall = false;
            this.move_towards_goal();
            break;
          }
          this.follow_wall();
        }
        break;
      case "bug2":
        if (!this.bug2_started) {
          this.m_line_x = this.current_state.robot.currentPose.x;
          this.m_line_y = this.current_state.robot.currentPose.y;
          this.bug2_started = true;
        }

        if (this.path_to_goal_is_clear() && !this.obstacle_encountered) {
          this.move_towards_goal();
        } else {
          if (!this.obstacle_encountered) {
            this.entrance_x = this.current_state.robot.currentPose.x;
            this.entrance_y = this.current_state.robot.currentPose.y;
            this.obstacle_encountered = true;
          }

          const current_distance_to_entrance = Math.sqrt(
            Math.pow(this.current_state.robot.currentPose.x - this.entrance_x, 2) +
            Math.pow(this.current_state.robot.currentPose.y - this.entrance_y, 2)
          );
          if (current_distance_to_entrance > 0.5) {
            this.left_entrance = true;
          }

          const distance_to_m_line = this.pointToLineDistance(
            this.current_state.robot.currentPose.x,
            this.current_state.robot.currentPose.y,
            this.m_line_x,
            this.m_line_y,
            this.current_state.goal.x,
            this.current_state.goal.y
          );

          const distance_of_entrance_to_goal = Math.sqrt(
            Math.pow(this.entrance_x - this.current_state.goal.x, 2) +
            Math.pow(this.entrance_y - this.current_state.goal.y, 2)
          );
          const distance_of_current_to_goal = Math.sqrt(
            Math.pow(this.current_state.robot.currentPose.x - this.current_state.goal.x, 2) +
            Math.pow(this.current_state.robot.currentPose.y - this.current_state.goal.y, 2)
          );
          if (this.left_entrance && distance_to_m_line < 0.05 && distance_of_entrance_to_goal > distance_of_current_to_goal) {
            this.obstacle_encountered = false;
            this.left_entrance = false;
            this.move_towards_goal();
            break;
          }

          this.follow_wall();
        }
        break;
      case "tangentBug":
        let discontinuities = this.get_discontinuities(this.sensor_ranges);
        let lines_intersected = false;
        const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
        const startAngle = this.current_state.robot.currentPose.theta - fieldOfView / 2;
        const isFullCircle = Math.abs(fieldOfView) >= 2 * Math.PI;
        const angleStep = this.sensor_ranges.length > 1
          ? fieldOfView / (isFullCircle ? this.sensor_ranges.length : this.sensor_ranges.length - 1)
          : 0;
        let bestIndex = -1;
        if (this.path_to_goal_is_clear() && !this.obstacle_encountered) {
          if (discontinuities.length > 1) {
            //check if robot to goal line intersects with any discontinuity lines
            let discontinuity_lines = this.get_discontinuity_lines();
            for (let i = 0; i < discontinuity_lines.length; i++) {
              const { start, end } = discontinuity_lines[i];
              const startRange = Math.min(this.sensor_ranges[start], this.sensor_ranges[start + 1]);
              const endRange = Math.min(this.sensor_ranges[end], this.sensor_ranges[end + 1]);
              const startBeamAngle = startAngle + start * angleStep;
              const endBeamAngle = startAngle + end * angleStep;
              if (this.lines_intersect(
                this.current_state.robot.currentPose.x,
                this.current_state.robot.currentPose.y,
                this.current_state.goal.x,
                this.current_state.goal.y,
                this.current_state.robot.currentPose.x + Math.cos(startBeamAngle) * startRange,
                this.current_state.robot.currentPose.y + Math.sin(startBeamAngle) * startRange,
                this.current_state.robot.currentPose.x + Math.cos(endBeamAngle) * endRange,
                this.current_state.robot.currentPose.y + Math.sin(endBeamAngle) * endRange
              )) {
                lines_intersected = true;
                break;
              }
            }
            if (lines_intersected) {
              let minHeuristic = Infinity;
              for (let i = 0; i < discontinuities.length; i++) {
                let heuristic = this.calculate_discontinuity_heuristic(discontinuities[i]);
                if (heuristic < minHeuristic) {
                  minHeuristic = heuristic;
                  bestIndex = discontinuities[i];
                }
              }
              // Move towards the discontinuity with the minimum heuristic
              if (bestIndex !== -1) {
                this.move_towards_x_y(
                  this.current_state.robot.currentPose.x + Math.cos(startAngle + bestIndex * angleStep) * this.sensor_ranges[bestIndex],
                  this.current_state.robot.currentPose.y + Math.sin(startAngle + bestIndex * angleStep) * this.sensor_ranges[bestIndex]
                );
              }
            } else {
              this.move_towards_goal();
            }
          } else {
            this.move_towards_goal();
          }
        } else {
          let minHeuristic = Infinity;
          for (let i = 0; i < discontinuities.length; i++) {
            let heuristic = this.calculate_discontinuity_heuristic(discontinuities[i]);
              if (heuristic < minHeuristic) {
                minHeuristic = heuristic;
                bestIndex = discontinuities[i];
            }
          }
          if (!this.obstacle_encountered && bestIndex !== -1) {
            if (bestIndex !== -1) {
              this.follow_direction = this.left_or_right(bestIndex);
            }
          }
          
          this.obstacle_encountered = true;
          if (bestIndex === -1) {
            this.follow_wall(this.follow_direction);
            break;
          }

          //if robot distance to goal is less than bestIndex distance to goal, move towards goal, else follow wall
          let distance_to_goal = Math.sqrt(
            Math.pow(this.current_state.robot.currentPose.x - this.current_state.goal.x, 2) +
            Math.pow(this.current_state.robot.currentPose.y - this.current_state.goal.y, 2)
          );

          let bestIndex_distance_to_goal = Math.sqrt(
            Math.pow(this.current_state.robot.currentPose.x + Math.cos(startAngle + bestIndex * angleStep) * this.sensor_ranges[bestIndex] - this.current_state.goal.x, 2) +
            Math.pow(this.current_state.robot.currentPose.y + Math.sin(startAngle + bestIndex * angleStep) * this.sensor_ranges[bestIndex] - this.current_state.goal.y, 2)
          );

          if (distance_to_goal < bestIndex_distance_to_goal) {
            this.move_towards_goal();
            this.obstacle_encountered = false;
          } else {
            this.follow_wall(this.follow_direction);
          }
        }
        this.drawDiscontinuities();
        this.drawDiscontinuityLines();
        break;
      case "potentialField": 
      if (this.path_to_goal_is_clear()) {
          this.move_towards_goal();
        } else {
          this.ux = -1;
          this.uy = -1;
        }
        break;
      case "PRM":
        if (this.path_to_goal_is_clear()) {
          this.move_towards_goal();
        } else {
          this.ux = -1;
          this.uy = 0;
        }
        break;
      case "RRT":
        if (this.path_to_goal_is_clear()) {
          this.move_towards_goal();
        } else {
          this.ux = 0;
          this.uy = -1;
        }
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
    const phi = 1; // rad/s

    const dt = this.current_state.simulation.timestep;
    if (distance < speed * dt) {
      this.current_state.robot.currentPose.x = this.current_state.goal.x;
      this.current_state.robot.currentPose.y = this.current_state.goal.y;
    } else {
      this.current_state.robot.currentPose.x += this.ux * speed * dt;
      this.current_state.robot.currentPose.y += this.uy * speed * dt;
      this.current_state.robot.currentPose.theta += this.utheta * phi * dt;
    }
  }
};
