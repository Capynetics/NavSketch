import type p5 from "p5";
import type { ParametersState, ScenarioObstacle } from "../types/parameters";
import { Lidar } from "./sensors/lidar";

type RoadmapPoint = { x: number; y: number };
type RoadmapEdge = { start: number; end: number };
type Roadmap = { points: RoadmapPoint[]; edges: RoadmapEdge[] };
type RrtNode = RoadmapPoint & { parent: number | null };
type RrtPhase = "growing" | "moving" | "failed";
type WavefrontPhase = "propagating" | "moving" | "failed";
type WavefrontGrid = {
  cellSize: number;
  columns: number;
  rows: number;
  distances: number[];
  blocked: boolean[];
  queue: number[];
  queueIndex: number;
  robotCell: number;
  goalCell: number;
  robotPosition: RoadmapPoint;
  goalPosition: RoadmapPoint;
};
type PrmPhase = "sampling" | "connecting" | "searching" | "moving";
type RoadmapSearch = {
  points: RoadmapPoint[];
  neighbors: number[][];
  startIndex: number;
  goalIndex: number;
  distanceFromStart: number[];
  previous: number[];
  openNodes: Set<number>;
  closedNodes: Set<number>;
  currentIndex: number | null;
};

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
  map_needs_to_be_built: boolean;
  min_distance_to_goal_point: { x: number; y: number };
  m_line_x: number;
  m_line_y: number;
  private bug2_started: boolean;
  roadmap: Roadmap = { points: [], edges: [] };
  private prmPath: RoadmapPoint[] = [];
  private prmWaypointIndex: number = 0;
  private prmPhase: PrmPhase = "sampling";
  private prmConnectionIndex: number = 0;
  private prmSearch: RoadmapSearch | null = null;
  private rrtNodes: RrtNode[] = [];
  private rrtPath: RoadmapPoint[] = [];
  private rrtWaypointIndex: number = 0;
  private rrtPhase: RrtPhase = "growing";
  private rrtNeedsToBeBuilt: boolean = true;
  private rrtAttempts: number = 0;
  private wavefrontGrid: WavefrontGrid | null = null;
  private wavefrontPath: RoadmapPoint[] = [];
  private wavefrontWaypointIndex: number = 0;
  private wavefrontPhase: WavefrontPhase = "propagating";
  private wavefrontNeedsToBeBuilt: boolean = true;

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
    this.map_needs_to_be_built = true; // Flag to indicate if the roadmap needs to be built

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
    } else {
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

  get_discontinuity_lines() {
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
        if (discontinuities[i + 1] !== undefined) {
          discontinuity_lines.push({ start: discontinuities[i], end: discontinuities[i + 1] });
        } else {
          discontinuity_lines.push({ start: discontinuities[i], end: discontinuities[0] });
        }
      }
    }

    for (let i = 0; i < discontinuity_lines.length; i++) {

      if (discontinuity_lines[i].start < discontinuity_lines[i].end) {
        if (ranges[Math.floor((discontinuity_lines[i].start + discontinuity_lines[i].end) / 2)] == this.current_state.lidar.range) {
          //remove discontinuity_lines[i] from the array
          discontinuity_lines.splice(i, 1);
          i--;
        }
      } else {
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

  private drawForceVector(x: number, y: number, r: number, g: number, b: number, scale = 0.5) {
    const originX = this.current_state.robot.currentPose.x;
    const originY = this.current_state.robot.currentPose.y;
    this.p.stroke(r, g, b);
    this.p.strokeWeight(4);
    this.p.line(
      originX * 100,
      originY * 100,
      (originX + x * scale) * 100,
      (originY + y * scale) * 100
    );
  }

  private drawPotentialFieldVectors(
    attractiveX: number,
    attractiveY: number,
    repulsiveX: number,
    repulsiveY: number
  ) {
    this.drawForceVector(attractiveX, attractiveY, 0, 0, 255); // blue = attractive
    this.drawForceVector(repulsiveX, repulsiveY, 255, 0, 255); // magenta = repulsive
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

  point_in_obstacle(x: number, y: number, obstacle: ScenarioObstacle): boolean {
    const dx = x - obstacle.pose.x;
    const dy = y - obstacle.pose.y;
    const clearance = this.current_state.robot.radius + 0.01;

    if (obstacle.type === "circle") {
      const radius = (obstacle.geometry.radius ?? 0) + clearance;
      return radius > 0 && dx * dx + dy * dy <= radius * radius;
    }

    if (obstacle.type === "rectangle") {
      const width = obstacle.geometry.width ?? 0;
      const height = obstacle.geometry.height ?? 0;
      if (width <= 0 || height <= 0) {
        return false;
      }

      const theta = obstacle.pose.theta ?? 0;
      const cosTheta = Math.cos(theta);
      const sinTheta = Math.sin(theta);
      const localX = dx * cosTheta + dy * sinTheta;
      const localY = -dx * sinTheta + dy * cosTheta;

      return (
        Math.abs(localX) <= width / 2 + clearance &&
        Math.abs(localY) <= height / 2 + clearance
      );
    }

    return false;
  }
  
  private pathIsClear(start: RoadmapPoint, end: RoadmapPoint): boolean {
    const distance = Math.hypot(end.x - start.x, end.y - start.y);
    const sampleSpacing = Math.max(this.current_state.robot.radius / 2, 0.01);
    const sampleCount = Math.max(1, Math.ceil(distance / sampleSpacing));

    for (let index = 0; index <= sampleCount; index += 1) {
      const progress = index / sampleCount;
      const x = start.x + (end.x - start.x) * progress;
      const y = start.y + (end.y - start.y) * progress;

      if (this.current_state.obstacles.some((obstacle) => this.point_in_obstacle(x, y, obstacle))) {
        return false;
      }
    }

    return true;
  }

  buildRoadMap(): Roadmap {
    return { points: [], edges: [] };
  }

  private addRoadmapSamples(sampleCount: number) {
    while (sampleCount > 0) {
      const x = Math.random() * this.current_state.simulation.canvasWidth / 100;
      const y = Math.random() * this.current_state.simulation.canvasHeight / 100;
      const inObstacle = this.current_state.obstacles.some((obstacle) =>
        this.point_in_obstacle(x, y, obstacle)
      );
      if (!inObstacle) {
        this.roadmap.points.push({ x, y });
        sampleCount -= 1;
      }
    }
  }

  private connectRoadmapPoints(connectionCount: number) {
    const neighborCount = 12;
    const connectedPairs = new Set(this.roadmap.edges.map(({ start, end }) => `${start}:${end}`));

    while (connectionCount > 0 && this.prmConnectionIndex < this.roadmap.points.length) {
      const start = this.prmConnectionIndex;
      const nearestNeighbors = this.roadmap.points
        .map((point, end) => ({ end, distance: Math.hypot(point.x - this.roadmap.points[start].x, point.y - this.roadmap.points[start].y) }))
        .filter(({ end }) => end !== start)
        .sort((first, second) => first.distance - second.distance)
        .slice(0, neighborCount);

      for (const { end } of nearestNeighbors) {
        const edgeStart = Math.min(start, end);
        const edgeEnd = Math.max(start, end);
        const edgeKey = `${edgeStart}:${edgeEnd}`;
        if (!connectedPairs.has(edgeKey) && this.pathIsClear(this.roadmap.points[edgeStart], this.roadmap.points[edgeEnd])) {
          this.roadmap.edges.push({ start: edgeStart, end: edgeEnd });
          connectedPairs.add(edgeKey);
        }
      }
      this.prmConnectionIndex += 1;
      connectionCount -= 1;
    }
  }

  useRoadMap(
    roadmap: Roadmap,
    robotPosition: RoadmapPoint,
    goalPosition: RoadmapPoint
  ): RoadmapPoint[] {
    const neighborCount = 12;
    const startIndex = roadmap.points.length;
    const goalIndex = startIndex + 1;
    const points = [...roadmap.points, robotPosition, goalPosition];
    const edges = [...roadmap.edges];
    const connectedPairs = new Set(edges.map(({ start, end }) => `${Math.min(start, end)}:${Math.max(start, end)}`));

    for (const nodeIndex of [startIndex, goalIndex]) {
      const nearestNeighbors = points
        .map((point, index) => ({ index, distance: Math.hypot(point.x - points[nodeIndex].x, point.y - points[nodeIndex].y) }))
        .filter(({ index }) => index !== nodeIndex)
        .sort((first, second) => first.distance - second.distance)
        .slice(0, neighborCount);

      for (const { index } of nearestNeighbors) {
        const start = Math.min(nodeIndex, index);
        const end = Math.max(nodeIndex, index);
        const edgeKey = `${start}:${end}`;
        if (!connectedPairs.has(edgeKey) && this.pathIsClear(points[start], points[end])) {
          edges.push({ start, end });
          connectedPairs.add(edgeKey);
        }
      }
    }

    const neighbors = Array.from({ length: points.length }, () => [] as number[]);
    for (const { start, end } of edges) {
      neighbors[start].push(end);
      neighbors[end].push(start);
    }

    const distanceFromStart = Array(points.length).fill(Infinity);
    const previous = Array(points.length).fill(-1);
    const openNodes = new Set<number>([startIndex]);
    distanceFromStart[startIndex] = 0;

    while (openNodes.size > 0) {
      let currentIndex = -1;
      let bestScore = Infinity;
      for (const index of openNodes) {
        const estimatedTotal = distanceFromStart[index] + Math.hypot(
          points[goalIndex].x - points[index].x,
          points[goalIndex].y - points[index].y
        );
        if (estimatedTotal < bestScore) {
          bestScore = estimatedTotal;
          currentIndex = index;
        }
      }

      if (currentIndex === goalIndex) {
        const path: RoadmapPoint[] = [];
        for (let index = goalIndex; index !== -1; index = previous[index]) {
          path.unshift(points[index]);
        }
        return path;
      }

      openNodes.delete(currentIndex);
      for (const neighborIndex of neighbors[currentIndex]) {
        const candidateDistance = distanceFromStart[currentIndex] + Math.hypot(
          points[neighborIndex].x - points[currentIndex].x,
          points[neighborIndex].y - points[currentIndex].y
        );
        if (candidateDistance < distanceFromStart[neighborIndex]) {
          distanceFromStart[neighborIndex] = candidateDistance;
          previous[neighborIndex] = currentIndex;
          openNodes.add(neighborIndex);
        }
      }
    }

    return [];
  }

  private beginRoadMapSearch(robotPosition: RoadmapPoint, goalPosition: RoadmapPoint): RoadmapSearch {
    const startIndex = this.roadmap.points.length;
    const goalIndex = startIndex + 1;
    const points = [...this.roadmap.points, robotPosition, goalPosition];
    const edges = [...this.roadmap.edges];
    const connectedPairs = new Set(edges.map(({ start, end }) => `${Math.min(start, end)}:${Math.max(start, end)}`));

    for (const nodeIndex of [startIndex, goalIndex]) {
      const nearestNeighbors = points
        .map((point, index) => ({ index, distance: Math.hypot(point.x - points[nodeIndex].x, point.y - points[nodeIndex].y) }))
        .filter(({ index }) => index !== nodeIndex)
        .sort((first, second) => first.distance - second.distance)
        .slice(0, 12);
      for (const { index } of nearestNeighbors) {
        const start = Math.min(nodeIndex, index);
        const end = Math.max(nodeIndex, index);
        const edgeKey = `${start}:${end}`;
        if (!connectedPairs.has(edgeKey) && this.pathIsClear(points[start], points[end])) {
          edges.push({ start, end });
          connectedPairs.add(edgeKey);
        }
      }
    }

    const neighbors = Array.from({ length: points.length }, () => [] as number[]);
    for (const { start, end } of edges) {
      neighbors[start].push(end);
      neighbors[end].push(start);
    }
    const distanceFromStart = Array(points.length).fill(Infinity);
    distanceFromStart[startIndex] = 0;
    return {
      points,
      neighbors,
      startIndex,
      goalIndex,
      distanceFromStart,
      previous: Array(points.length).fill(-1),
      openNodes: new Set<number>([startIndex]),
      closedNodes: new Set<number>(),
      currentIndex: null,
    };
  }

  private advanceRoadMapSearch(search: RoadmapSearch, expansionCount: number): RoadmapPoint[] | null {
    while (expansionCount > 0 && search.openNodes.size > 0) {
      let currentIndex = -1;
      let bestScore = Infinity;
      for (const index of search.openNodes) {
        const estimatedTotal = search.distanceFromStart[index] + Math.hypot(
          search.points[search.goalIndex].x - search.points[index].x,
          search.points[search.goalIndex].y - search.points[index].y
        );
        if (estimatedTotal < bestScore) {
          bestScore = estimatedTotal;
          currentIndex = index;
        }
      }
      search.currentIndex = currentIndex;
      if (currentIndex === search.goalIndex) {
        const path: RoadmapPoint[] = [];
        for (let index = search.goalIndex; index !== -1; index = search.previous[index]) {
          path.unshift(search.points[index]);
        }
        return path;
      }
      search.openNodes.delete(currentIndex);
      search.closedNodes.add(currentIndex);
      for (const neighborIndex of search.neighbors[currentIndex]) {
        const candidateDistance = search.distanceFromStart[currentIndex] + Math.hypot(
          search.points[neighborIndex].x - search.points[currentIndex].x,
          search.points[neighborIndex].y - search.points[currentIndex].y
        );
        if (candidateDistance < search.distanceFromStart[neighborIndex]) {
          search.distanceFromStart[neighborIndex] = candidateDistance;
          search.previous[neighborIndex] = currentIndex;
          search.openNodes.add(neighborIndex);
        }
      }
      expansionCount -= 1;
    }
    return search.openNodes.size === 0 ? [] : null;
  }

  private growRrt(goal: RoadmapPoint) {
    const stepSize = 0.35;
    const maxAttempts = 10000;
    const randomTarget = Math.random() < 0.1
      ? goal
      : {
          x: Math.random() * this.current_state.simulation.canvasWidth / 100,
          y: Math.random() * this.current_state.simulation.canvasHeight / 100,
        };
    let nearestIndex = 0;
    let nearestDistance = Infinity;
    for (let index = 0; index < this.rrtNodes.length; index += 1) {
      const distance = Math.hypot(
        randomTarget.x - this.rrtNodes[index].x,
        randomTarget.y - this.rrtNodes[index].y
      );
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestIndex = index;
      }
    }

    const nearest = this.rrtNodes[nearestIndex];
    this.rrtAttempts += 1;
    if (nearestDistance < stepSize) {
      if (this.rrtAttempts >= maxAttempts) {
        this.rrtPhase = "failed";
      }
      return;
    }

    const scale = stepSize / nearestDistance;
    const newPoint = {
      x: nearest.x + (randomTarget.x - nearest.x) * scale,
      y: nearest.y + (randomTarget.y - nearest.y) * scale,
    };
    if (Number.isFinite(scale) && this.pathIsClear(nearest, newPoint)) {
      this.rrtNodes.push({ ...newPoint, parent: nearestIndex });
      const newIndex = this.rrtNodes.length - 1;
      if (Math.hypot(goal.x - newPoint.x, goal.y - newPoint.y) <= stepSize && this.pathIsClear(newPoint, goal)) {
        this.rrtNodes.push({ ...goal, parent: newIndex });
        this.rrtPath = this.getRrtPath(this.rrtNodes.length - 1);
        this.rrtWaypointIndex = 1;
        this.rrtPhase = "moving";
      }
    }

    if (this.rrtAttempts >= maxAttempts && this.rrtPhase === "growing") {
      this.rrtPhase = "failed";
    }
  }

  private getRrtPath(goalIndex: number): RoadmapPoint[] {
    const path: RoadmapPoint[] = [];
    for (let index: number | null = goalIndex; index !== null; index = this.rrtNodes[index].parent) {
      path.unshift(this.rrtNodes[index]);
    }
    return path;
  }

  private drawRrt() {
    if (!this.current_state.visualization.showPlannerGraph) {
      return;
    }

    this.p.stroke(70, 120, 180);
    this.p.strokeWeight(1);
    for (const node of this.rrtNodes) {
      if (node.parent === null) {
        continue;
      }
      const parent = this.rrtNodes[node.parent];
      this.p.line(parent.x * 100, parent.y * 100, node.x * 100, node.y * 100);
    }
    this.p.stroke(0, 180, 0);
    this.p.strokeWeight(3);
    for (let index = this.rrtWaypointIndex; index < this.rrtPath.length - 1; index += 1) {
      const start = this.rrtPath[index];
      const end = this.rrtPath[index + 1];
      this.p.line(start.x * 100, start.y * 100, end.x * 100, end.y * 100);
    }
    this.p.noStroke();
  }

  private initializeWavefront(): boolean {
    const cellSize = 0.2;
    const columns = Math.ceil(this.current_state.simulation.canvasWidth / (cellSize * 100));
    const rows = Math.ceil(this.current_state.simulation.canvasHeight / (cellSize * 100));
    const cellCount = columns * rows;
    const toCell = (point: RoadmapPoint) => {
      const column = Math.max(0, Math.min(columns - 1, Math.floor(point.x / cellSize)));
      const row = Math.max(0, Math.min(rows - 1, Math.floor(point.y / cellSize)));
      return row * columns + column;
    };
    const blocked = Array(cellCount).fill(false);
    for (let index = 0; index < cellCount; index += 1) {
      const column = index % columns;
      const row = Math.floor(index / columns);
      blocked[index] = this.current_state.obstacles.some((obstacle) =>
        this.point_in_obstacle((column + 0.5) * cellSize, (row + 0.5) * cellSize, obstacle)
      );
    }

    const robotPosition = { ...this.current_state.robot.currentPose };
    const goalPosition = { ...this.current_state.goal };
    const robotCell = toCell(robotPosition);
    const goalCell = toCell(goalPosition);
    if (blocked[robotCell] || blocked[goalCell]) {
      return false;
    }

    const distances = Array(cellCount).fill(-1);
    distances[goalCell] = 0;
    this.wavefrontGrid = {
      cellSize,
      columns,
      rows,
      distances,
      blocked,
      queue: [goalCell],
      queueIndex: 0,
      robotCell,
      goalCell,
      robotPosition,
      goalPosition,
    };
    return true;
  }

  private advanceWavefront(cellCount: number): boolean | null {
    const grid = this.wavefrontGrid;
    if (!grid) {
      return false;
    }
    const directions = [[0, -1], [1, 0], [0, 1], [-1, 0]];
    while (cellCount > 0 && grid.queueIndex < grid.queue.length) {
      const currentCell = grid.queue[grid.queueIndex];
      grid.queueIndex += 1;
      if (currentCell === grid.robotCell) {
        this.wavefrontPath = this.getWavefrontPath();
        this.wavefrontWaypointIndex = 1;
        return true;
      }

      const column = currentCell % grid.columns;
      const row = Math.floor(currentCell / grid.columns);
      for (const [columnOffset, rowOffset] of directions) {
        const neighborColumn = column + columnOffset;
        const neighborRow = row + rowOffset;
        if (
          neighborColumn < 0 || neighborColumn >= grid.columns ||
          neighborRow < 0 || neighborRow >= grid.rows
        ) {
          continue;
        }
        const neighborCell = neighborRow * grid.columns + neighborColumn;
        if (!grid.blocked[neighborCell] && grid.distances[neighborCell] === -1) {
          grid.distances[neighborCell] = grid.distances[currentCell] + 1;
          grid.queue.push(neighborCell);
        }
      }
      cellCount -= 1;
    }
    return grid.queueIndex === grid.queue.length ? false : null;
  }

  private getWavefrontPath(): RoadmapPoint[] {
    const grid = this.wavefrontGrid;
    if (!grid || grid.distances[grid.robotCell] === -1) {
      return [];
    }

    const directions = [[0, -1], [1, 0], [0, 1], [-1, 0]];
    const path = [grid.robotPosition];
    let currentCell = grid.robotCell;
    while (currentCell !== grid.goalCell) {
      const column = currentCell % grid.columns;
      const row = Math.floor(currentCell / grid.columns);
      let nextCell = -1;
      for (const [columnOffset, rowOffset] of directions) {
        const neighborColumn = column + columnOffset;
        const neighborRow = row + rowOffset;
        if (
          neighborColumn < 0 || neighborColumn >= grid.columns ||
          neighborRow < 0 || neighborRow >= grid.rows
        ) {
          continue;
        }
        const neighborCell = neighborRow * grid.columns + neighborColumn;
        if (grid.distances[neighborCell] === grid.distances[currentCell] - 1) {
          nextCell = neighborCell;
          break;
        }
      }
      if (nextCell === -1) {
        return [];
      }
      currentCell = nextCell;
      path.push({
        x: (currentCell % grid.columns + 0.5) * grid.cellSize,
        y: (Math.floor(currentCell / grid.columns) + 0.5) * grid.cellSize,
      });
    }
    path.push(grid.goalPosition);
    return path;
  }

  private drawWavefront() {
    const grid = this.wavefrontGrid;
    if (!grid || !this.current_state.visualization.showPlannerGraph) {
      return;
    }

    this.p.noStroke();
    this.p.fill(0, 150, 255, 45);
    this.p.textAlign(this.p.CENTER, this.p.CENTER);
    this.p.textSize(12);
    for (let index = 0; index < grid.distances.length; index += 1) {
      if (grid.distances[index] < 0) {
        continue;
      }
      const column = index % grid.columns;
      const row = Math.floor(index / grid.columns);
      this.p.rect(
        column * grid.cellSize * 100,
        row * grid.cellSize * 100,
        grid.cellSize * 100,
        grid.cellSize * 100
      );
      this.p.fill(0);
      this.p.text(
        grid.distances[index],
        (column + 0.5) * grid.cellSize * 100,
        (row + 0.5) * grid.cellSize * 100
      );
      this.p.fill(0, 150, 255, 45);
    }
    this.p.stroke(0, 180, 0);
    this.p.strokeWeight(3);
    for (let index = this.wavefrontWaypointIndex; index < this.wavefrontPath.length - 1; index += 1) {
      const start = this.wavefrontPath[index];
      const end = this.wavefrontPath[index + 1];
      this.p.line(start.x * 100, start.y * 100, end.x * 100, end.y * 100);
    }
    this.p.noStroke();
  }

  drawRoadMap(
    points: RoadmapPoint[],
    edges: RoadmapEdge[],
    path: RoadmapPoint[],
    search: RoadmapSearch | null
  ) {
    if (!this.current_state.visualization.showPlannerGraph) {
      return;
    }

    this.p.stroke(80, 80, 80);
    this.p.strokeWeight(1);
    for (const edge of edges) {
      const start = points[edge.start];
      const end = points[edge.end];
      this.p.line(start.x * 100, start.y * 100, end.x * 100, end.y * 100);
    }
    if (search) {
      this.p.stroke(230, 140, 0);
      this.p.strokeWeight(6);
      for (const index of search.closedNodes) {
        const point = search.points[index];
        this.p.point(point.x * 100, point.y * 100);
      }
      this.p.stroke(0, 150, 255);
      this.p.strokeWeight(7);
      for (const index of search.openNodes) {
        const point = search.points[index];
        this.p.point(point.x * 100, point.y * 100);
      }
      if (search.currentIndex !== null) {
        const point = search.points[search.currentIndex];
        this.p.stroke(255, 255, 255);
        this.p.strokeWeight(10);
        this.p.point(point.x * 100, point.y * 100);
      }
    }
    this.p.stroke(0, 180, 0);
    this.p.strokeWeight(3);
    for (let index = 0; index < path.length - 1; index += 1) {
      const start = path[index];
      const end = path[index + 1];
      this.p.line(start.x * 100, start.y * 100, end.x * 100, end.y * 100);
    }
    this.p.stroke(80, 80, 80);
    this.p.strokeWeight(5);
    for (const point of points) {
      this.p.point(point.x * 100, point.y * 100);
    }
    this.p.noStroke();
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
        //Calculate atractive force towards goal
        let dx = this.current_state.goal.x - this.current_state.robot.currentPose.x;
        let dy = this.current_state.goal.y - this.current_state.robot.currentPose.y;
        let distance = Math.sqrt(dx * dx + dy * dy);
        if (distance <= 1e-9) {
          this.ux = 0;
          this.uy = 0;
          return;
        }
        let attractive_force_x = dx / distance;
        let attractive_force_y = dy / distance;

        //Calculate repulsive force from obstacles
        let repulsive_force_x = 0;
        let repulsive_force_y = 0;
        const potentialFieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
        const potentialFieldStartAngle = this.current_state.robot.currentPose.theta - potentialFieldOfView / 2;
        const potentialFieldIsFullCircle = Math.abs(potentialFieldOfView) >= 2 * Math.PI;
        const potentialFieldAngleStep = this.sensor_ranges.length > 1
          ? potentialFieldOfView / (potentialFieldIsFullCircle ? this.sensor_ranges.length : this.sensor_ranges.length - 1)
          : 0;
        for (let i = 0; i < this.sensor_ranges.length; i++) {
          let range = this.sensor_ranges[i];
          if (range < this.current_state.lidar.range) {
            let angle = potentialFieldStartAngle + i * potentialFieldAngleStep;
            let obstacle_x = this.current_state.robot.currentPose.x + range * Math.cos(angle);
            let obstacle_y = this.current_state.robot.currentPose.y + range * Math.sin(angle);
            let dx_obstacle = this.current_state.robot.currentPose.x - obstacle_x;
            let dy_obstacle = this.current_state.robot.currentPose.y - obstacle_y;
            let distance_to_obstacle = Math.sqrt(dx_obstacle * dx_obstacle + dy_obstacle * dy_obstacle);
            if (distance_to_obstacle <= 1e-9) {
              continue;
            }

            let repulsive_force_magnitude = (1 / distance_to_obstacle) * 0.03;
            repulsive_force_x += (dx_obstacle / distance_to_obstacle) * repulsive_force_magnitude;
            repulsive_force_y += (dy_obstacle / distance_to_obstacle) * repulsive_force_magnitude;
          }
        }

        // Clamp repulsive force magnitude to the attractive force's magnitude
        const attractive_force_magnitude = Math.hypot(attractive_force_x, attractive_force_y);
        const repulsive_force_magnitude_total = Math.hypot(repulsive_force_x, repulsive_force_y);
        if (repulsive_force_magnitude_total > attractive_force_magnitude) {
          const clampScale = attractive_force_magnitude / repulsive_force_magnitude_total;
          repulsive_force_x *= clampScale;
          repulsive_force_y *= clampScale;
        }

        this.drawPotentialFieldVectors(attractive_force_x, attractive_force_y, repulsive_force_x, repulsive_force_y);

        //Combine attractive and repulsive forces
        this.ux = attractive_force_x + repulsive_force_x;
        this.uy = attractive_force_y + repulsive_force_y;
        let combined_distance = Math.sqrt(this.ux * this.ux + this.uy * this.uy);
        if (combined_distance <= 1e-9) {
          this.ux = 0;
          this.uy = 0;
          return;
        }
        this.ux /= combined_distance;
        this.uy /= combined_distance;
        break;
      case "PRM":
        if (this.map_needs_to_be_built) {
          this.roadmap = this.buildRoadMap();
          this.prmPath = [];
          this.prmWaypointIndex = 0;
          this.prmPhase = "sampling";
          this.prmConnectionIndex = 0;
          this.prmSearch = null;
          this.map_needs_to_be_built = false;
        }

        if (this.prmPhase === "sampling") {
          this.addRoadmapSamples(Math.min(12, 1000 - this.roadmap.points.length));
          if (this.roadmap.points.length === 1000) {
            this.prmPhase = "connecting";
          }
          this.ux = 0;
          this.uy = 0;
        } else if (this.prmPhase === "connecting") {
          this.connectRoadmapPoints(20);
          if (this.prmConnectionIndex === this.roadmap.points.length) {
            this.prmSearch = this.beginRoadMapSearch(
              this.current_state.robot.currentPose,
              this.current_state.goal
            );
            this.prmPhase = "searching";
          }
          this.ux = 0;
          this.uy = 0;
        } else if (this.prmPhase === "searching" && this.prmSearch) {
          const path = this.advanceRoadMapSearch(this.prmSearch, 1);
          if (path !== null) {
            this.prmPath = path;
            this.prmWaypointIndex = 1;
            this.prmSearch = null;
            this.prmPhase = "moving";
          }
          this.ux = 0;
          this.uy = 0;
        } else if (this.prmPhase === "moving") {
          const waypointArrivalDistance = 2 * this.current_state.simulation.timestep;
          while (this.prmWaypointIndex < this.prmPath.length - 1) {
            const waypoint = this.prmPath[this.prmWaypointIndex];
            const distanceToWaypoint = Math.hypot(
              waypoint.x - this.current_state.robot.currentPose.x,
              waypoint.y - this.current_state.robot.currentPose.y
            );
            if (distanceToWaypoint > waypointArrivalDistance) {
              break;
            }
            this.prmWaypointIndex += 1;
          }

          if (this.prmWaypointIndex < this.prmPath.length) {
            const waypoint = this.prmPath[this.prmWaypointIndex];
            this.move_towards_x_y(waypoint.x, waypoint.y);
          } else {
            this.ux = 0;
            this.uy = 0;
          }
        }
        this.drawRoadMap(
          this.roadmap.points,
          this.roadmap.edges,
          this.prmPath.slice(this.prmWaypointIndex),
          this.prmSearch
        );
        break;
      case "RRT":
        if (this.rrtNeedsToBeBuilt) {
          this.rrtNodes = [{ ...this.current_state.robot.currentPose, parent: null }];
          this.rrtPath = [];
          this.rrtWaypointIndex = 0;
          this.rrtPhase = "growing";
          this.rrtAttempts = 0;
          this.rrtNeedsToBeBuilt = false;
        }

        if (this.rrtPhase === "growing") {
          this.growRrt(this.current_state.goal);
          this.ux = 0;
          this.uy = 0;
        } else if (this.rrtPhase === "moving") {
          const waypointArrivalDistance = 2 * this.current_state.simulation.timestep;
          while (this.rrtWaypointIndex < this.rrtPath.length - 1) {
            const waypoint = this.rrtPath[this.rrtWaypointIndex];
            if (Math.hypot(
              waypoint.x - this.current_state.robot.currentPose.x,
              waypoint.y - this.current_state.robot.currentPose.y
            ) > waypointArrivalDistance) {
              break;
            }
            this.rrtWaypointIndex += 1;
          }
          if (this.rrtWaypointIndex < this.rrtPath.length) {
            const waypoint = this.rrtPath[this.rrtWaypointIndex];
            this.move_towards_x_y(waypoint.x, waypoint.y);
          } else {
            this.ux = 0;
            this.uy = 0;
          }
        } else {
          this.ux = 0;
          this.uy = 0;
        }
        this.drawRrt();
        break;
      case "wavefront":
        if (this.wavefrontNeedsToBeBuilt) {
          this.wavefrontPath = [];
          this.wavefrontWaypointIndex = 0;
          this.wavefrontPhase = this.initializeWavefront() ? "propagating" : "failed";
          this.wavefrontNeedsToBeBuilt = false;
        }

        if (this.wavefrontPhase === "propagating") {
          const result = this.advanceWavefront(20);
          if (result === true) {
            this.wavefrontPhase = "moving";
          } else if (result === false) {
            this.wavefrontPhase = "failed";
          }
          this.ux = 0;
          this.uy = 0;
        } else if (this.wavefrontPhase === "moving") {
          const waypointArrivalDistance = 2 * this.current_state.simulation.timestep;
          while (this.wavefrontWaypointIndex < this.wavefrontPath.length - 1) {
            const waypoint = this.wavefrontPath[this.wavefrontWaypointIndex];
            if (Math.hypot(
              waypoint.x - this.current_state.robot.currentPose.x,
              waypoint.y - this.current_state.robot.currentPose.y
            ) > waypointArrivalDistance) {
              break;
            }
            this.wavefrontWaypointIndex += 1;
          }
          if (this.wavefrontWaypointIndex < this.wavefrontPath.length) {
            const waypoint = this.wavefrontPath[this.wavefrontWaypointIndex];
            this.move_towards_x_y(waypoint.x, waypoint.y);
          } else {
            this.ux = 0;
            this.uy = 0;
          }
        } else {
          this.ux = 0;
          this.uy = 0;
        }
        this.drawWavefront();
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
