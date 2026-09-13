import type { ParametersState, ScenarioObstacle } from "../types/parameters";
import { Lidar } from "./sensors/lidar";

export type RoadmapPoint = { x: number; y: number };
export type RoadmapEdge = { start: number; end: number };
export type Roadmap = { points: RoadmapPoint[]; edges: RoadmapEdge[] };
export type RrtNode = RoadmapPoint & { parent: number | null };
type RrtPhase = "growing" | "moving" | "failed";
type WavefrontPhase = "propagating" | "moving" | "failed";
export type WavefrontGrid = {
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
export type RoadmapSearch = {
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
  private prmGridPoints: RoadmapPoint[] = [];
  private prmGridIndex: number = 0;
  private prmGridColumns: number = 0;
  private prmGridCellToPointIndex: number[] = [];
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
  private potentialFieldForces: { attractiveX: number; attractiveY: number; repulsiveX: number; repulsiveY: number } | null = null;
  tangent_state: string = "go_to_target";
  d_followed: number = Infinity;
  d_reach: number | undefined;
  d_temp: number | undefined;
  private followedDiscontinuityPoint: RoadmapPoint | null = null;

  get is_bug2_started(): boolean {
    return this.bug2_started;
  }

  constructor(current_state: ParametersState) {
    this.current_state = current_state;
    this.sensor_ranges = [];
    this.ux = 0;
    this.uy = 0;
    this.utheta = 0;
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

  get plannerRenderData() {
    return {
      roadmap: this.roadmap,
      prmPath: this.prmPath.slice(this.prmWaypointIndex),
      prmSearch: this.prmSearch,
      rrtNodes: this.rrtNodes,
      rrtPath: this.rrtPath,
      rrtWaypointIndex: this.rrtWaypointIndex,
      wavefrontGrid: this.wavefrontGrid,
      wavefrontPath: this.wavefrontPath,
      wavefrontWaypointIndex: this.wavefrontWaypointIndex,
      potentialFieldForces: this.potentialFieldForces,
    };
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

  /**
   * Returns the minimum Euclidean distance from the sensed boundary of the
   * closest obstacle to the goal. Only the contiguous cluster of LiDAR beams
   * around the closest reading is considered; all other obstacles are ignored.
   * If no boundary was sensed, returns Infinity.
   */
  get_min_distance_from_sensed_boundary_to_goal(): number {
    const ranges = this.sensor_ranges;
    if (ranges.length === 0) {
      return Infinity;
    }

    const maxRange = this.current_state.lidar.range;
    const isOutOfRange = (range: number) =>
      !Number.isFinite(range) || range >= maxRange - 1e-9;

    // Find the index of the closest non-max-range reading.
    let minRange = Infinity;
    let minIndex = -1;
    for (let i = 0; i < ranges.length; i += 1) {
      const range = ranges[i];
      if (!isOutOfRange(range) && range < minRange) {
        minRange = range;
        minIndex = i;
      }
    }

    if (minIndex === -1) {
      return Infinity;
    }

    // Collect the contiguous block of finite readings around the closest beam.
    const obstacleIndices: number[] = [minIndex];
    for (let direction of [-1, 1]) {
      for (
        let i = minIndex + direction;
        i >= 0 && i < ranges.length;
        i += direction
      ) {
        if (isOutOfRange(ranges[i])) {
          break;
        }
        obstacleIndices.push(i);
      }
    }

    const robotPose = this.current_state.robot.currentPose;
    const goal = this.current_state.goal;
    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const beamCount = ranges.length;
    const isFullCircle = Math.abs(fieldOfView) >= 2 * Math.PI;
    const angleStep = beamCount > 1
      ? fieldOfView / (isFullCircle ? beamCount : beamCount - 1)
      : 0;
    const startAngle = robotPose.theta - fieldOfView / 2;

    let minDistance = Infinity;
    for (const i of obstacleIndices) {
      const range = ranges[i];
      const angle = startAngle + i * angleStep;
      const boundaryX = robotPose.x + range * Math.cos(angle);
      const boundaryY = robotPose.y + range * Math.sin(angle);
      const distanceToGoal = Math.hypot(
        goal.x - boundaryX,
        goal.y - boundaryY
      );

      if (distanceToGoal < minDistance) {
        minDistance = distanceToGoal;
      }
    }

    return minDistance;
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
    const epsilon = 1e-9;
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
      bx >= Math.min(ax, cx) - epsilon &&
      bx <= Math.max(ax, cx) + epsilon &&
      by >= Math.min(ay, cy) - epsilon &&
      by <= Math.max(ay, cy) + epsilon;

    const orientation1 = orientation(x1, y1, x2, y2, x3, y3);
    const orientation2 = orientation(x1, y1, x2, y2, x4, y4);
    const orientation3 = orientation(x3, y3, x4, y4, x1, y1);
    const orientation4 = orientation(x3, y3, x4, y4, x2, y2);
    const hasOppositeSigns = (first: number, second: number) =>
      (first > epsilon && second < -epsilon) || (first < -epsilon && second > epsilon);

    if (hasOppositeSigns(orientation1, orientation2) && hasOppositeSigns(orientation3, orientation4)) {
      return true;
    }

    return (
      (Math.abs(orientation1) <= epsilon && isOnSegment(x1, y1, x3, y3, x2, y2)) ||
      (Math.abs(orientation2) <= epsilon && isOnSegment(x1, y1, x4, y4, x2, y2)) ||
      (Math.abs(orientation3) <= epsilon && isOnSegment(x3, y3, x1, y1, x4, y4)) ||
      (Math.abs(orientation4) <= epsilon && isOnSegment(x3, y3, x2, y2, x4, y4))
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

  private generatePrmGridPoints(): RoadmapPoint[] {
    // Spacing chosen so a full canvas yields roughly the same point count as PRM's 1000 random samples,
    // and is uniform in x and y so cells are square with no diagonal neighbors.
    const width = this.current_state.simulation.canvasWidth / 100;
    const height = this.current_state.simulation.canvasHeight / 100;
    const spacing = Math.sqrt((width * height) / 1000) || 0.1;
    const columns = Math.max(1, Math.floor(width / spacing));
    const rows = Math.max(1, Math.floor(height / spacing));
    const points: RoadmapPoint[] = [];
    for (let row = 0; row < rows; row += 1) {
      for (let column = 0; column < columns; column += 1) {
        points.push({ x: (column + 0.5) * spacing, y: (row + 0.5) * spacing });
      }
    }
    this.prmGridColumns = columns;
    this.prmGridCellToPointIndex = Array(points.length).fill(-1);
    return points;
  }

  private addRoadmapGridSamples(sampleCount: number) {
    while (sampleCount > 0 && this.prmGridIndex < this.prmGridPoints.length) {
      const cellIndex = this.prmGridIndex;
      const point = this.prmGridPoints[cellIndex];
      this.prmGridIndex += 1;
      const inObstacle = this.current_state.obstacles.some((obstacle) =>
        this.point_in_obstacle(point.x, point.y, obstacle)
      );
      if (!inObstacle) {
        this.roadmap.points.push(point);
        this.prmGridCellToPointIndex[cellIndex] = this.roadmap.points.length - 1;
        sampleCount -= 1;
      }
    }
  }

  private connectPrmGridPoints(connectionCount: number) {
    // Only connect orthogonal grid neighbors (right/down) so the roadmap forms a square grid with no diagonals
    const columns = this.prmGridColumns;
    const totalCells = this.prmGridCellToPointIndex.length;

    while (connectionCount > 0 && this.prmConnectionIndex < totalCells) {
      const cellIndex = this.prmConnectionIndex;
      const pointIndex = this.prmGridCellToPointIndex[cellIndex];
      if (pointIndex !== -1) {
        const column = cellIndex % columns;
        const rightCell = column + 1 < columns ? cellIndex + 1 : -1;
        const downCell = cellIndex + columns < totalCells ? cellIndex + columns : -1;
        for (const neighborCell of [rightCell, downCell]) {
          if (neighborCell === -1) {
            continue;
          }
          const neighborPointIndex = this.prmGridCellToPointIndex[neighborCell];
          if (neighborPointIndex === -1) {
            continue;
          }
          if (this.pathIsClear(this.roadmap.points[pointIndex], this.roadmap.points[neighborPointIndex])) {
            this.roadmap.edges.push({ start: pointIndex, end: neighborPointIndex });
          }
        }
      }
      this.prmConnectionIndex += 1;
      connectionCount -= 1;
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

  calculate_next_step() {
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
        this.d_temp = this.get_min_distance_from_sensed_boundary_to_goal();
        if (this.d_temp < this.d_followed) {
          this.d_followed = this.d_temp;
        }
        switch (this.tangent_state) {
          case "go_to_target":
            // Move towards the goal until a discontinuity is detected
            this.move_towards_goal();
            let discontiuity_lines = this.get_discontinuity_lines();
            if (this.path_to_goal_is_clear()) {
              this.move_towards_goal();
            } else {
              this.follow_wall(this.follow_direction);
            }
            if (discontiuity_lines.length > 0) {
              this.tangent_state = "block_check";
            }
            break;
          case "block_check":
            // Check if robot-goal line intersects with any discontinuity line
            let intersects = false;
            const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
            const beamCount = this.sensor_ranges.length;
            const isFullCircle = Math.abs(fieldOfView) >= 2 * Math.PI;
            const angleStep = beamCount > 1
              ? fieldOfView / (isFullCircle ? beamCount : beamCount - 1)
              : 0;
            const startAngle = this.current_state.robot.currentPose.theta - fieldOfView / 2;
            const pointAtDiscontinuity = (index: number) => {
              const range = this.sensor_ranges[index];
              const angle = startAngle + index * angleStep;
              return {
                x: this.current_state.robot.currentPose.x + range * Math.cos(angle),
                y: this.current_state.robot.currentPose.y + range * Math.sin(angle),
              };
            };
            for (let line of this.get_discontinuity_lines()) {
              const start = pointAtDiscontinuity(line.start);
              const end = pointAtDiscontinuity(line.end);
              if (this.lines_intersect(
                this.current_state.robot.currentPose.x,
                this.current_state.robot.currentPose.y,
                this.current_state.goal.x,
                this.current_state.goal.y,
                start.x,
                start.y,
                end.x,
                end.y
              )) {
                intersects = true;
                break;
              }
            }
            this.tangent_state = intersects ? "chose_side" : "go_to_target";
            break;   
          case "chose_side":
            // Choose which side to follow the discontinuity line based on heuristic distance
            const discontinuityLines = this.get_discontinuity_lines();
            const tangentFieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
            const tangentBeamCount = this.sensor_ranges.length;
            const tangentIsFullCircle = Math.abs(tangentFieldOfView) >= 2 * Math.PI;
            const tangentAngleStep = tangentBeamCount > 1
              ? tangentFieldOfView / (tangentIsFullCircle ? tangentBeamCount : tangentBeamCount - 1)
              : 0;
            const tangentStartAngle = this.current_state.robot.currentPose.theta - tangentFieldOfView / 2;
            const pointAtIndex = (index: number) => {
              const range = this.sensor_ranges[index];
              const angle = tangentStartAngle + index * tangentAngleStep;
              return {
                x: this.current_state.robot.currentPose.x + range * Math.cos(angle),
                y: this.current_state.robot.currentPose.y + range * Math.sin(angle),
              };
            };
            const candidateIndices = discontinuityLines
              .filter((line) => {
                const start = pointAtIndex(line.start);
                const end = pointAtIndex(line.end);
                return this.lines_intersect(
                  this.current_state.robot.currentPose.x,
                  this.current_state.robot.currentPose.y,
                  this.current_state.goal.x,
                  this.current_state.goal.y,
                  start.x,
                  start.y,
                  end.x,
                  end.y
                );
              })
              .flatMap((line) => [line.start, line.end]);

            let bestDiscontinuityIndex: number | null = null;
            let bestHeuristic = Infinity;
            for (const index of candidateIndices) {
              const heuristic = this.calculate_discontinuity_heuristic(index);
              if (heuristic < bestHeuristic) {
                bestHeuristic = heuristic;
                bestDiscontinuityIndex = index;
              }
            }

            if (bestDiscontinuityIndex === null) {
              this.tangent_state = "go_to_target";
              this.followedDiscontinuityPoint = null;
            } else {
              this.follow_direction = this.left_or_right(bestDiscontinuityIndex);
              this.followedDiscontinuityPoint = pointAtIndex(bestDiscontinuityIndex);
              this.tangent_state = "follow_discontinuity";
            }
            break;
          case "follow_discontinuity": 
            const followFieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
            const followBeamCount = this.sensor_ranges.length;
            const followIsFullCircle = Math.abs(followFieldOfView) >= 2 * Math.PI;
            const followAngleStep = followBeamCount > 1
              ? followFieldOfView / (followIsFullCircle ? followBeamCount : followBeamCount - 1)
              : 0;
            const followStartAngle = this.current_state.robot.currentPose.theta - followFieldOfView / 2;
            const pointAtDiscontinuityIndex = (index: number) => {
              const range = Math.min(this.sensor_ranges[index], this.sensor_ranges[index + 1]);
              const angle = followStartAngle + index * followAngleStep;
              return {
                x: this.current_state.robot.currentPose.x + range * Math.cos(angle),
                y: this.current_state.robot.currentPose.y + range * Math.sin(angle),
              };
            };

            const discontinuities = this.get_discontinuities(this.sensor_ranges).filter(
              (index) => index + 1 < this.sensor_ranges.length
            );

            // Stay locked onto the discontinuity we were already following (closest by
            // position) instead of re-picking the globally cheapest one every tick, which
            // caused the robot to keep flip-flopping between two discontinuities.
            let targetDiscontinuityIndex: number | null = null;
            if (this.followedDiscontinuityPoint) {
              let bestDistance = Infinity;
              for (const index of discontinuities) {
                const point = pointAtDiscontinuityIndex(index);
                const distance = Math.hypot(
                  point.x - this.followedDiscontinuityPoint.x,
                  point.y - this.followedDiscontinuityPoint.y
                );
                if (distance < bestDistance) {
                  bestDistance = distance;
                  targetDiscontinuityIndex = index;
                }
              }
            }

            if (targetDiscontinuityIndex === null) {
              let targetHeuristic = Infinity;
              for (const index of discontinuities) {
                const heuristic = this.calculate_discontinuity_heuristic(index);
                if (heuristic < targetHeuristic) {
                  targetHeuristic = heuristic;
                  targetDiscontinuityIndex = index;
                }
              }
            }

            if (targetDiscontinuityIndex === null) {
              this.tangent_state = "go_to_target";
              this.followedDiscontinuityPoint = null;
              this.move_towards_goal();
              break;
            }

            const targetPoint = pointAtDiscontinuityIndex(targetDiscontinuityIndex);
            this.followedDiscontinuityPoint = targetPoint;
            const closestObstacleRange = Math.min(...this.sensor_ranges);
            const obstacleFollowDistance = this.current_state.robot.radius + 0.05;

            if (closestObstacleRange <= obstacleFollowDistance) {
              this.tangent_state = "follow_obstacle";
              this.followedDiscontinuityPoint = null;
            } else {
              this.move_towards_x_y(targetPoint.x, targetPoint.y);
            }
            break;
          case "follow_obstacle":
            // this.d_reach is equal to the distance from the robot to the goal 
             this.d_reach = Math.sqrt(
               Math.pow(this.current_state.goal.x - this.current_state.robot.currentPose.x, 2) +
               Math.pow(this.current_state.goal.y - this.current_state.robot.currentPose.y, 2)
             );
             if (this.d_reach < this.d_followed) {
               this.tangent_state = "go_to_target";
               break;
             }

            this.follow_wall(this.follow_direction);

            break;
          default:
            break;
        }
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

        this.potentialFieldForces = {
          attractiveX: attractive_force_x,
          attractiveY: attractive_force_y,
          repulsiveX: repulsive_force_x,
          repulsiveY: repulsive_force_y,
        };

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
      case "PRM_GRID":
        if (this.map_needs_to_be_built) {
          this.roadmap = this.buildRoadMap();
          this.prmPath = [];
          this.prmWaypointIndex = 0;
          this.prmPhase = "sampling";
          this.prmConnectionIndex = 0;
          this.prmSearch = null;
          if (this.current_state.planner.algorithm === "PRM_GRID") {
            this.prmGridPoints = this.generatePrmGridPoints();
            this.prmGridIndex = 0;
          }
          this.map_needs_to_be_built = false;
        }

        if (this.prmPhase === "sampling") {
          if (this.current_state.planner.algorithm === "PRM_GRID") {
            this.addRoadmapGridSamples(12);
            if (this.prmGridIndex >= this.prmGridPoints.length) {
              this.prmPhase = "connecting";
            }
          } else {
            this.addRoadmapSamples(Math.min(12, 1000 - this.roadmap.points.length));
            if (this.roadmap.points.length === 1000) {
              this.prmPhase = "connecting";
            }
          }
          this.ux = 0;
          this.uy = 0;
        } else if (this.prmPhase === "connecting") {
          if (this.current_state.planner.algorithm === "PRM_GRID") {
            this.connectPrmGridPoints(20);
            if (this.prmConnectionIndex === this.prmGridCellToPointIndex.length) {
              this.prmSearch = this.beginRoadMapSearch(
                this.current_state.robot.currentPose,
                this.current_state.goal
              );
              this.prmPhase = "searching";
            }
          } else {
            this.connectRoadmapPoints(20);
            if (this.prmConnectionIndex === this.roadmap.points.length) {
              this.prmSearch = this.beginRoadMapSearch(
                this.current_state.robot.currentPose,
                this.current_state.goal
              );
              this.prmPhase = "searching";
            }
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
