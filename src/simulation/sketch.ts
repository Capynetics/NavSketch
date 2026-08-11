import p5 from "p5";
import type { ParametersState } from "../types/parameters";
import { Simulation } from "./simulation";
import { Render } from "./render";

type SimulationRef = {
    current: Simulation | null;
};

type DragCallbacks = {
    onRobotDrag: (x: number, y: number) => void;
    onGoalDrag: (x: number, y: number) => void;
};

const METERS_TO_PIXELS = 100;
const GOAL_HIT_RADIUS_PX = 10;

export function createSketch(
    parent: HTMLElement,
    parameters: ParametersState,
    simulationRef: SimulationRef,
    dragCallbacks: DragCallbacks
) {


    return (p: p5) => {
        
        const simulation = new Simulation(parameters, p);
        simulationRef.current = simulation;

        const render = new Render(simulation);
        let draggingTarget: "robot" | "goal" | null = null;

        const isMouseOverCanvas = () =>
            p.mouseX >= 0 && p.mouseX <= p.width && p.mouseY >= 0 && p.mouseY <= p.height;

        p.setup = () => {
            p.createCanvas(
                parameters.simulation.canvasWidth,
                parameters.simulation.canvasHeight
            ).parent(parent);
            p.frameRate(60);
            p.background(220);
            simulation.sensor_read();
            render.draw(p);
        };

        p.draw = () => {
            p.background(220);
            simulation.sensor_read();
            if (simulation.current_state.simulation.running) {
                simulation.calculate_next_step(p);
                simulation.move_to_next_step();
            }
            render.draw(p);
        };

        p.mousePressed = () => {
            if (!isMouseOverCanvas()) return;

            const state = simulation.current_state;
            const goalX = state.goal.x * METERS_TO_PIXELS;
            const goalY = state.goal.y * METERS_TO_PIXELS;
            const robotX = state.robot.currentPose.x * METERS_TO_PIXELS;
            const robotY = state.robot.currentPose.y * METERS_TO_PIXELS;
            const robotHitRadius = state.robot.radius * METERS_TO_PIXELS;

            if (p.dist(p.mouseX, p.mouseY, goalX, goalY) <= GOAL_HIT_RADIUS_PX) {
                draggingTarget = "goal";
            } else if (p.dist(p.mouseX, p.mouseY, robotX, robotY) <= robotHitRadius) {
                draggingTarget = "robot";
            }
        };

        p.mouseDragged = () => {
            if (!draggingTarget) return;

            const x = p.constrain(p.mouseX, 0, p.width) / METERS_TO_PIXELS;
            const y = p.constrain(p.mouseY, 0, p.height) / METERS_TO_PIXELS;
            const state = simulation.current_state;

            if (draggingTarget === "robot") {
                state.robot.currentPose.x = x;
                state.robot.currentPose.y = y;
                dragCallbacks.onRobotDrag(x, y);
            } else {
                state.goal.x = x;
                state.goal.y = y;
                dragCallbacks.onGoalDrag(x, y);
            }
        };

        p.mouseReleased = () => {
            draggingTarget = null;
        };

        p.windowResized = () => {};
    };
}