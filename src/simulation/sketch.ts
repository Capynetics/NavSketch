import p5 from "p5";
import type { ParametersState } from "../types/parameters";
import { Simulation } from "./simulation";

type SimulationRef = {
    current: Simulation | null;
};

export function createSketch(
    parent: HTMLElement,
    parameters: ParametersState,
    simulationRef: SimulationRef
) {


    return (p: p5) => {
        
        const simulation = new Simulation(parameters);
        simulationRef.current = simulation;

        p.setup = () => {
            p.createCanvas(
                parameters.simulation.canvasWidth,
                parameters.simulation.canvasHeight
            ).parent(parent);
            p.frameRate(60);
            p.background(220);
            simulation.draw(p);
        };

        p.draw = () => {
            if (simulation.current_state.simulation.running) {
                //Simulation.sense_envirement(parameters);
                simulation.calculate_next_step();
                simulation.draw(p);
            }
        };

        p.windowResized = () => {};
    };
}