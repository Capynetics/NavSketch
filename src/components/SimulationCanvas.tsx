import { useEffect, useRef } from "react";
import p5 from "p5";
import { createSketch } from "../simulation/sketch";
import type { Simulation } from "../simulation/simulation";
import type { ParametersState } from "../types/parameters";

type SimulationCanvasProps = {
    parameters: ParametersState;
};

export function SimulationCanvas({ parameters }: SimulationCanvasProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const instanceRef = useRef<p5 | null>(null);
    const simulationRef = useRef<Simulation | null>(null);

    useEffect(() => {
        if (!containerRef.current) return;

        const sketch = createSketch(containerRef.current, parameters, simulationRef);
        const instance = new p5(sketch);
        instanceRef.current = instance;

        return () => {
            instance.remove();
            instanceRef.current = null;
            simulationRef.current = null;
        };
    }, []);

    useEffect(() => {
        if (!simulationRef.current) return;

        simulationRef.current.updateState(parameters);
    }, [parameters]);

    useEffect(() => {
        if (!instanceRef.current) return;

        instanceRef.current.resizeCanvas(
            parameters.simulation.canvasWidth,
            parameters.simulation.canvasHeight
        );
    }, [parameters.simulation.canvasWidth, parameters.simulation.canvasHeight]);

    return (
        <div
            ref={containerRef}
            style={{
                width: `${parameters.simulation.canvasWidth}px`,
                height: `${parameters.simulation.canvasHeight}px`,
            }}
        />
    );
}