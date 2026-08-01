import { useEffect, useRef } from "react";
import p5 from "p5";
import { createSketch } from "../simulation/sketch";
import type { ParametersState } from "../types/parameters";

type SimulationCanvasProps = {
    parameters: ParametersState;
};

export function SimulationCanvas({ parameters }: SimulationCanvasProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const instanceRef = useRef<p5 | null>(null);

    useEffect(() => {
        if (!containerRef.current) return;

        const sketch = createSketch(containerRef.current, parameters);
        const instance = new p5(sketch);
        instanceRef.current = instance;

        return () => {
            instance.remove();
            instanceRef.current = null;
        };
    }, []);

    useEffect(() => {
        if (!instanceRef.current) return;

        instanceRef.current.resizeCanvas(
            parameters.simulation.canvasWidth,
            parameters.simulation.canvasHeight
        );
    }, [parameters.simulation.canvasWidth, parameters.simulation.canvasHeight]);

    useEffect(() => {
        if (!import.meta.env.DEV) return;

        const syncWorldJson = async () => {
            try {
                await fetch("/__world-json", {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                    },
                    body: JSON.stringify({
                    parameters,
                    scenarioName: parameters.environment.scenario,
                    }),
                });
            } catch (error) {
                console.error("Failed to sync world.json", error);
            }
        };

        void syncWorldJson();
    }, [parameters]);

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