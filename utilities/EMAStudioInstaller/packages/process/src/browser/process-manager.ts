/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { Process } from "./process";
import { Emitter, Event } from "@theia/core/lib/common";

@injectable()
export class ProcessManager {
    protected processes: Map<number, Process>;

    protected readonly addEmitter: Emitter<number>;
    protected readonly deleteEmitter: Emitter<number>;

    public constructor() {
        this.processes = new Map();
        this.addEmitter = new Emitter<number>();
        this.deleteEmitter = new Emitter<number>();
    }

    public register(process: Process): void {
        this.processes.set(process.id, process);
        this.addEmitter.fire(process.id);
    }

    public async unregister(process: Process): Promise<void> {
        if (this.processes.delete(process.id)) this.deleteEmitter.fire(process.id);
    }

    public getProcess(id: number): Process | undefined {
        return this.processes.get(id);
    }

    public getProcesses(): Process[] {
        return Array.from(this.processes.values());
    }

    public get onAdd(): Event<number> {
        return this.addEmitter.event;
    }

    public get onDelete(): Event<number> {
        return this.deleteEmitter.event;
    }
}
