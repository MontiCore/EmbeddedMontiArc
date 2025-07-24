/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { ProcessDescription, ProcessServer } from "../common/process-protocol";
import { Process, ProcessFactory } from "./process";

@injectable()
export class ProcessService {
    @inject(ProcessServer) protected readonly processServer: ProcessServer;
    @inject(ProcessFactory) protected readonly processFactory: ProcessFactory;

    public async spawn(label: string, description: ProcessDescription): Promise<Process | undefined> {
        const id = await this.processServer.spawn(description);

        if (typeof id === "number") return this.processFactory({ id, label });
    }
}
