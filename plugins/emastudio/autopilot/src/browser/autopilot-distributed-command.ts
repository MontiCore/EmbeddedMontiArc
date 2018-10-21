/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandRegistry } from "@theia/core/lib/common";
import { AutoPilotConditions } from "./autopilot-conditions";
import { EXECUTE_DISTRIBUTED_SCRIPT, EXECUTE_DISTRIBUTED_KILL_SCRIPT } from "../common";
import { MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { AutoPilotCommandContribution } from "./autopilot-command-contribution";
import { AutoPilotCommands } from "./autopilot-command-contribution";

@injectable()
export class AutoPilotDistributedCommandContribution extends AutoPilotCommandContribution {
    @inject(AutoPilotConditions) protected readonly conditions: AutoPilotConditions;

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(AutoPilotCommands.EXECUTE_DISTRIBUTED, {
            execute: this.execute.bind(this)
        });

        return this.conditions.checkDistributed(command);
    }

    public async executeKill(): Promise<void> {
        return this.doExecuteKill(EXECUTE_DISTRIBUTED_KILL_SCRIPT);
    }

    protected async executeStart(): Promise<void> {
        return this.doExecuteStart("Simulation (Distributed)", EXECUTE_DISTRIBUTED_SCRIPT);
    }

    protected onStartProcessExit(): void {
        const options = <MiniBrowserProps>{
            startPage: "http://localhost:80/visualization",
            toolbar: "read-only",
            name: "AutoPilot (Distributed)"
        };

        super.onStartProcessExit();
        window.setTimeout(async () => this.openMiniBrowser(options), 10000);
    }
}
