/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, named } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { ExecutingConditions } from "./executing-conditions";
import { DefaultExecutingHandler, ExecutingHandler } from "./executing-handler";
import { ContributionProvider } from "@theia/core/lib/common";

export const EXECUTE_ICON_CLASS: string = "emastudio-execute-icon";

export namespace ExecutingCommands {
    export const EXECUTE: Command = {
        id: "emastudio.executing.execute",
        label: "Execute Model",
        iconClass: EXECUTE_ICON_CLASS
    };
}

@injectable()
export class ExecutingCommandContribution implements CommandContribution {
    @inject(ExecutingConditions) protected readonly conditions: ExecutingConditions;
    @inject(DefaultExecutingHandler) protected readonly defaultExecuteHandler: DefaultExecutingHandler;
    @inject(ContributionProvider) @named(ExecutingHandler)
    protected readonly contributions: ContributionProvider<ExecutingHandler>;

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(ExecutingCommands.EXECUTE, {
            execute: this.execute.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async execute(): Promise<void> {
        for (const contribution of this.contributions.getContributions()) {
            if (await contribution.isEnabled()) return contribution.execute();
        }

        return this.defaultExecuteHandler.execute();
    }
}
