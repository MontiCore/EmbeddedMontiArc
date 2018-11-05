/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { EMAM2WASMConditions } from "./emam2wasm-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { EMAM2WASM_SCRIPT } from "../common";

export const EMAM2WASM_ICON_CLASS: string = "emastudio-emam2wasm-icon";

export namespace EMAM2WASMCommands {
    export const GENERATE: Command = {
        id: "emastudio.emam2wasm.generate",
        label: "Generate WebAssembly",
        iconClass: EMAM2WASM_ICON_CLASS
    };
}

@injectable()
export class EMAM2WASMCommandContribution implements CommandContribution {
    @inject(EMAM2WASMConditions) protected readonly conditions: EMAM2WASMConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(EMAM2WASMCommands.GENERATE, {
            execute: this.executeEMAM2WASM.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async executeEMAM2WASM(): Promise<void> {
        await this.scriptsService.execute({
            label: "EMAM2WASM",
            plugin: "emam2wasm",
            script: EMAM2WASM_SCRIPT
        });
    }
}
