/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { Endpoint } from "@theia/core/lib/browser";
import { SUPERMARIO_STATIC_PATH } from "../common";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { SuperMarioConditions } from "./supermario-conditions";
import { ApplicationShell } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const SUPERMARIO_ICON_CLASS: string = "emastudio-supermario-icon";

export namespace SuperMarioCommands {
    export const PLAY: Command = {
        id: "emastudio.supermario.play",
        label: "Play SuperMario",
        iconClass: SUPERMARIO_ICON_CLASS
    };
}

@injectable()
export class SuperMarioCommandContribution implements CommandContribution {
    @inject(SuperMarioConditions) protected readonly conditions: SuperMarioConditions;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(SuperMarioCommands.PLAY, {
            execute: this.executePlay.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async executePlay(): Promise<void> {
        const endpoint = new Endpoint({ path: SUPERMARIO_STATIC_PATH });
        const url = endpoint.getRestUrl().toString();
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "SuperMario" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "split-bottom" };

        this.shell.addWidget(widget, addOptions);
    }
}
