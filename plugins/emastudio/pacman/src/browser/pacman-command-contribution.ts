/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { Endpoint } from "@theia/core/lib/browser";
import { PACMAN_STATIC_PATH } from "../common";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { PacManConditions } from "./pacman-conditions";
import { ApplicationShell } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const PACMAN_ICON_CLASS: string = "emastudio-pacman-icon";

export namespace PacManCommands {
    export const PLAY: Command = {
        id: "emastudio.pacman.play",
        label: "Play PacMan",
        iconClass: PACMAN_ICON_CLASS
    };
}

@injectable()
export class PacManCommandContribution implements CommandContribution {
    @inject(PacManConditions) protected readonly conditions: PacManConditions;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(PacManCommands.PLAY, {
            execute: this.executePlay.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async executePlay(): Promise<void> {
        const endpoint = new Endpoint({ path: PACMAN_STATIC_PATH });
        const url = endpoint.getRestUrl().toString();
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "PacMan" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "split-right" };

        this.shell.addWidget(widget, addOptions);
    }
}
