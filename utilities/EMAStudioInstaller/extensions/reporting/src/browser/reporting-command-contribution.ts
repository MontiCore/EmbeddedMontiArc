/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { ReportingConditions } from "./reporting-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { REPORT_SCRIPT, REPORT_STREAMS_SCRIPT } from "../common";
import { ApplicationShell, Endpoint } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { REPORTING_STATIC_PATH } from "../common";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const REPORTING_ICON_CLASS: string = "emastudio-report-icon";

export namespace ReportingCommands {
    export const REPORT: Command = {
        id: "emastudio.reporting.report",
        label: "Show Report",
        iconClass: REPORTING_ICON_CLASS
    };

    export const REPORT_STREAMS: Command = {
        id: "emastudio.reporting.report.streams",
        label: "Show Report with Streams",
        iconClass: REPORTING_ICON_CLASS
    };
}

@injectable()
export class ReportingCommandContribution implements CommandContribution {
    @inject(ReportingConditions) protected readonly conditions: ReportingConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    public async registerCommands(registry: CommandRegistry): Promise<void> {
        await Promise.all([
            this.registerCommand(registry),
            this.registerCommandStreams(registry)
        ]);
    }

    protected async registerCommand(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(ReportingCommands.REPORT, {
            execute: this.executeReport.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async registerCommandStreams(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(ReportingCommands.REPORT_STREAMS, {
            execute: this.executeReportStreams.bind(this)
        });

        return this.conditions.checkStreams(command);
    }

    protected async executeReport(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Report",
            plugin: "reporting",
            script: REPORT_SCRIPT
        });

        if (process) process.onExit(() => this.openMiniBrowser("report.html?ide=false"));
    }

    protected async executeReportStreams(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Report (Streams)",
            plugin: "reporting",
            script: REPORT_STREAMS_SCRIPT
        });

        if (process) process.onExit(() => this.openMiniBrowser("report.html?ide=false&streams=true"));
    }

    protected async openMiniBrowser(page: string): Promise<void> {
        const endpoint = new Endpoint({ path: `${REPORTING_STATIC_PATH}/${page}` });
        const url = endpoint.getRestUrl().toString(true);
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "Reporting" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "bottom", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
