/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import {
    AbstractViewContribution,
    ApplicationShell,
    FrontendApplication,
    FrontendApplicationContribution,
    StatusBar,
    StatusBarAlignment
} from "@theia/core/lib/browser";
import { ProcessWidget } from "./process-widget";

import Area = ApplicationShell.Area;

@injectable()
export class ProcessViewContribution extends AbstractViewContribution<ProcessWidget>
        implements FrontendApplicationContribution {
    @inject(StatusBar) protected readonly statusBar: StatusBar;

    public constructor() {
        super({
            widgetId: ProcessWidget.WIDGET_ID,
            widgetName: "Processes",
            defaultWidgetOptions: {
                area: "bottom" as Area
            },
            toggleCommandId: `${ProcessWidget.WIDGET_ID}:toggle`
        });
    }

    public async initializeLayout(app: FrontendApplication): Promise<void> {
        await this.openView();
    }

    public onStart(application: FrontendApplication): void {
        this.statusBar.setElement(ProcessViewContribution.STATUS_BAR_ID, {
            text: "$(flag) Processes",
            alignment: StatusBarAlignment.LEFT,
            priority: 9,
            command: this.toggleCommand ? this.toggleCommand.id : undefined
        });
    }
}

export namespace ProcessViewContribution {
    export const STATUS_BAR_ID: string = "process-widget-status";
}
