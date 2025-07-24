/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Command } from "@theia/core/lib/common";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { ConfigurationProcessorSelectionService } from "./configuration-processor-selection-service";
import { ConfigurationProcessorWidget } from "./configuration-processor-widget";

import {
    AbstractViewContribution,
    FrontendApplication,
    FrontendApplicationContribution,
    StatusBar,
    StatusBarAlignment
} from "@theia/core/lib/browser";

export namespace ConfigurationProcessorCommands {
    export const TOGGLE_WIDGET: Command = {
        id: "configuration-run-widget.toggle"
    };
}

@injectable()
export class ConfigurationProcessorViewContribution extends AbstractViewContribution<ConfigurationProcessorWidget>
        implements FrontendApplicationContribution {
    @inject(StatusBar) protected readonly statusBar: StatusBar;
    @inject(ConfigurationProcessorSelectionService) protected selection: ConfigurationProcessorSelectionService;

    public constructor() {
        super({
            widgetId: ConfigurationProcessorWidget.ID,
            widgetName: "Run",
            defaultWidgetOptions: {
                area: "bottom",
                rank: 300
            },
            toggleCommandId: ConfigurationProcessorCommands.TOGGLE_WIDGET.id // TODO: Maybe add keybinding?
        });
    }

    @postConstruct()
    protected init(): void {
        this.selection.onSelectionChanged(this.onSelectionChanged);
    }

    public async initializeLayout(app: FrontendApplication): Promise<void> {
        await this.openView();
    }

    public async onStart(): Promise<void> {
        this.statusBar.setElement(ConfigurationProcessorWidget.ID, {
            text: "$(play) Run",
            tooltip: "Run",
            command: this.toggleCommand ? this.toggleCommand.id : undefined,
            priority: 5,
            alignment: StatusBarAlignment.LEFT
        }).catch(() => console.error("'Run' could not be added to the status bar."));
    }

    @bind
    protected async onSelectionChanged(): Promise<void> {
        await this.openView({ activate: true, reveal: true });
    }
}
