/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Command, CommandRegistry } from "@theia/core/lib/common";
import { TabBarToolbarContribution, TabBarToolbarRegistry } from "@theia/core/lib/browser/shell/tab-bar-toolbar";
import { FileStat } from "@theia/filesystem/lib/common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable, postConstruct } from "inversify";
import { ConfigurationDialogFactory } from "./configuration-dialog";
import { ConfigurationWidget } from "./configuration-widget";

import {
    AbstractViewContribution,
    FrontendApplication,
    FrontendApplicationContribution,
    Widget
} from "@theia/core/lib/browser";

export namespace ConfigurationCommands {
    export const TOGGLE_WIDGET: Command = {
        id: "configuration-widget.toggle"
    };

    export const SHOW_DIALOG: Command = {
        id: "configuration-dialog.show",
        label: "Edit Configurations...",
        iconClass: "fa fa-gear",
        category: "Configurations"
    };

    export const WIDGET_SHOW_DIALOG: Command = {
        id: "configuration-widget.configuration-dialog.show",
        iconClass: "fa fa-gear"
    };
}

@injectable()
export class ConfigurationViewContribution extends AbstractViewContribution<ConfigurationWidget>
        implements FrontendApplicationContribution, TabBarToolbarContribution {
    @inject(ConfigurationDialogFactory) protected readonly dialogFactory: ConfigurationDialogFactory;
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;

    protected roots: FileStat[];

    public constructor() {
        super({
            widgetId: ConfigurationWidget.ID,
            widgetName: "Configurations",
            defaultWidgetOptions: {
                area: "right",
                rank: 600
            },
            toggleCommandId: ConfigurationCommands.TOGGLE_WIDGET.id // TODO: Maybe add keybinding?
        });

        this.roots = [];
    }

    @postConstruct()
    protected init(): void {
        this.workspace.onWorkspaceChanged(roots => this.roots = roots);
    }

    public async initializeLayout(app: FrontendApplication): Promise<void> {
        await this.openView({ activate: false });
    }

    public async registerCommands(registry: CommandRegistry): Promise<void> {
        const execute = () => this.showDialog();
        const isEnabled = () => this.roots.length > 0;

        super.registerCommands(registry);

        registry.registerCommand(ConfigurationCommands.SHOW_DIALOG, {
            isEnabled: isEnabled,
            execute: execute
        });

        registry.registerCommand(ConfigurationCommands.WIDGET_SHOW_DIALOG, {
            isVisible: widget => this.withWidget(widget, isEnabled),
            isEnabled: widget => this.withWidget(widget, isEnabled),
            execute: execute
        });
    }

    public registerToolbarItems(registry: TabBarToolbarRegistry): void {
        registry.registerItem({
            id: ConfigurationCommands.WIDGET_SHOW_DIALOG.id,
            command: ConfigurationCommands.WIDGET_SHOW_DIALOG.id,
            tooltip: "Edit Configurations",
            priority: 0
        });
    }

    protected withWidget<T>(widget: Widget | undefined = this.tryGetWidget(), callback: (widget: ConfigurationWidget) => T): T | false {
        if (widget instanceof ConfigurationWidget && widget.id === ConfigurationWidget.ID) return callback(widget);
        return false;
    }

    protected async showDialog(): Promise<void> {
        const dialog = this.dialogFactory({ title: "Edit Configurations" });

        await dialog.open();
    }
}
