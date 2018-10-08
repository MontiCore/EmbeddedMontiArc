/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { OCLVerificationConditions } from "./oclverification-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { CHECK_SCRIPT, VISUALIZE_SCRIPT } from "../common";
import { ApplicationShell, Endpoint } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { OCLVERIFICATION_STATIC_PATH } from "../common";
import { EditorManager } from "@theia/editor/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { FileSystem, FileStat } from "@theia/filesystem/lib/common";

import WidgetOptions = ApplicationShell.WidgetOptions;
import URI from "@theia/core/lib/common/uri";

export const OCLVERIFICATION_ICON_CLASS: string = "emastudio-oclverification-icon";
export const OCLVERIFICATION_EYE_ICON_CLASS: string = `${OCLVERIFICATION_ICON_CLASS} emastudio-oclverification-eye`;
export const OCLVERIFICATION_TICK_ICON_CLASS: string = `${OCLVERIFICATION_ICON_CLASS} emastudio-oclverification-tick`;

export namespace OCLVerificationCommands {
    export const CHECK: Command = {
        id: "emastudio.oclverification.check",
        label: "Check OCL Constraint",
        iconClass: OCLVERIFICATION_TICK_ICON_CLASS
    };

    export const VISUALIZE: Command = {
        id: "emastudio.oclverification.visualize",
        label: "Visualize Class Diagram",
        iconClass: OCLVERIFICATION_EYE_ICON_CLASS
    };
}

@injectable()
export class OCLVerificationCommandContribution implements CommandContribution {
    @inject(OCLVerificationConditions) protected readonly conditions: OCLVerificationConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(EditorManager) protected readonly editorManager: EditorManager;
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public async registerCommands(registry: CommandRegistry): Promise<void> {
        await Promise.all([
            this.registerCommandCheck(registry),
            this.registerCommandVisualize(registry)
        ]);
    }

    protected async registerCommandCheck(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(OCLVerificationCommands.CHECK, {
            execute: this.executeCheck.bind(this),
            isEnabled: this.isCheckEnabled.bind(this)
        });

        return this.conditions.checkCheck(command);
    }

    protected async registerCommandVisualize(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(OCLVerificationCommands.VISUALIZE, {
            execute: this.executeVisualize.bind(this),
            isEnabled: this.isVisualizeEnabled.bind(this)
        });

        return this.conditions.checkVisualize(command);
    }

    protected isCheckEnabled(): boolean {
        const currentEditor = this.editorManager.currentEditor;
        const uri = currentEditor ? currentEditor.getResourceUri() : undefined;

        return uri !== undefined && uri.toString(true).endsWith(".ocl");
    }

    protected isVisualizeEnabled(): boolean {
        const currentEditor = this.editorManager.currentEditor;
        const uri = currentEditor ? currentEditor.getResourceUri() : undefined;

        return uri !== undefined && uri.toString(true).endsWith(".cd");
    }

    protected async getCurrentModel(suffix: string): Promise<string> {
        const currentEditor = this.editorManager.currentEditor;
        const uri = currentEditor ? currentEditor.getResourceUri() : undefined;
        const roots = await this.workspaceService.roots;
        const workspace = new URI(roots[0].uri);
        const relativePath = workspace.relative(uri!);
        const qualifiedParts = relativePath!.toString().replace(suffix, '').split('/');
        const length = qualifiedParts.length;
        const modelName = qualifiedParts[length - 1];

        qualifiedParts[length - 1] = modelName.charAt(0).toLowerCase() + modelName.slice(1);

        return qualifiedParts.join('.');
    }

    protected async getContentsOfCurrentModel(): Promise<{ stat: FileStat, content: string} | undefined> {
        const currentEditor = this.editorManager.currentEditor;
        const uri = currentEditor ? currentEditor.getResourceUri() : undefined;

        if (uri) return this.fileSystem.resolveContent(uri.toString());
    }

    protected async executeCheck(): Promise<void> {
        const model = await this.getCurrentModel(".ocl");

        await this.scriptsService.execute({
            label: "OCL Verification",
            plugin: "oclverification",
            script: CHECK_SCRIPT,
            args: [model]
        });
    }

    protected async executeVisualize(): Promise<void> {
        const contents = await this.getContentsOfCurrentModel();
        const content = contents ? contents.content.replace(/(?:\r\n|\r|\n)/g, ' ') : '';

        const process = await this.scriptsService.execute({
            label: "CD Visualization",
            plugin: "oclverification",
            script: VISUALIZE_SCRIPT,
            args: [content]
        });

        if (process) process.onExit(() => this.openMiniBrowser());
    }

    protected async openMiniBrowser(): Promise<void> {
        const endpoint = new Endpoint({ path: `${OCLVERIFICATION_STATIC_PATH}/visualizeCD.html?ide=false` });
        const url = endpoint.getRestUrl().toString(true);
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "CD Visualization", resetBackground: true };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
