/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FileSystem } from "@theia/filesystem/lib/common";
import { WorkspaceServer } from "@theia/workspace/lib/common";
import URI from "@theia/core/lib/common/uri";
import { FrontendApplicationContribution } from "@theia/core/lib/browser/frontend-application";
import { FrontendApplicationStateService, FrontendApplicationState } from "@theia/core/lib/browser/frontend-application-state";
import { DisposableCollection } from "@theia/core/lib/common";
import { WidgetOpenerOptions } from "@theia/core/lib/browser/widget-open-handler";
import { ApplicationShell, OpenerService, OpenHandler } from "@theia/core/lib/browser";
import WidgetOptions = ApplicationShell.WidgetOptions;
import { WorkspaceInitiatorStateService } from "./workspace-initiator-state";

/**
 * The location of the configs.json in the workspace.
 */
const PATH: string = "/.elysium/workspace-initiator/configs.json";

/**
 * A single entry of the array in configs.json for the WorkspaceInitiator.
 */
export interface WorkspaceInitiatorConfig {
    readonly uri: string;
    readonly opener: string;
    readonly reference?: number;
    readonly options?: WidgetOpenerOptions;
}

/**
 * `FrontendApplicationContribution` which handles the initiation of a newly opened workspace.
 */
@injectable()
export class WorkspaceInitiatorContribution implements FrontendApplicationContribution {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(WorkspaceServer) protected readonly workspace: WorkspaceServer;
    @inject(FrontendApplicationStateService) protected readonly appStateService: FrontendApplicationStateService;
    @inject(WorkspaceInitiatorStateService) protected readonly stateService: WorkspaceInitiatorStateService;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(OpenerService) protected readonly openerService: OpenerService;

    protected readonly toDispose: DisposableCollection = new DisposableCollection();

    public async onStart(): Promise<void> {
        this.toDispose.push(
            this.appStateService.onStateChanged(
                (state: FrontendApplicationState) => this.handleStateChanged(state)
            )
        );
    }

    protected async handleStateChanged(state: FrontendApplicationState): Promise<void> {
        if (state === "ready") {
            this.toDispose.dispose();

            return this.handleReadyState();
        }
    }

    protected async handleReadyState(): Promise<void> {
        await Promise.all([
            this.stateService.getPanelOpened() ? Promise.resolve() : this.openPanel(),
            this.stateService.getFilesOpened() ? Promise.resolve() : this.openFiles()
        ]);
    }

    protected async openFiles(): Promise<void> {
        const root = await this.workspace.getMostRecentlyUsedWorkspace();
        const rootURI = new URI(root);

        if (root) return this.handleRoot(rootURI);
    }

    protected async openPanel(): Promise<void> {
        this.shell.expandPanel("left");

        return this.stateService.setPanelOpened(true);
    }

    protected async handleRoot(rootURI: URI): Promise<void> {
        const servicePathURI = rootURI.resolve(PATH);
        const servicePath = servicePathURI.toString();

        if (await this.fileSystem.exists(servicePath)) return this.handleServicePath(rootURI, servicePathURI);
        else return this.stateService.setFilesOpened(true);
    }

    protected async handleServicePath(rootURI: URI, servicePathURI: URI): Promise<void> {
        const servicePath = servicePathURI.toString();
        const resolving = await this.fileSystem.resolveContent(servicePath);
        const entries: WorkspaceInitiatorConfig[] = JSON.parse(resolving.content);

        return this.handleEntries(rootURI, entries);
    }

    protected async handleEntries(rootURI: URI, entries: WorkspaceInitiatorConfig[]): Promise<void> {
        for (const entry of entries) {
            await this.handleEntry(rootURI, entry);
        }

        return this.stateService.setFilesOpened(true);
    }

    protected async handleEntry(rootURI: URI, entry: WorkspaceInitiatorConfig): Promise<void> {
        const uri = new URI(entry.uri);

        if (uri.scheme === "file") return this.handleFileEntry(rootURI, entry);
        else if (uri.scheme.startsWith("http")) return this.handleHTTPEntry(entry);
    }

    protected normalizeWidgetOpenerOptions(options: WidgetOpenerOptions, reference: number = -1): WidgetOpenerOptions | undefined {
        if (options && options.widgetOptions) options.widgetOptions = this.normalizeWidgetOptions(options.widgetOptions, reference);

        return options;
    }

    protected normalizeWidgetOptions(options: WidgetOptions, reference: number = -1): WidgetOptions {
        const area = options.area;
        const widgets = this.shell.getWidgets(area);
        const offset = reference > 0 ? reference : widgets.length - reference;

        if (offset >= 0) options.ref = widgets[offset];

        return options;
    }

    protected async getOpener(id: string): Promise<OpenHandler | undefined> {
        const openers = await this.openerService.getOpeners();

        for (const opener of openers) {
            if (opener.id === id) return opener;
        }
    }

    protected async handleFileEntry(rootURI: URI, entry: WorkspaceInitiatorConfig): Promise<void> {
        const uri = entry.uri.replace("file://", '');
        const opener = await this.getOpener(entry.opener);
        const targetURI = rootURI.resolve(uri);

        let options = undefined;

        if (entry.options) options = this.normalizeWidgetOpenerOptions(entry.options, entry.reference);
        if (opener) await opener.open(targetURI, options);
    }

    protected async handleHTTPEntry(entry: WorkspaceInitiatorConfig): Promise<void> {
        const uri = new URI(entry.uri);
        const opener = await this.getOpener(entry.opener);

        let options = undefined;

        if (entry.options) options = this.normalizeWidgetOpenerOptions(entry.options, entry.reference);
        if (opener) await opener.open(uri, options);
    }
}
