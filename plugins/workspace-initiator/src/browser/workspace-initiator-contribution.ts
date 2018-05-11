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
import { StorageService, ApplicationShell, OpenerService, OpenHandler } from "@theia/core/lib/browser";
import WidgetOptions = ApplicationShell.WidgetOptions;

/**
 * The location of the configs.json in the workspace.
 */
export const PATH: string = "/.elysium/workspace-initiator/configs.json";

/**
 * The key for the storage of the flags.
 */
export const STORAGE_KEY: string = "workspace-initiator";

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
 * The flags representing whether a certain initiation step has already been done.
 */
export interface WorkspaceInitiatorFlags {
    readonly panelOpened: boolean;
    readonly filesOpened: boolean;
}

/**
 * Default flags for the case that the workspace has not been opened yet.
 */
export const defaultWorkspaceInitiatorFlags: WorkspaceInitiatorFlags = {
    panelOpened: false,
    filesOpened: false
};

/**
 * FrontendApplicationContribution which handles the initiation of a newly opened workspace.
 */
@injectable()
export class WorkspaceInitiatorContribution implements FrontendApplicationContribution {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(WorkspaceServer) protected readonly workspace: WorkspaceServer;
    @inject(FrontendApplicationStateService) protected readonly stateService: FrontendApplicationStateService;
    @inject(StorageService) protected readonly storageService: StorageService;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(OpenerService) protected readonly openerService: OpenerService;

    protected readonly toDispose: DisposableCollection = new DisposableCollection();

    public async onStart(): Promise<void> {
        this.toDispose.push(
            this.stateService.onStateChanged(
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

    protected async getData(): Promise<WorkspaceInitiatorFlags> {
        return this.storageService.getData<WorkspaceInitiatorFlags>(STORAGE_KEY, defaultWorkspaceInitiatorFlags);
    }

    protected async setData(data: WorkspaceInitiatorFlags): Promise<void> {
        return this.storageService.setData<WorkspaceInitiatorFlags>(STORAGE_KEY, data);
    }

    protected async handleReadyState(): Promise<void> {
        const flags = await this.getData();

        await Promise.all([
            flags.panelOpened ? Promise.resolve() : this.openPanel(),
            flags.filesOpened ? Promise.resolve() : this.openFiles()
        ]);
    }

    protected async openFiles(): Promise<void> {
        const root = await this.workspace.getRoot();
        const rootURI = new URI(root);

        if (root) return this.handleRoot(rootURI);
    }

    protected async openPanel(): Promise<void> {
        const data = { panelOpened: true };
        const currentData = await this.getData();
        const newData = Object.assign(currentData, data);

        this.shell.expandPanel("left");

        return this.setData(newData);
    }

    protected async handleRoot(rootURI: URI): Promise<void> {
        const servicePathURI = rootURI.resolve(PATH);
        const servicePath = servicePathURI.toString();

        if (await this.fileSystem.exists(servicePath)) {
            return this.handleServicePath(rootURI, servicePathURI);
        } else {
            const data = { filesOpened: true };
            const currentData = await this.getData();
            const newData: WorkspaceInitiatorFlags = Object.assign(currentData, data);

            return this.setData(newData);
        }
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

        const data = { filesOpened: true };
        const currentData = await this.getData();
        const newData: WorkspaceInitiatorFlags = Object.assign(currentData, data);

        return this.setData(newData);
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
        const uri = new URI(entry.uri);
        const opener = await this.getOpener(entry.opener);
        const targetURI = rootURI.resolve(uri.path);

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
