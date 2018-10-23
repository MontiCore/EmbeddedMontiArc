/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, postConstruct } from "inversify";
import { StorageService } from "@theia/core/lib/browser";

/**
 * The key for the storage of the state.
 */
const STORAGE_KEY: string = "workspace-initiator";

/**
 * The state of the workspace-initiator.
 */
export interface WorkspaceInitiatorState {
    panelOpened: boolean;
    filesOpened: boolean;
}

/**
 * Default state for the case that the workspace has not been opened yet.
 */
export const defaultWorkspaceInitiatorState: WorkspaceInitiatorState = {
    panelOpened: false,
    filesOpened: false
};

/**
 * Class which handles loading and saving of workspace-initiator state information.
 */
@injectable()
export class WorkspaceInitiatorStateService {
    @inject(StorageService) protected readonly storageService: StorageService;

    protected state: WorkspaceInitiatorState;

    @postConstruct()
    protected async init(): Promise<void> {
        this.state = await this.loadState();
    }

    protected loadState(): Promise<WorkspaceInitiatorState> {
        return this.storageService.getData<WorkspaceInitiatorState>(STORAGE_KEY, defaultWorkspaceInitiatorState);
    }

    protected saveState(): Promise<void> {
        return this.storageService.setData<WorkspaceInitiatorState>(STORAGE_KEY, this.state);
    }

    public getPanelOpened(): boolean {
        return this.state.panelOpened;
    }

    public setPanelOpened(toggle: boolean): Promise<void> {
        this.state.panelOpened = toggle;

        return this.saveState();
    }

    public getFilesOpened(): boolean {
        return this.state.filesOpened;
    }

    public setFilesOpened(toggle: boolean): Promise<void> {
        this.state.filesOpened = toggle;

        return this.saveState();
    }
}
