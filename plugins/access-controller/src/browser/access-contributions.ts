/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { AccessContribution } from "./access-controller";
import { FileSystem } from "@theia/filesystem/lib/common";
import { EditorManager } from "@theia/editor/lib/browser";

/**
 * `AccessContribution` which enables the internal or external use of the FileSystem.
 */
@injectable()
export class FileSystemAccessContribution implements AccessContribution {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public readonly id: string = "filesystem";

    public fetch(): object {
        return this.fileSystem;
    }
}

/**
 * `AccessContribution` which enables the internal or external use of the EditorManager.
 */
@injectable()
export class EditorManagerAccessContribution implements AccessContribution {
    @inject(EditorManager) protected readonly editorManager: EditorManager;

    public readonly id: string = "editorManager";

    public fetch(): object {
        return this.editorManager;
    }
}
