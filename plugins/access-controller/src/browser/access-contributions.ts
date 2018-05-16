/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { AccessContribution } from "./access-controller";
import { FileSystem } from "@theia/filesystem/lib/common";
import { EditorManager } from "@theia/editor/lib/browser";
import URI from "@theia/core/lib/common/uri";
import { WorkspaceServer } from "@theia/workspace/lib/common";

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

/**
 * `AccessContribution` which enables the internal or external creation of URI objects.
 */
@injectable()
export class URIFactoryAccessContribution implements AccessContribution {
    public readonly id: string = "uriFactory";

    public fetch(): object {
        return { "create": (uri: string) => new URI(uri) };
    }
}

/**
 * `AccessContribution` which enables the internal or external access to the `WorkspaceServer`.
 */
@injectable()
export class WorkspaceAccessContribution implements AccessContribution {
    @inject(WorkspaceServer) protected readonly workspace: WorkspaceServer;

    public readonly id: string = "workspace";

    public fetch(): object {
        return this.workspace;
    }
}
