/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FileNavigatorWidget } from "@theia/navigator/lib/browser";
import { ContextMenuRenderer, TreeProps } from '@theia/core/lib/browser';
import { FileNavigatorModel } from "@theia/navigator/lib/browser";
import { CommandService } from "@theia/core/lib/common/command";
import { SelectionService } from "@theia/core/lib/common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { LabelProvider } from "@theia/core/lib/browser/label-provider";
import { h } from "@phosphor/virtualdom/lib";
import { BrowserWorkspaceCommands } from "@elysium/workspace/lib/browser";
import { SearchBoxFactory } from "@theia/navigator/lib/browser/search-box";
import { FileNavigatorSearch } from "@theia/navigator/lib/browser/navigator-search";

@injectable()
export class BrowserFileNavigatorWidget extends FileNavigatorWidget {
    public constructor(
        @inject(TreeProps) readonly props: TreeProps,
        @inject(FileNavigatorModel) readonly model: FileNavigatorModel,
        @inject(ContextMenuRenderer) contextMenuRenderer: ContextMenuRenderer,
        @inject(CommandService) protected readonly commandService: CommandService,
        @inject(SelectionService) protected readonly selectionService: SelectionService,
        @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService,
        @inject(LabelProvider) protected readonly labelProvider: LabelProvider,
        @inject(FileNavigatorSearch) protected readonly navigatorSearch: FileNavigatorSearch,
        @inject(SearchBoxFactory) protected readonly searchBoxFactory: SearchBoxFactory
    ) {
        super(props, model, contextMenuRenderer, commandService, selectionService,
            workspaceService, labelProvider, navigatorSearch, searchBoxFactory);
    }

    protected renderOpenWorkspaceDiv(): h.Child {
        const newButton = h.button({
            className: "open-workspace-button",
            title: "Create a new workspace",
            onclick: e => this.commandService.executeCommand(BrowserWorkspaceCommands.OPEN_DASHBOARD.id)
        }, "Open Dashboard");

        const newButtonContainer = h.div({ className: "open-workspace-button-container" }, newButton);

        return h.div({ className: "theia-navigator-container" }, newButtonContainer);
    }
}
