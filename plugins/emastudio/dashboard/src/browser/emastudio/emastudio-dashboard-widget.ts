/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable, postConstruct } from "inversify";
import { FileSystemDashboardWidget } from "@elysium/dashboard/lib/browser/filesystem";
import { h } from "@phosphor/virtualdom";
import { EMAStudioDashboardModel } from "./emastudio-dashboard-model";
import { DashboardItem, SelectableDashboardItem } from "@elysium/dashboard/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { Widget } from "@theia/core/lib/browser";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";

@injectable()
export class EMAStudioDashboardWidget extends FileSystemDashboardWidget {
    @inject(EMAStudioDashboardModel) public readonly model: EMAStudioDashboardModel;
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(FrontendApplicationStateService) protected readonly stateService: FrontendApplicationStateService;

    @postConstruct()
    protected async init(): Promise<void> {
        super.init();

        await this.stateService.reachedState("initialized_layout");

        const roots = await this.workspaceService.roots;
        const stat = roots[0];

        if (!stat) Widget.attach(this, document.body);
    }

    protected renderControlItem(): h.Child {
        // tslint:disable-next-line:no-null-keyword
        return null;
    }

    protected async handleItemClickEvent(event: MouseEvent, item: DashboardItem): Promise<void> {
        // await super.handleItemClickEvent(event, item);
        this.model.selectItem(<Readonly<SelectableDashboardItem>>item);
        if (this.props.itemContextMenu) this.contextMenuRenderer.render(this.props.itemContextMenu, event);
    }
}
