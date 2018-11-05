/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import {
    CommonFrontendContribution as BaseCommonFrontendContribution,
    ApplicationShell, OpenerService
} from "@theia/core/lib/browser";
import { SelectionService } from "@theia/core/lib/common/selection-service";
import { MessageService } from '@theia/core/lib/common/message-service';
import { AboutDialog } from '@theia/core/lib/browser/about-dialog';
import { MenuModelRegistry } from "@theia/core/lib/common";
import { MAIN_MENU_BAR } from "@theia/core/lib/common";

export namespace CommonMenus {
    export const FEATURES = [...MAIN_MENU_BAR, "5_features"];
}

@injectable()
export class CommonFrontendContribution extends BaseCommonFrontendContribution {
    public constructor(
        @inject(ApplicationShell) protected readonly shell: ApplicationShell,
        @inject(SelectionService) protected readonly selectionService: SelectionService,
        @inject(MessageService) protected readonly messageService: MessageService,
        @inject(OpenerService) protected readonly openerService: OpenerService,
        @inject(AboutDialog) protected readonly aboutDialog: AboutDialog
    ) {
        super(shell, selectionService, messageService, openerService, aboutDialog);
    }

    public registerMenus(registry: MenuModelRegistry): void {
        super.registerMenus(registry);
        registry.registerSubmenu(CommonMenus.FEATURES, "Features");
    }
}
