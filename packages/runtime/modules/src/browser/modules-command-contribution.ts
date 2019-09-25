/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Command, CommandContribution, CommandRegistry, SelectionService } from "@theia/core/lib/common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";

export namespace ModulesCommands {
    const MODULES_CATEGORY: string = "Modules";

    export const MARK_AS_SOURCES_ROOT: Command = {
        id: "modules:markAsSourcesRoot",
        category: MODULES_CATEGORY
    };

    export const MARK_AS_TEST_SOURCES_ROOT: Command = {
        id: "modules:markAsTestSourcesRoot",
        category: MODULES_CATEGORY
    };

    export const MARK_AS_GENERATED_SOURCES_ROOT: Command = {
        id: "modules:markAsGeneratedSourcesRoot",
        category: MODULES_CATEGORY
    };

    export const UNMARK_SOURCES_ROOT: Command = {
        id: "modules:unmarkSourcesRoot",
        category: MODULES_CATEGORY
    };
}

@injectable()
export class ModulesCommandContribution implements CommandContribution {
    @inject(SelectionService) protected readonly selection: SelectionService;
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;

    public registerCommands(registry: CommandRegistry): void {
    }
}
