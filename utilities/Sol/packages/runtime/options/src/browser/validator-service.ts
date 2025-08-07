/*
 * (c) https://github.com/MontiCore/monticore
 */

// tslint:disable:no-any

import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { ValidatorRegistry, ValidatorServer } from "../common";

export const ValidatorService = Symbol("ValidatorService");
export interface ValidatorService {
    validate<V, E>(id: string, type: string, options: V): Promise<E>;
}

@injectable()
export class ValidatorServiceImpl implements ValidatorService {
    @inject(ValidatorRegistry) protected readonly registry: ValidatorRegistry;
    @inject(ValidatorServer) protected readonly backend: ValidatorServer;
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;

    public async validate<V, E>(id: string, type: string, options: V): Promise<E> {
        const workspace = await this.workspace.workspace;
        const context = { workspace: workspace ? workspace.uri : undefined };
        const frontend = this.registry.getValidator(id, type);
        const frontendErrors = frontend ? await frontend.validate(options, context) : {};
        const backendErrors = await this.backend.validate(id, type, options, context);

        this.filterErrors(frontendErrors);
        this.filterErrors(backendErrors);

        return typeof backendErrors === "object" ? { ...backendErrors, ...frontendErrors } : (backendErrors || frontendErrors);
    }

    protected filterErrors(errors: any): void {
        for (const name in errors) {
            if (errors.hasOwnProperty(name) && this.shouldRemoveError(errors[name])) delete errors[name];
        }
    }

    protected shouldRemoveError(error: any): boolean {
        if (error === undefined) return true;
        else if (Array.isArray(error)) return this.shouldRemoveArray(error);
        else if (typeof error === "object") return this.shouldRemoveObject(error);
        else return false;
    }

    protected shouldRemoveArray(errors: any[]): boolean {
        const filteredErrors = errors.filter(error => !this.shouldRemoveError(error));

        return filteredErrors.length === 0;
    }

    protected shouldRemoveObject(errors: any[string]): boolean {
        this.filterErrors(errors);

        const keys = Object.keys(errors);

        return keys.length === 0;
    }
}
