/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
import URI from "@theia/core/lib/common/uri";
import { UriCommandHandler } from "@theia/core/lib/common/uri-command-handler";
import { inject, injectable } from "inversify";
import { ModulesService } from "./modules-service";

// tslint:disable:no-any

@injectable()
export class MarkAsContentRootHandler implements UriCommandHandler<URI> {
    public execute(uri: URI, ...args: any[]): any {
    }

    public isEnabled(uri: URI, ...args: any[]): boolean {
        return false;
    }

    public isVisible(uri: URI, ...args: any[]): boolean {
        return false;
    }
}

@injectable()
export class UnmarkContentRootHandler implements UriCommandHandler<URI> {
    public execute(uri: URI, ...args: any[]): any {
    }

    public isEnabled(uri: URI, ...args: any[]): boolean {
        return false;
    }

    public isVisible(uri: URI, ...args: any[]): boolean {
        return false;
    }
}

@injectable()
export class MarkAsSourcesRootHandler implements UriCommandHandler<URI> {
    @inject(ModulesService) protected readonly modules: ModulesService;

    public execute(uri: URI, ...args: any[]): any {
    }

    public isEnabled(uri: URI, ...args: any[]): boolean {
        return this.isVisible(uri, ...args);
    }

    public isVisible(uri: URI, ...args: any[]): boolean {
        return false;
    }
}

@injectable()
export class MarkAsTestSourcesRootHandler implements UriCommandHandler<URI> {
    public execute(uri: URI, ...args: any[]): any {
    }

    public isEnabled(uri: URI, ...args: any[]): boolean {
        return false;
    }

    public isVisible(uri: URI, ...args: any[]): boolean {
        return false;
    }
}

@injectable()
export class MarkAsGeneratedSourcesRootHandler implements UriCommandHandler<URI> {
    public execute(uri: URI, ...args: any[]): any {
    }

    public isEnabled(uri: URI, ...args: any[]): boolean {
        return false;
    }

    public isVisible(uri: URI, ...args: any[]): boolean {
        return false;
    }
}

@injectable()
export class UnmarkSourcesRootHandler implements UriCommandHandler<URI> {
    public execute(uri: URI, ...args: any[]): any {
    }

    public isEnabled(uri: URI, ...args: any[]): boolean {
        return false;
    }

    public isVisible(uri: URI, ...args: any[]): boolean {
        return false;
    }
}
