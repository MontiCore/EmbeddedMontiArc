/*
 * (c) https://github.com/MontiCore/monticore
 */
import { AbstractDialog } from "@theia/core/lib/browser";
import { inject, injectable } from "inversify";
import * as ReactDOM from "react-dom";
import { OptionType } from "../common";
import { OptionManager } from "./option-manager";
import { GroupOption } from "./group-option";

import * as React from "react";
import { ValidatorService } from "./validator-service";

import Fragment = React.Fragment;
import ReactNode = React.ReactNode;

/**
 * Abstract implementation of an option dialog.
 */
@injectable()
export abstract class OptionDialog<R, E = any, V = any> extends AbstractDialog<R> { // tslint:disable-line:no-any
    @inject(OptionManager) protected readonly omanager: OptionManager;
    @inject(ValidatorService) protected readonly validator: ValidatorService;

    protected options: V;

    public async update(): Promise<void> {
        await this.validate();
        ReactDOM.render(<Fragment>{this.render()}</Fragment>, this.contentNode);
    }

    protected async setOptions(options: V): Promise<void> {
        this.options = options;

        return this.update();
    }

    protected abstract render(): ReactNode;

    protected renderDynamicContent(key: string, options: OptionType[], errors: E, values?: R): ReactNode {
        return <div className="sol-runtime-options content">
            <GroupOption key={key} value={values} error={errors} manager={this.omanager} options={options}
                         setParentValue={value => this.setOptions(value)}/>
        </div>;
    }
}
