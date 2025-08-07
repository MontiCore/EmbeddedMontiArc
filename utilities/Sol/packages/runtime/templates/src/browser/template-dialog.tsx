/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionDialog } from "@embeddedmontiarc/sol-runtime-options/lib/browser/option-dialog";
import { OptionType } from "@embeddedmontiarc/sol-runtime-options/src/common";
import { DialogError, DialogMode, DialogProps } from "@theia/core/lib/browser";
import { inject, injectable } from "inversify";
import { TEMPLATE_VALIDATOR_TYPE } from "../common";

import * as React from "react";

import ReactNode = React.ReactNode;

export const TemplateDialogProps = Symbol("TemplateDialogProps");
export interface TemplateDialogProps extends DialogProps {
    readonly id: string;
    readonly options: OptionType[];
}

export const TemplateDialogFactory = Symbol("TemplateDialogFactory");
export interface TemplateDialogFactory {
    (props: TemplateDialogProps): TemplateDialog;
}

@injectable()
export class TemplateDialog<E = any, V = any[string]> extends OptionDialog<V> { // tslint:disable-line:no-any
    protected errors: E;

    public constructor(@inject(TemplateDialogProps) protected readonly props: TemplateDialogProps) {
        super(props);

        this.errors = {} as E;

        this.appendAcceptButton("OK");
        this.appendCloseButton("Cancel");
    }

    public get value(): V {
        return this.options;
    }

    protected async isValid(value: V, mode: DialogMode): Promise<DialogError> {
        this.errors = await this.validator.validate(this.props.id, TEMPLATE_VALIDATOR_TYPE, value);

        const keys = Object.keys(this.errors);

        return keys.length > 0 ? "There are erroneous form fields." : '';
    }

    protected render(): ReactNode {
        return this.renderDynamicContent(this.props.id, this.props.options, this.errors, this.options);
    }
}
