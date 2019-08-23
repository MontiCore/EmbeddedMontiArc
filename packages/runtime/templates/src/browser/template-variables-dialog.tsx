import { ComponentManager } from "@embeddedmontiarc/sol-runtime-components/lib/browser/component-manager";
import { DynamicDialog, DynamicDialogProps } from "@embeddedmontiarc/sol-runtime-components/lib/browser/dynamic-dialog";
import * as ReactDOM from "react-dom";
import * as React from "react";
import Fragment = React.Fragment;

export class TemplateVariablesDialog extends DynamicDialog {
    public constructor(manager: ComponentManager, props: DynamicDialogProps) {
        super(manager, props);

        this.appendAcceptButton("OK");
        this.appendCloseButton("Cancel");
        ReactDOM.render(<Fragment>{this.renderDynamicContent()}</Fragment>, this.contentNode);
    }
}
