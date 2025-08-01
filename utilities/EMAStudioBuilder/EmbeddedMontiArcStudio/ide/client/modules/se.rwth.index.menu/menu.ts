/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.module/module";

export class Menu extends Module {
    protected $panel: JQuery;
    protected $buttonFromGitHub: JQuery;
    protected $buttonDemo: JQuery;
    protected $buttonCancel: JQuery;

    protected constructor() {
        super();
        this.$panel = jQuery("#menu-panel");
        this.$buttonFromGitHub = jQuery("#menu-item-from-github");
        this.$buttonDemo = jQuery("#menu-item-demo");
        this.$buttonCancel = jQuery("#menu-item-cancel");
    }

    public static async create(): Promise<Menu> {

    }


    public show(): void {
        this.$panel.slideDown();
        this.emit("show");
    }

    public hide(): void {
        this.$panel.slideUp();
        this.emit("hide");
    }


    private onNewProjectClick(): void {

    }

    private onDemoClick(): void {

    }

    private onCancelClick(): void {

    }
}
