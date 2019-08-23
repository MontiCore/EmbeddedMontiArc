/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class GitHub extends Module {
    protected $container: JQuery;

    protected constructor() {
        super();
        this.setHTMLElements();
    }

    protected static instance: GitHub;

    public static async create(): Promise<GitHub> {

    }


    protected setHTMLElements(): void {
        this.$container = jQuery("#github-container");
    }


    public show(): void {
        this.$container.show();
        this.emit("show");
    }

    public hide(): void {
        this.$container.hide();
        this.emit("hide");
    }
}
