/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class User {

}

export class Users extends Module {
    protected $panel: JQuery;
    protected $list: JQuery;

    protected constructor() {
        super();
        this.setHTMLElements();
        this.addEventListeners();
    }

    protected setHTMLElements(): void {
        this.$panel = jQuery("#users-panel");
        this.$list = jQuery("#users-list");
    }

    protected addEventListeners(): void {

    }

    protected static instance: Users;

    public static async create(): Promise<Users> {

    }


    public show(): void {
        this.$panel.show();
        this.emit("show");
    }

    public hide(): void {
        this.$panel.hide();
        this.emit("hide");
    }


    protected onItemClick(): void {

    }
}
