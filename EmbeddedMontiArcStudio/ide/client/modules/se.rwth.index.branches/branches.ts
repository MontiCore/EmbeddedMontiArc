/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class Branch {

}

export class Branches extends Module {
    protected $panel: JQuery;
    protected $list: JQuery;
    protected $itemBack: JQuery;

    protected constructor() {
        super();
        this.$panel = jQuery("#branches-panel");
        this.$list = jQuery("#branches-list");
        this.$itemBack = jQuery("#branches-back");
    }


    public show(): void {
        this.$panel.show();
        this.emit("show");
    }

    public hide(): void {
        this.$panel.hide();
        this.emit("hide");
    }
}
