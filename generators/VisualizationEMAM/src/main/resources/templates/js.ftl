<#-- (c) https://github.com/MontiCore/monticore -->
    class Hash {
        constructor(hash, closeCross, overlay, port) {
            this._parts = hash.substring(1).split('&');

            this.onHashChange = this._onHashChange.bind(this);

            this._visualization = new Visualization(this);
            this._mathview = new MathView(this, closeCross, overlay, port);
        }

        init() {
            this._mathview.reset();
            this._visualization.reset();

            this._mathview._onHashChange();
            this._visualization._onHashChange();
        }

        getPart(index) {
            return this._parts[index];
        }

        update() {
            var visualizationPart = this._visualization.getHashPart();
            var mathviewPart = this._mathview.getHashPart();

            window.location.hash = visualizationPart + (mathviewPart ? '&' + mathviewPart : '');
        }

        set onHashChange(callback) {
            window.addEventListener("hashchange", callback);
        }

        _onHashChange() {
            this._parts = window.location.hash.substring(1).split('&');
        }
    }

    class Visualization {
        constructor(hash) {
            this._hash = hash;
            this._hash.onHashChange = this._onHashChange.bind(this);
            this._element = document.getElementById("visualization");

            this._element.addEventListener("load", this._onLoad.bind(this));
        }

        reset() {
            var part = hash.getPart(0);
            var subParts = part ? part.split(':') : [];

            this.setQualifiedName(subParts[0] || "${helper.getInput()}", false);
            this.setMode(subParts[1] || 1, true);
        }

        setQualifiedName(qualifiedName, pushState) {
            if(this._qualifiedName !== qualifiedName) {
                this._qualifiedName = qualifiedName;

                if (pushState) this._hash.update();
            }
        }

        getQualifiedName() {
            return this._qualifiedName;
        }

        setMode(mode, pushState) {
            if(this._mode !== mode) {
                this._mode = mode;

                if (pushState) this._hash.update();
            }
        }

        getMode() {
            return this._mode;
        }

        getURL() {
            var qualifiedName = this.getQualifiedName();

            return qualifiedName ? "visualization/" + qualifiedName + this.getSuffix() + ".html" : '';
        }

        getSuffix() {
            switch(+this.getMode()) {
                case 0: return "_extended";
                case 1: return '';
                case 2: return "_no_port_names";
                case 3: return "_simplified";
            }
        }

        getHashPart() {
            return this.getQualifiedName() + ':' + this.getMode();
        }

        _overwriteOpen(contentDocument) {
            var childElement = contentDocument.getElementById("svg");
            var childDocument = childElement.contentDocument;
            var childContentWindow = childElement.contentWindow || childDocument.parentWindow || childDocument.defaultView;

            childContentWindow.open = function(url) {
                var qualifiedName = url.replace(/([\w.]+?)(_\w*)*\.html$/, "$1");;

                this.setQualifiedName(qualifiedName, true);
            }.bind(this);
        }

        _overwriteLinks(contentDocument) {
            var anchors = contentDocument.querySelectorAll("a");
            var length = anchors.length;

            for(var i = 0; i < length; i++) {
                var anchor = anchors[i];

                anchor.addEventListener("click", this._createClickHandler(i, anchor));
            }
        }

        _createClickHandler(index, anchor) {
            return function(event) {
                event.preventDefault();

                if(index < 4) {
                    this.setMode(index, true);
                } else {
                    var url = anchor.getAttribute("href");
                    var qualifiedName = url.replace(/([\w.]+?)(_\w*)*\.html$/, "$1");

                    this.setQualifiedName(qualifiedName, true);
                }
            }.bind(this);
        }

        _onHashChange() {
            this.reset();

            var contentWindow = this._element.contentWindow;
            var url = this.getURL();
            var location = contentWindow.location;

            if(!location.href.endsWith(url)) location.replace(url);
        }

        _onLoad() {
            var contentWindow = this._element.contentWindow;
            var contentDocument = contentWindow.document;

            this._overwriteOpen(contentDocument);
            this._overwriteLinks(contentDocument);
        }
    }

    class MathView {
        constructor(hash, closeCross, overlay, port) {
            this._hash = hash;
            this._overlay = overlay;

            this._hash.onHashChange = this._onHashChange.bind(this);
            this._element = document.getElementById("mathviews");

            this._element.addEventListener("load", this._onLoad.bind(this));

            closeCross.onClick = this._onCloseClick.bind(this);
            port.onMessage = this._onMessage.bind(this);
        }

        reset() {
            var part = hash.getPart(1);
            var subParts = part ? part.split(':') : [];

            this.setQualifiedName(subParts[0], false);
            this.setMode(subParts[1] || this._mode || 0, false);
            this.setStart(subParts[2], false);
            this.setEnd(subParts[3], false);
        }

        setQualifiedName(qualifiedName, pushState) {
            if(this._qualifiedName !== qualifiedName) {
                this._qualifiedName = qualifiedName;

                if (pushState) this._hash.update();
            }
        }

        getQualifiedName() {
            return this._qualifiedName;
        }

        setMode(mode, pushState) {
            if(this._mode !== mode) {
                this._mode = mode;

                if (pushState) this._hash.update();
            }
        }

        getMode() {
            return this._mode;
        }

        setStart(start, pushState) {
            if (this._start !== start) {
                this._start = start;

                if (pushState) this._hash.update();
            }
        }

        setEnd(end, pushState) {
            if (this._end !== end) {
                this._end = end;

                if (pushState) this._hash.update();
            }
        }

        getStart() {
            return this._start;
        }

        getEnd() {
            return this._end;
        }

        getURL() {
            var qualifiedName = this.getQualifiedName();
            var start = this.getStart();
            var end = this.getEnd();

            return qualifiedName ? ("mathviews/" + qualifiedName.replace(/\./g, "/") + "/index.html" + '#' + this.getMode() + (start ? ':' + start : '') + (end ? ':' + end : '')) : undefined;
        }

        getHashPart() {
            var qualifiedName = this.getQualifiedName();
            var mode = this.getMode();
            var start = this.getStart();
            var end = this.getEnd();

            return qualifiedName ? (qualifiedName + ':' + mode + (start ? ':' + start : '') + (end ? ':' + end : '')) : undefined;
        }

        _createClickHandler(index) {
            return function(event) {
                this.setMode(index, true);
            }.bind(this);
        }

        _onHashChange() {
            this.reset();

            var url = this.getURL();

            if(url) {
                var contentWindow = this._element.contentWindow;
                var location = contentWindow.location;

                if(!location.href.endsWith(url)) location.replace(url);
                else this._overlay.show();
            } else {
                this._overlay.hide();
            }
        }

        _onCloseClick() {
            this.setQualifiedName(undefined, true);
        }

        _onMessage(message) {
            if (message.source !== window.top) this.setQualifiedName(message.data, true);
        }

        _onLoad() {
            var contentWindow = this._element.contentWindow;
            var contentDocument = contentWindow.document;
            var inputs = contentDocument.querySelectorAll("input");
            var length = inputs.length;

            if (this.getQualifiedName()) this._overlay.show();

            for (var i = 0; i < length; i++) {
                inputs[i].addEventListener("click", this._createClickHandler(i));
            }
        }
    }

    class Overlay {
        constructor(closeCross, themeSwitch) {
            this._element = document.getElementById("overlay");

            closeCross.onClick = this._onCloseClick.bind(this);
            themeSwitch.onClick = this._onThemeSwitchClick.bind(this);
        }

        show() {
            this._element.style = "display:block";
        }

        hide() {
            this._element.style = "display:none";
        }

        _onCloseClick() {
            this.hide();
        }

        _onThemeSwitchClick() {
            var classList = this._element.classList;

            classList.toggle("inverted");
        }
    }

    class CloseCross {
        constructor() {
            this._element = document.getElementById("close");
        }

        set onClick(callback) {
            this._element.addEventListener("click", callback);
        }
    }

    class Dialog {
        constructor(themeSwitch) {
            this._element = document.getElementById("dialog");

            themeSwitch.onClick = this._onThemeSwitchClick.bind(this);
        }

        init() {}

        _onThemeSwitchClick() {
            var classList = this._element.classList;

            classList.toggle("inverted");
        }
    }

    class Port {
        set onMessage(callback) {
            window.top.addEventListener("message", callback);
        }
    }

    class ThemeButton {
		constructor() {
			this._element = document.getElementById("theme-button");

            this.onClick = this._onClick.bind(this);
		}

		set onClick(callback) {
			this._element.addEventListener("click", callback);
		}

		_onClick() {
			var htmls = document.getElementsByTagName("html");
			var classList = htmls[0].classList;

			classList.toggle("constrasted");
		}
	}
