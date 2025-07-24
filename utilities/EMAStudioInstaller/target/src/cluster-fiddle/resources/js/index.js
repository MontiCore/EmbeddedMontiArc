$(function() {
    var $spinner = $("#spinner");
    var $result = $("#result");

    var sendMessage = function(message) {};
    var onMessage = function(callback) {};

    if (window.top.process) {
        var electron = window.top.require("electron");
        var ipcRenderer = electron.ipcRenderer;

        onMessage = function(callback) {
            ipcRenderer.on("relay", function(event, message) { console.log(message); callback(message); });
        };

        sendMessage = function(message) {
            ipcRenderer.send("relay", message);
        };
    } else {
        onMessage = function(callback) {
            window.top.addEventListener("message", function(event) { callback(event.data); });
        };

        sendMessage = function(message) {
            window.top.postMessage(message, '*');
        };
    }

    function getDataUri(url, callback) {
        var image = new Image();

        image.onload = function() {
            var canvas = document.createElement('canvas');

            canvas.width = this.naturalWidth;
            canvas.height = this.naturalHeight;

            canvas.getContext('2d').drawImage(this, 0, 0);

            callback(canvas.toDataURL('image/jpeg'));
        };

        image.setAttribute('crossOrigin', 'anonymous');
        image.src = url;
    }

    function dataURItoBlob(dataURI) {
        var content = [];
        var byteString = "";

        if(dataURI.split(',')[0].indexOf('base64') !== -1) byteString = atob(dataURI.split(',')[1]);
        else byteString = decodeURI(dataURI.split(',')[1]);

        var mimestring = dataURI.split(',')[0].split(':')[1].split(';')[0];

        for(var i = 0; i < byteString.length; i++) {
            content[i] = byteString.charCodeAt(i)
        }

        return new Blob([new Uint8Array(content)], {
            type: mimestring
        });
    }

    var dropzone = null;
    var isSender = false;
    $("#dropzone").dropzone({
        init: function() {
            dropzone = this;
        },
        url: "/services/emastudio-clustering",
        success: function() {
            if (isSender) sendMessage({ path: "/services/emastudio-clustering", type: "Cluster" }, '*');
            isSender = false;
        },
        drop: function(event) {
            var imageUrl = event.dataTransfer.getData('URL');

            isSender = true;
            event.dataTransfer.effectAllowed = 'copy';

            sendMessage({ path: "/services/emastudio-clustering", type: "Drop", message: imageUrl });
        }
    });

    onMessage(function(data) {
        if (data.path !== "/services/emastudio-clustering") return;

        if (data.type === "Drop") {
            $result.attr("src", "");
            $spinner.show();

            getDataUri(data.message, function(dataUri) {
                var blob = dataURItoBlob(dataUri);

                blob.name = data.message.split('/').pop();
                Dropzone.forElement("#dropzone").addFile(blob);
            });
        } else if (data.type === "Clustered") {
            $spinner.hide();
            $result.attr("src", "img.bmp?v=" + Date.now());
            Dropzone.forElement("#dropzone").removeAllFiles(true);
        }
    });
});