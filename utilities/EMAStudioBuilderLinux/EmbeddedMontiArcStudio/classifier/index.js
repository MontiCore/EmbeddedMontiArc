/* (c) https://github.com/MontiCore/monticore */
$(function() {
    var $spinner = $("#spinner");
    var $result = document.getElementById("result");

    function getDataUri(url, callback) {
        var image = new Image();

        image.onload = function() {
            var canvas = document.createElement('canvas');

            canvas.width = this.naturalWidth;
            canvas.height = this.naturalHeight;

            canvas.getContext('2d').drawImage(this, 0, 0);

            callback(canvas.toDataURL('image/png'));
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

    $("#dropzone").dropzone({
        url: "/services/classifier/simulate/classify",
        timeout: 60 * 60 * 1000,
        success: function(event, response) {
            $spinner.hide();
            fetch('class.txt')
              .then(response => response.text())
              .then(text => $result.innerHTML="<big><big><big>"+text+"</big></big></big>")
            this.removeAllFiles();
        },
        drop: function(event) {
            var imageUrl = event.dataTransfer.getData('URL');
            var fileName = imageUrl.split('/').pop();
            var that = this;

            $result.innerHTML="";
            $spinner.show();
            event.dataTransfer.effectAllowed = 'copy';

            getDataUri(imageUrl, function(dataUri) {
                var blob = dataURItoBlob(dataUri);

                blob.name = fileName;
                that.addFile(blob);
            });
        }
    });
});
