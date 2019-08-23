/* (c) https://github.com/MontiCore/monticore */
var Module = {};

$(function() {
    var $spinner = $("#spinner");
    var $result = $("#result");
    var $dropzone = $("#dropzone");

    var loadedImage = undefined;

    function getDataUri(url, callback) {
        var image = new Image();

        image.onload = function() {
            loadedImage = document.createElement('canvas');

            loadedImage.width = this.naturalWidth;
            loadedImage.height = this.naturalHeight;

            loadedImage.getContext('2d').drawImage(this, 0, 0);

            callback(loadedImage.toDataURL('image/jpeg'));
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

    function locateFile(file) {
        if(file.endsWith(".wasm")) return "resources/gen/" + file;
        else return file;
    }

    function clusterImage() {
        var newCanvas2 = document.createElement('canvas');

        newCanvas2.getContext('2d').drawImage(loadedImage,0,0,50,50);

        var pixelData = newCanvas2.getContext('2d').getImageData(0, 0, 50, 50).data;
        var dataForClusterer = new Array(2500);
        var counter = 0;

        for(var i = 0; i < 2500; i++){
            dataForClusterer[i] = new Array(3);
            dataForClusterer[i][0] = pixelData[counter];
            dataForClusterer[i][1] = pixelData[counter + 1];
            dataForClusterer[i][2] = pixelData[counter + 2];
            counter=counter + 4;
        }

        Module.setImgFront(dataForClusterer);
        Module.execute();
    }

    function drawClusteredImage() {
        var result = Module.getClusters();
        var context = $result[0].getContext("2d");
        var newCanvas = document.createElement('canvas');
        var newImgData = context.createImageData(50,50);
        var canvas = $result[0];
        var counter = 0;

        for(var i=0;i<result.length;i++){
            if(result[i][0]==0){
                newImgData.data[counter]=255;
                newImgData.data[counter+1]=0;
                newImgData.data[counter+2]=0;
                newImgData.data[counter+3]=255;
                counter=counter + 4;
            }else if(result[i][0]==1){
                newImgData.data[counter]=0;
                newImgData.data[counter+1]=0;
                newImgData.data[counter+2]=0;
                newImgData.data[counter+3]=255;
                counter=counter + 4;
            }else if(result[i][0]==2){
                newImgData.data[counter]=255;
                newImgData.data[counter+1]=255;
                newImgData.data[counter+2]=255;
                newImgData.data[counter+3]=255;
                counter=counter + 4;
            }else if(result[i][0]==3){
                newImgData.data[counter]=136;
                newImgData.data[counter+1]=136;
                newImgData.data[counter+2]=136;
                newImgData.data[counter+3]=255;
                counter=counter + 4;
            }else if(result[i][0]==4){
                newImgData.data[counter]=77;
                newImgData.data[counter+1]=77;
                newImgData.data[counter+2]=77;
                newImgData.data[counter+3]=255;
                counter=counter + 4;
            }else{
                newImgData.data[counter]=0;
                newImgData.data[counter+1]=0;
                newImgData.data[counter+2]=255;
                newImgData.data[counter+3]=255;
                counter=counter + 4;
            }
        }

        newCanvas.width=newImgData.width;
        newCanvas.height=newImgData.height;
        newCanvas.getContext("2d").putImageData(newImgData, 0, 0);

        context.drawImage(newCanvas,0,0,canvas.width,canvas.height);
    }

    $dropzone.dropzone({
        autoProcessQueue: false,
        url: "/services/clustering/simulate/cluster",
        timeout: 60 * 60 * 1000,
        /*success: function(event, response) {
            $spinner.hide();
            $result.attr("src", "img.bmp?v=" + Date.now());
            this.removeAllFiles();
        },*/
        drop: function(event) {
            var imageUrl = event.dataTransfer.getData('URL');
            var fileName = imageUrl.split('/').pop();
            var that = this;

            $spinner.show();
            event.dataTransfer.effectAllowed = 'copy';

            getDataUri(imageUrl, function(dataUri) {
                var blob = dataURItoBlob(dataUri);

                blob.name = fileName;
                that.addFile(blob);
            });
        },

        addedfile: function(event) {
            var reader = new FileReader();

            reader.onloadend = function(event) {
                var dataURL = event.target.result;

                $spinner.show();

                getDataUri(dataURL, function(dataUri) {
                    clusterImage();
                    drawClusteredImage();
                    $spinner.hide();
                    that.removeAllFiles();
                });
            };

            reader.readAsDataURL(event);
        }
    });


    var isTest = window.location.href.indexOf("test=true") > -1;
    var scriptPath = "resources/gen/";

    if(isTest) scriptPath += "objectdetector1test.js";
    else scriptPath += "objectDetector1.js";

    function onRuntimeInitialized() {
        Module.init();
        $spinner.hide();
    }

    $spinner.show();
    Module.locateFile = locateFile;
    Module.onRuntimeInitialized = onRuntimeInitialized;
    jQuery.getScript(scriptPath);
});
