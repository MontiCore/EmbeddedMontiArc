/**
 * I set up the listeners for dragging and dropping as well
 * as creating an iFrame for holding dragged in images
 * @returns {undefined}
 */
function main() {
    // The div that receives drops and the new iFrame
    var targetDiv = document.getElementById("dropTarget"),
        iframe = document.getElementById("iframeTarget");

    // Set the iframe to a blank page
    iframe.src = "about:blank";

    // Append it to the target
    //document.getElementById("iframeTarget").appendChild(iframe);
    //document.getElementById("iframeTarget").width="1000";
    // Drag over is when an object is hovering over the div
    // e.preventDefault keeps the page from changing
    targetDiv.addEventListener("dragover", function (e) {
        e.preventDefault();
        this.className = "dropMe";
    }, false);

    // Drag leave is when the object leaves the div but isn't dropped
    targetDiv.addEventListener("dragleave", function (e) {
        e.preventDefault();
        this.className = "dropEnabled";
    }, false);

    // Drop is when the click is released
    targetDiv.addEventListener("drop", function (e) {
        e.preventDefault();
        this.className = "dropEnabled";
        loadFile(e.dataTransfer.files[0], iframe);
    }, false);

    document.getElementById("upload").addEventListener("click", function () {
        var file = document.getElementById("browsedFile").files[0];
        loadFile(file, iframe);
    }, false);
}

/**
 * Load a file and then put it on an ifrmae
 * @param {Element} f The file that needs to get loaded
 * @param {Element} destination The iframe that the file is appended to
 * @returns {undefined}
 */
function loadFile(f, destination) {
    // Make a file reader to interpret the file
    var reader = new FileReader();

    // When the reader is done reading,
    // Make a new image tag and append it to the iFrame
    reader.onload = function (event) {
        var oldImageCanvas=document.getElementById('LoadedImage');
        /*if(oldImage!=null){
            destination.removeChild(oldImage);
            console.log("removing old child");
        }*/
        var newImageCanvas = oldImageCanvas;
        var newImage = document.createElement('img');
        newImage.src = event.target.result;
        newImage.id = "CurrentImage";
        if(oldImageCanvas == null){
            newImageCanvas = document.createElement('canvas');
            newImageCanvas.id = "LoadedImage";
            destination.appendChild(newImageCanvas);
        }
        //if(destination.getElementsByName("body")
        /*destination.contentWindow.document.getElementsByTagName("body")[0].appendChild(newImage);
        if(destination.contentWindow.document.getElementsByTagName("body")[0].childElementCount>1){
            destination.contentWindow.document.getElementsByTagName("body")[0].removeChild(destination.contentWindow.document.getElementsByTagName("body")[0].firstChild);
        }*/
        //$('#iframe').height("1000px");
        //$('#iframe').width("1000px");
        console.log("Loaded File");
        var img = newImage;
        var canvas = document.getElementById('ClusterResult');
        if(canvas == null){
            console.log("Creating new Canvas");
        canvas = document.createElement('canvas');
        canvas.id ="ClusterResult";
        console.log(img.width, img.height);
        destination.appendChild(canvas);
        }
        img.addEventListener("load",function(){
                var context=canvas.getContext('2d');
                var contextSource=newImageCanvas.getContext('2d');
                canvas.width = img.width;
                canvas.height = img.height;
                newImageCanvas.width=img.width;
                newImageCanvas.height=img.height;
                context.drawImage(img, 0, 0, 50, 50);
                contextSource.drawImage(img,0,0,img.width,img.height);
                //var pixelData = context.getImageData(0, 0, 1, 1).data;
            
        });
    };

    // Tell the reader to start reading asynchrounously
    reader.readAsDataURL(f);
}

function loadModule(){
    
}

function clusterInWebAssembly(){
    Module.init();
    var out1 = Module.getOut1();
    out1=true;
    Module.setIn1(out1);
    Module.execute();
    var out2 = Module.getOut1();
}

function updateCanvas(){
    var imgData = ctx.createImageData(width, height);
    var i;
    for (i = 0; i < imgData.data.length; i += 4) {
        imgData.data[i+0] = randomInt(0, 255);
        imgData.data[i+1] = randomInt(0, 255);
        imgData.data[i+2] = randomInt(0, 255);
        imgData.data[i+3] = 255;
    }
    ctx.putImageData(imgData, 0, 0);   // set x and y
}
// Run the main script
//window.onload = main;