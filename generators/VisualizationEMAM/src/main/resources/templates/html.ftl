<#-- (c) https://github.com/MontiCore/monticore -->
<#assign helper = glex.getGlobalVar("helper")>
<!DOCTYPE HTML>
<html>
<head>
    <title>${helper.getName()}</title>
    <style>
        <!--
        html, body {
            margin:0;
            padding:0;
            height:100%;
            overflow:hidden;
        }

        iframe {
            border:none;
        }

        #visualization {
            height:100%;
            width:100%;
        }

        #overlay {
            background-color:rgba(0,0,0,0.5);
            width:100%;
            height:100%;
            position:fixed;
            display:none;
            top:0;
            left:0;
        }

        #dialog {
            position:fixed;
            top:50%;
            left:50%;
            transform:translate(-50%,-50%);
            width:80%;
            height:80%;
        }

        #mathviews {
            width:100%;
            height:calc(100% - 26px);
        }

        #header {
            background-color:#252525;
            padding-right:5px;
        }

        #close {
            width:100%;
            text-align:right;
            cursor:pointer;
            display:block;
            color:#cecece;
            font-size:20px;
            height:26px;
        }

        #theme-button {
            position:fixed;
            right:27px;
            top:9px;
            background-image: url("data:image/svg+xml;charset=UTF-8,%3c?xml version='1.0'?%3e%3csvg width='50' height='50' xmlns='http://www.w3.org/2000/svg'%3e%3c!-- Created with SVG-edit - http://svg-edit.googlecode.com/ --%3e%3cg%3e%3ctitle%3eLayer 1%3c/title%3e%3ccircle stroke-width='3' stroke='%23000000' fill='none' r='16' cy='25' cx='25'/%3e%3cpath stroke-width='0' fill='black' d='m25,9a16,16 0 0 0 0,32l0,-32z'/%3e%3c/g%3e%3c/svg%3e");
            width:30px;
            height:30px;
            background-color:#cecece;
            background-size:contain;
            opacity:0.2;
        }

        #theme-button:hover {
            opacity:1.0;
            cursor:pointer;
        }

        .constrasted {
            filter:invert(100%) contrast(141%);
        }

        .inverted {
            filter:invert(100%);
        }
        -->
    </style>
</head>
<body>
<iframe id="visualization" src="visualization/${helper.getInput()}.html"></iframe>
<div id="overlay">
    <div id="dialog">
        <div id="header"><span id="close">&#x2716</span></div>
        <iframe id="mathviews" src=""></iframe>
    </div>
</div>
<span id="theme-button" title="Switch Theme"></span>
</body>
<script>
${include("templates/js.ftl")}
</script>
<script>
    var themeButton = new ThemeButton();
    var port = new Port();
    var closeCross = new CloseCross();
    var overlay = new Overlay(closeCross, themeButton);
    var dialog = new Dialog(themeButton);
    var hash = new Hash(window.location.hash, closeCross, overlay, port);


    hash.init();
    dialog.init();
</script>
</html>
