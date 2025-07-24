# (c) https://github.com/MontiCore/monticore  
$dir = $ext        

if(!(Test-Path -path $ext)) {
  New-Item -ItemType directory -Path $ext
}        

"Extracting depencencies."

foreach($line in Get-Content "..\dependencies.txt") {

    # if line begins with * is extraction directory
    if($line -match '^\*\s.+'){
       $subdir = $line.Remove(0,2)
       $dir = $ext + $subdir
       New-Item -ItemType Directory -Force -Path $dir
    }

    # if line begins with ** extract link
    if($line -match '^\*\*.+'){
        $link = $line.Remove(0,3)

        # if it is a zip extract to dir
        if($link -match '[a-zA-Z0-9\-\._]+\.zip'){
            $filename = $dl + $subdir + $matches[0]

           "Extracting " + $filename + " to " + $dir
           &$7z x -aos $filename $("-o" + $build + $dir) | Out-Null
        }
    }
} 

