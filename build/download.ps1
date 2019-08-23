# (c) https://github.com/MontiCore/monticore  
# download new dependencies 
if(!(Test-Path -path $dl)) {
    New-Item -ItemType directory -Path $dl
}

"Downloading dependencies"

foreach($line in Get-Content "..\dependencies.txt") {

    # if line begins with * is extraction directory
    if($line -match '^\*\s.+'){
       cd $build
       $subdir = $line.Remove(0,2)
       $dir = $dl + $subdir
       New-Item -ItemType Directory -Force -Path $dir
       cd $dir
    }

    # if line begins with ** extract link
    if($line -match '^\*\*.+'){
        $link = $line.Remove(0,3)

        # extract filename
        if($link -match '[a-zA-Z0-9\-\._]+\.zip'){
            $filename = $matches[0]
    
            # if file does not exist, download
            if(!(Test-Path -path $filename)) {
                "Downloading " + $filename + " from: " + $link
                &$curl -LJO $link | Out-Null
            }
        }
    }
} 

cd $build


