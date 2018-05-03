# download new dependencies 
if(!(Test-Path -path $dl)) {
    New-Item -ItemType directory -Path $dl
}

cd $dl
"Downloading dependencies"
        
foreach($line in Get-Content "..\..\dependencies.txt") {

    # if line beginns with ** extract link
    if($line -match '^\*\*.+'){
        $link = $line.Remove(0,3)

        # extract filename
        if($link -match '[a-zA-Z0-9\-\.]+\.zip'){
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


