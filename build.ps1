if(!(Test-Path -path ".\7ZipPortable.zip")) {
  "Getting 7Zip"
  $url = "https://github.com/EmbeddedMontiArc/EMAStudioBuilder/releases/download/tools/7ZipPortable.zip" #http://github.com/EmbeddedMontiArc/EMAStudioBuilder/releases/download/tools/7ZipPortable.zip"
  $file = ".\7ZipPortable.zip"
  (New-Object System.Net.WebClient).DownloadFile($url, $file)
}


if(!(Test-Path -path ".\7ZipPortable\")) {
  Try {
    "Extracting 7Zip"
    "This needs .NET 4.5 or just extract 7Zip yourself!"
    Add-Type -Path "C:\Program Files (x86)\Reference Assemblies\Microsoft\Framework\.NETFramework\v4.5\System.IO.Compression.FileSystem.dll"
    [System.IO.Compression.ZipFile]::ExtractToDirectory(".\7ZipPortable.zip", ".\")
  } Catch {
    "..."
    ".NET 4.5 not found, unpack 7ZipPortable manually and try again!"
    Exit
  }
}


if(!(Test-Path -path ".\build")) {
  "Clean last build"
  Remove-Item -Recurse -Force ".\build"
}

$dl = ".\download\"       
$dep = ".\dependencies\"       
$dir = $dep        
# download new dependencies if dependencies.txt has changed
if(!(Test-Path -path $dep)) {
  New-Item -ItemType directory -Path $dl
  New-Item -ItemType directory -Path $dep
  Set-Location -Path $dl
        
  "Dependencies have changed. Downloading again."
        
  foreach($line in Get-Content .\dependencies.txt) {
  
      # if line begins with * extract directory
      if($line -match '^\*\s.+'){
         $dir = "..\" + $dep + $line.Remove(0,2)
         New-Item -ItemType Directory -Force -Path $dir
      }
      
      # if line beginns with ** extract link
      if($line -match '^\*\*.+'){
          $link = $line.Remove(0,3)
  
          # if it is a zip download and extract
          if($link -match '[a-zA-Z0-9\-\.]+\.zip'){
            $filename = $matches[0]
            "Downloading " + $filename + " from: " + $link
            (new-object System.Net.WebClient).DownloadFile($link, $filename)
            # appveyor DownloadFile $link -FileName $filename
            "Extracting " + $filename + " to " + $dir
            .\7ZipPortable\App\7-Zip64\7z.exe e $filename -oc:$dir
          }
      }
  } 
  Set-Location -Path "..\"
  
} else {
  "Loading dependencies from cache."
}


"Copy dependencies to build/"
New-Item -ItemType directory -Path ".\build\EMAStudio"
Copy-Item -Recurse -Force ".\dependencies\EmbeddedMontiArcStudio" -Destination ".\build\EMAStudio" 


"Copy project items to build/"
Copy-Item -Recurse -Force ".\EmbeddedMontiArcStudio" -Destination ".\build\EMAStudio" 
Copy-Item -Force ".\ide.bat" -Destination ".\build\EMAStudio\ide.bat" 


"Packing to self-extracting exe"
.\7ZipPortable\App\7-Zip64\7z.exe a EmbeddedMontiArcStudio.exe -sfx -t7z -m0=lzma2 -mx5 ".\build\EMAStudio"
