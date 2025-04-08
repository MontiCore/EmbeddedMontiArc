import subprocess

def run_git_filter_repo(path = "."):
    try:
        result = subprocess.run(
            ['git', 'filter-repo', '--strip-blobs-bigger-than', '100M', '--force'],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=path
        )
        print("Command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))

def split_large_files(directory, size="100M"):
    try:
        command = f"find {directory} -type f -size +{size} -exec sh -c 'split -b {size} \"$0\" \"$0.part\"' {{}} \\;"
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Split command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))

def concatenate_files(directory):
    try:
        command = f"find {directory} -type f -name '*.part*' | while read part; do base=$(echo \"$part\" | sed 's/.part.*//'); cat \"$part\" >> \"$base\"; rm \"$part\"; done"
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Concatenate command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))

if __name__ == "__main__":
    #run_git_filter_repo()
    #split_large_files("./test")
    concatenate_files("./test")