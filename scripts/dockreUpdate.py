import subprocess
import sys

def pull_image(image_name, tag):
    command = f"docker pull {image_name}:{tag}"
    result = subprocess.run(command.split(), capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
        sys.exit(1)
    print(result.stdout)
    print(f"Successfully pulled {image_name}:{tag}")

def get_base_image(image_name, tag):
    command = f"docker history {image_name}:{tag} --no-trunc"
    result = subprocess.run(command.split(), capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
        sys.exit(1)

    lines = result.stdout.splitlines()
    for line in lines:
        print(lines)
        if "FROM" in line:
            parts = line.split()
            for part in parts:
                if part.startswith("FROM"):
                    return part.split("FROM")[1]
    return None

def create_new_dockerfile(base_image, original_image, new_base_image):
    dockerfile_content = f"""
    FROM {new_base_image}
    COPY --from={original_image} / /
    """
    with open("Dockerfile", "w") as file:
        file.write(dockerfile_content)

def build_new_image(new_image_name):
    command = f"docker build -t {new_image_name} ."
    result = subprocess.run(command.split(), capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
        sys.exit(1)
    print(result.stdout)
    print(f"Successfully built {new_image_name}")

def main():
    if len(sys.argv) != 4:
        print("Usage: python update_base_image.py <original_image> <tag> <new_base_image>")
        sys.exit(1)

    original_image = sys.argv[1]
    tag = sys.argv[2]
    new_base_image = sys.argv[3]

    pull_image(original_image, tag)
    base_image = get_base_image(original_image, tag)
    if not base_image:
        print("Could not determine the base image.")
        sys.exit(1)
    print(base_image)
    create_new_dockerfile(base_image, f"{original_image}:{tag}", str(input("New base image: ")))
    build_new_image(f"{original_image}_updated")

if __name__ == "__main__":
    main()