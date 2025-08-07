## Docker commands

- List Images:
  - `docker images`
- Listing dangling images:
  - `docker image list --filter dangling=true`
- Remove image
  - `docker rmi <name/id>`
- Create image from Dockerfile (in folder with Dockerfile)
  - `docker build -t <tag/name> .`
- Login to registry
  - `docker login registry.example.com -u <username> -p <token>`