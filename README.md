# data-consumer



## Starting the application in Docker

To start the application, first set a symlink to the production environment file.
```
ln -s .env.production .env
```

After this
ln -s .env.production .env
ln -s .env.development .env

## Starting the application in local mode for development

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/catena-x/data-consumer.git
git branch -M main
git push -uf origin main
```
