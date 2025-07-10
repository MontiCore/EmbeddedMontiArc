FROM node:16.13.2
WORKDIR /app
COPY package*.json ./
RUN npm install
COPY .  .
ENTRYPOINT ["npm", "run", "serve", "--", "--port", "8081"]
