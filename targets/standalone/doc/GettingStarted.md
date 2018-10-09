# Getting Started

## Local Execution
Transferring the output files to the server responsible for their distribution could become
quite inefficient when done after each implementation phase in order to test the usability of
new features. For this reason, this project is shipped with its own server implementation
based on [**Express**](http://expressjs.com) in order to locally test the functionality.
Starting this server is performed with the following console command executed in the root
directory:

```bash
yarn run start
```

The port used for the server defaults to `3005` and the implementation can be visited on
[http://localhost:3005](http://localhost:3005) with any of the
[supported browsers](../README.md#browser-support).