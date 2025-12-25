# Physical AI and Human-Aided Robotics Book

This is the documentation website for the Physical AI and Human-Aided Robotics curriculum, built with Docusaurus.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GH_TOKEN=<GITHUB_TOKEN> npm run deploy
```

For more details on deployment, see the [Docusaurus documentation](https://docusaurus.io/docs/deployment).