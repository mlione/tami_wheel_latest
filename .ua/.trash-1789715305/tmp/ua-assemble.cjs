#!/usr/bin/env node
const fs = require('fs');

const root = process.argv[2];
const graphPath = `${root}/.ua/intermediate/assembled-graph.json`;
const scan = JSON.parse(fs.readFileSync(`${root}/.ua/intermediate/scan-result.json`, 'utf8'));
const graph = JSON.parse(fs.readFileSync(graphPath, 'utf8'));
const rawLayers = JSON.parse(fs.readFileSync(`${root}/.ua/intermediate/layers.json`, 'utf8'));
const rawTour = JSON.parse(fs.readFileSync(`${root}/.ua/intermediate/tour.json`, 'utf8'));
const layers = Array.isArray(rawLayers) ? rawLayers : rawLayers.layers;
const tour = (Array.isArray(rawTour) ? rawTour : rawTour.steps).sort((a, b) => a.order - b.order);
const nodeIds = new Set(graph.nodes.map((node) => node.id));

for (const layer of layers) {
  for (const field of ['id', 'name', 'description', 'nodeIds']) {
    if (layer[field] === undefined) throw new Error(`Layer missing ${field}`);
  }
  layer.nodeIds = layer.nodeIds.filter((id) => nodeIds.has(id));
}
for (const step of tour) {
  for (const field of ['order', 'title', 'description', 'nodeIds']) {
    if (step[field] === undefined) throw new Error(`Tour step missing ${field}`);
  }
  step.nodeIds = step.nodeIds.filter((id) => nodeIds.has(id));
}

const output = {
  version: '1.0.0',
  project: {
    name: scan.name,
    languages: scan.languages,
    frameworks: scan.frameworks,
    description: scan.description,
    analyzedAt: new Date().toISOString(),
    gitCommitHash: 'cdfa8d21e2e2a07aa99b7bb83d371fb26dc1df51',
  },
  nodes: graph.nodes,
  edges: graph.edges,
  layers,
  tour,
};
fs.writeFileSync(graphPath, `${JSON.stringify(output, null, 2)}\n`);
console.log(`Assembled ${output.nodes.length} nodes, ${output.edges.length} edges, ${layers.length} layers, ${tour.length} tour steps`);
