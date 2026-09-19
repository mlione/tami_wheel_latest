#!/usr/bin/env node

const fs = require('fs');

function fail(message) {
  process.stderr.write(`ua-tour-analyze: ${message}\n`);
  process.exit(1);
}

try {
  const [inputPath, outputPath] = process.argv.slice(2);
  if (!inputPath || !outputPath) fail('usage: node ua-tour-analyze.js INPUT OUTPUT');

  const input = JSON.parse(fs.readFileSync(inputPath, 'utf8'));
  const nodes = Array.isArray(input.nodes) ? input.nodes : [];
  const edges = Array.isArray(input.edges) ? input.edges : [];
  const layers = Array.isArray(input.layers) ? input.layers : [];
  const nodeById = new Map(nodes.map((node) => [node.id, node]));
  const fanIn = new Map(nodes.map((node) => [node.id, 0]));
  const fanOut = new Map(nodes.map((node) => [node.id, 0]));

  for (const edge of edges) {
    if (!nodeById.has(edge.source) || !nodeById.has(edge.target)) continue;
    fanOut.set(edge.source, fanOut.get(edge.source) + 1);
    fanIn.set(edge.target, fanIn.get(edge.target) + 1);
  }

  const ranking = (counts, field) => nodes
    .map((node) => ({ id: node.id, [field]: counts.get(node.id), name: node.name }))
    .sort((a, b) => b[field] - a[field] || a.id.localeCompare(b.id))
    .slice(0, 20);

  const sortedOut = [...fanOut.values()].sort((a, b) => b - a);
  const sortedIn = [...fanIn.values()].sort((a, b) => a - b);
  const top10Threshold = sortedOut[Math.max(0, Math.ceil(sortedOut.length * 0.10) - 1)] ?? 0;
  const bottom25Threshold = sortedIn[Math.max(0, Math.ceil(sortedIn.length * 0.25) - 1)] ?? 0;
  const entryNames = new Set([
    'index.ts', 'index.js', 'main.ts', 'main.js', 'app.ts', 'app.js', 'server.ts',
    'server.js', 'mod.rs', 'main.go', 'main.py', 'main.rs', 'manage.py', 'app.py',
    'wsgi.py', 'asgi.py', 'run.py', '__main__.py', 'Application.java', 'Main.java',
    'Program.cs', 'config.ru', 'index.php', 'App.swift', 'Application.kt', 'main.cpp', 'main.c'
  ]);

  const entryPointCandidates = nodes.map((node) => {
    let score = 0;
    const path = node.filePath || '';
    const depth = path.split('/').filter(Boolean).length;
    if (node.type === 'file') {
      if (entryNames.has(node.name)) score += 3;
      if (depth <= 2) score += 1;
      if ((fanOut.get(node.id) || 0) >= top10Threshold) score += 1;
      if ((fanIn.get(node.id) || 0) <= bottom25Threshold) score += 1;
    } else if (node.type === 'document') {
      if (path === 'README.md') score += 5;
      else if (depth === 1 && path.endsWith('.md')) score += 2;
    }
    return { id: node.id, score, name: node.name, summary: node.summary || '' };
  }).filter((item) => item.score > 0)
    .sort((a, b) => b.score - a.score || a.id.localeCompare(b.id))
    .slice(0, 5);

  const topCodeEntry = entryPointCandidates.find((candidate) => nodeById.get(candidate.id)?.type === 'file');
  const bfsTraversal = { startNode: topCodeEntry?.id || null, order: [], depthMap: {}, byDepth: {} };
  if (topCodeEntry) {
    const queue = [topCodeEntry.id];
    bfsTraversal.depthMap[topCodeEntry.id] = 0;
    const traversable = new Map();
    for (const edge of edges) {
      if (!['imports', 'calls'].includes(edge.type)) continue;
      if (!nodeById.has(edge.source) || !nodeById.has(edge.target)) continue;
      if (!traversable.has(edge.source)) traversable.set(edge.source, []);
      traversable.get(edge.source).push(edge.target);
    }
    while (queue.length) {
      const current = queue.shift();
      bfsTraversal.order.push(current);
      const depth = bfsTraversal.depthMap[current];
      const key = String(depth);
      if (!bfsTraversal.byDepth[key]) bfsTraversal.byDepth[key] = [];
      bfsTraversal.byDepth[key].push(current);
      for (const target of (traversable.get(current) || []).sort()) {
        if (Object.prototype.hasOwnProperty.call(bfsTraversal.depthMap, target)) continue;
        bfsTraversal.depthMap[target] = depth + 1;
        queue.push(target);
      }
    }
  }

  const inventoryItem = (node) => ({ id: node.id, name: node.name, type: node.type, summary: node.summary || '' });
  const nonCodeFiles = {
    documentation: nodes.filter((node) => node.type === 'document').map(inventoryItem),
    infrastructure: nodes.filter((node) => ['service', 'pipeline', 'resource'].includes(node.type)).map(inventoryItem),
    data: nodes.filter((node) => ['table', 'schema', 'endpoint'].includes(node.type)).map(inventoryItem),
    config: nodes.filter((node) => node.type === 'config').map(inventoryItem)
  };

  const relationKeys = new Set(edges
    .filter((edge) => ['imports', 'calls'].includes(edge.type))
    .map((edge) => `${edge.source}\u0000${edge.target}\u0000${edge.type}`));
  const pairs = [];
  for (const edge of edges) {
    if (!['imports', 'calls'].includes(edge.type)) continue;
    if (relationKeys.has(`${edge.target}\u0000${edge.source}\u0000${edge.type}`) && edge.source < edge.target) {
      pairs.push(new Set([edge.source, edge.target]));
    }
  }
  const adjacency = new Map(nodes.map((node) => [node.id, new Set()]));
  for (const edge of edges) {
    if (!nodeById.has(edge.source) || !nodeById.has(edge.target)) continue;
    adjacency.get(edge.source).add(edge.target);
    adjacency.get(edge.target).add(edge.source);
  }
  const clusters = pairs.map((cluster) => {
    let changed = true;
    while (changed && cluster.size < 5) {
      changed = false;
      const candidate = nodes
        .filter((node) => !cluster.has(node.id))
        .map((node) => ({ id: node.id, links: [...cluster].filter((id) => adjacency.get(node.id).has(id)).length }))
        .filter((item) => item.links >= 2)
        .sort((a, b) => b.links - a.links || a.id.localeCompare(b.id))[0];
      if (candidate) { cluster.add(candidate.id); changed = true; }
    }
    const ids = [...cluster].sort();
    const edgeCount = edges.filter((edge) => cluster.has(edge.source) && cluster.has(edge.target)).length;
    return { nodes: ids, edgeCount };
  });
  const uniqueClusters = [...new Map(clusters.map((cluster) => [cluster.nodes.join('|'), cluster])).values()]
    .sort((a, b) => b.edgeCount - a.edgeCount || a.nodes.join('|').localeCompare(b.nodes.join('|')))
    .slice(0, 10);

  const nodeSummaryIndex = Object.fromEntries(nodes.map((node) => [
    node.id,
    { name: node.name, type: node.type, summary: node.summary || '' }
  ]));

  const output = {
    scriptCompleted: true,
    entryPointCandidates,
    fanInRanking: ranking(fanIn, 'fanIn'),
    fanOutRanking: ranking(fanOut, 'fanOut'),
    bfsTraversal,
    nonCodeFiles,
    clusters: uniqueClusters,
    layers: { count: layers.length, list: layers.map(({ id, name, description }) => ({ id, name, description })) },
    nodeSummaryIndex,
    totalNodes: nodes.length,
    totalEdges: edges.length
  };
  fs.writeFileSync(outputPath, `${JSON.stringify(output, null, 2)}\n`);
} catch (error) {
  fail(error.stack || String(error));
}
