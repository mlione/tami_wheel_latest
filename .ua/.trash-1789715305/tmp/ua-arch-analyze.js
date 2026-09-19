#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

function die(message) {
  process.stderr.write(`${message}\n`);
  process.exit(1);
}

const [inputPath, outputPath] = process.argv.slice(2);
if (!inputPath || !outputPath) die('usage: ua-arch-analyze.js INPUT OUTPUT');

let input;
try {
  input = JSON.parse(fs.readFileSync(inputPath, 'utf8'));
} catch (error) {
  die(`failed to read input: ${error.message}`);
}

const fileNodes = Array.isArray(input.fileNodes) ? input.fileNodes : [];
const importEdges = Array.isArray(input.importEdges) ? input.importEdges : [];
const allEdges = Array.isArray(input.allEdges) ? input.allEdges : [];
const ids = new Set(fileNodes.map((node) => node.id));
const nodeById = new Map(fileNodes.map((node) => [node.id, node]));

function commonDirectoryPrefix(paths) {
  if (!paths.length) return '';
  const dirs = paths.map((value) => value.split('/').slice(0, -1));
  const prefix = [];
  for (let index = 0; index < Math.min(...dirs.map((parts) => parts.length)); index += 1) {
    const segment = dirs[0][index];
    if (!dirs.every((parts) => parts[index] === segment)) break;
    prefix.push(segment);
  }
  return prefix.length ? `${prefix.join('/')}/` : '';
}

const paths = fileNodes.map((node) => node.filePath || node.name || node.id);
const commonPrefix = commonDirectoryPrefix(paths);
const flat = paths.every((value) => !value.includes('/'));

function flatGroup(node) {
  const value = (node.filePath || node.name || '').toLowerCase();
  if (/([._-](test|spec)\.|^(test_))/.test(value)) return 'test';
  if (/(config|cmakelists|package\.xml)/.test(value)) return 'config';
  const extension = path.extname(value).slice(1);
  return extension || 'root';
}

function groupFor(node) {
  const filePath = node.filePath || node.name || node.id;
  if (flat) return flatGroup(node);
  let relative = commonPrefix && filePath.startsWith(commonPrefix)
    ? filePath.slice(commonPrefix.length)
    : filePath;
  const parts = relative.split('/');
  return parts.length > 1 ? parts[0] : 'root';
}

const directoryGroups = {};
const groupById = new Map();
for (const node of fileNodes) {
  const group = groupFor(node);
  if (!directoryGroups[group]) directoryGroups[group] = [];
  directoryGroups[group].push(node.id);
  groupById.set(node.id, group);
}

const nodeTypeGroups = {};
for (const node of fileNodes) {
  if (!nodeTypeGroups[node.type]) nodeTypeGroups[node.type] = [];
  nodeTypeGroups[node.type].push(node.id);
}

const fileFanIn = Object.fromEntries(fileNodes.map((node) => [node.id, 0]));
const fileFanOut = Object.fromEntries(fileNodes.map((node) => [node.id, 0]));
const adjacency = Object.fromEntries(fileNodes.map((node) => [node.id, []]));
const interCounts = new Map();
const groupInternal = Object.fromEntries(Object.keys(directoryGroups).map((group) => [group, 0]));
const groupTotal = Object.fromEntries(Object.keys(directoryGroups).map((group) => [group, 0]));
for (const edge of importEdges) {
  if (!ids.has(edge.source) || !ids.has(edge.target)) continue;
  adjacency[edge.source].push(edge.target);
  fileFanOut[edge.source] += 1;
  fileFanIn[edge.target] += 1;
  const from = groupById.get(edge.source);
  const to = groupById.get(edge.target);
  groupTotal[from] += 1;
  if (to !== from) groupTotal[to] += 1;
  if (from === to) groupInternal[from] += 1;
  const key = `${from}\u0000${to}`;
  interCounts.set(key, (interCounts.get(key) || 0) + 1);
}

const interGroupImports = [...interCounts.entries()].map(([key, count]) => {
  const [from, to] = key.split('\u0000');
  return {from, to, count};
}).sort((a, b) => a.from.localeCompare(b.from) || a.to.localeCompare(b.to));

const intraGroupDensity = {};
for (const group of Object.keys(directoryGroups)) {
  const internalEdges = groupInternal[group];
  const totalEdges = groupTotal[group];
  intraGroupDensity[group] = {
    internalEdges,
    totalEdges,
    density: totalEdges ? internalEdges / totalEdges : 0,
  };
}

const patterns = [
  [/^(routes?|api|controllers?|endpoints?|handlers?|serializers?|routers?|blueprints?)$/i, 'api'],
  [/^(services?|core|lib|domain|logic|internal|composables|mailers|jobs|channels|signals)$/i, 'service'],
  [/^(models?|db|data|persistence|repository|entities|entity|migrations|sql|database|schema)$/i, 'data'],
  [/^(components?|views?|pages?|ui|layouts?|screens?)$/i, 'ui'],
  [/^(middleware|plugins?|interceptors?|guards?)$/i, 'middleware'],
  [/^(utils?|helpers?|common|shared|tools|pkg|templatetags)$/i, 'utility'],
  [/^(config|constants|env|settings|management|commands)$/i, 'config'],
  [/^(__tests__|tests?|specs?)$/i, 'test'],
  [/^(types?|interfaces?|schemas?|contracts?|dtos?|dto|request|response)$/i, 'types'],
  [/^(hooks?)$/i, 'hooks'],
  [/^(store|state|reducers|actions|slices)$/i, 'state'],
  [/^(assets?|static|public)$/i, 'assets'],
  [/^(cmd|bin)$/i, 'entry'],
  [/^(docs?|documentation|wiki)$/i, 'documentation'],
  [/^(deploy|deployment|infra|infrastructure|k8s|kubernetes|helm|charts|terraform|tf|docker)$/i, 'infrastructure'],
  [/^(\.github|\.gitlab|\.circleci)$/i, 'ci-cd'],
];

function filePattern(node) {
  const p = (node.filePath || node.name || '').toLowerCase();
  const base = path.basename(p);
  if (/((\.test|\.spec)\.|(^|\/)test_[^/]+\.py$|_test\.go$|test\.java$|_spec\.rb$|test\.php$|tests\.cs$)/.test(p)) return 'test';
  if (p.endsWith('.d.ts') || /\.(graphql|gql|proto)$/.test(p)) return 'types';
  if (/\.(md|rst)$/.test(p)) return 'documentation';
  if (/\.(sql)$/.test(p)) return 'data';
  if (base === 'dockerfile' || /^docker-compose\./.test(base) || /\.(tf|tfvars)$/.test(p) || base === 'makefile') return 'infrastructure';
  if (p.startsWith('.github/workflows/') || base === '.gitlab-ci.yml' || base === 'jenkinsfile') return 'ci-cd';
  if (['cargo.toml', 'go.mod', 'gemfile', 'pom.xml', 'build.gradle', 'composer.json', 'cmakelists.txt', 'package.xml'].includes(base)) return 'config';
  if (['index.ts', 'index.js', '__init__.py', 'manage.py', 'config.ru', 'main.rs', 'lib.rs', 'application.java', 'program.cs'].includes(base)) return 'entry';
  if (['wsgi.py', 'asgi.py'].includes(base)) return 'config';
  return null;
}

const patternMatches = {};
for (const [group, members] of Object.entries(directoryGroups)) {
  const directoryMatch = patterns.find(([regex]) => regex.test(group));
  if (directoryMatch) {
    patternMatches[group] = directoryMatch[1];
    continue;
  }
  const labels = members.map((id) => filePattern(nodeById.get(id))).filter(Boolean);
  if (labels.length && labels.every((label) => label === labels[0])) patternMatches[group] = labels[0];
}

const crossCounts = new Map();
for (const edge of allEdges) {
  if (!ids.has(edge.source) || !ids.has(edge.target)) continue;
  const fromType = nodeById.get(edge.source).type;
  const toType = nodeById.get(edge.target).type;
  if (fromType === toType) continue;
  const key = `${fromType}\u0000${toType}\u0000${edge.type}`;
  crossCounts.set(key, (crossCounts.get(key) || 0) + 1);
}
const crossCategoryEdges = [...crossCounts.entries()].map(([key, count]) => {
  const [fromType, toType, edgeType] = key.split('\u0000');
  return {fromType, toType, edgeType, count};
});

const reverseInter = new Map(interCounts);
const dependencyDirection = [];
const compared = new Set();
for (const {from, to, count} of interGroupImports) {
  if (from === to) continue;
  const pair = [from, to].sort().join('\u0000');
  if (compared.has(pair)) continue;
  compared.add(pair);
  const reverse = reverseInter.get(`${to}\u0000${from}`) || 0;
  if (count > reverse) dependencyDirection.push({dependent: from, dependsOn: to});
  else if (reverse > count) dependencyDirection.push({dependent: to, dependsOn: from});
}

const infraFiles = fileNodes.filter((node) => {
  const p = (node.filePath || '').toLowerCase();
  const base = path.basename(p);
  return base === 'dockerfile' || /^docker-compose\./.test(base) || /(^|\/)(k8s|kubernetes|helm|charts|terraform)(\/|$)/.test(p) || /\.(tf|tfvars)$/.test(p) || p.startsWith('.github/workflows/') || base === '.gitlab-ci.yml' || base === 'jenkinsfile';
}).map((node) => node.filePath);

const deploymentTopology = {
  hasDockerfile: infraFiles.some((p) => path.basename(p).toLowerCase() === 'dockerfile'),
  hasCompose: infraFiles.some((p) => /^docker-compose\./.test(path.basename(p).toLowerCase())),
  hasK8s: infraFiles.some((p) => /(^|\/)(k8s|kubernetes|helm|charts)(\/|$)/.test(p.toLowerCase())),
  hasTerraform: infraFiles.some((p) => /(^|\/)(terraform)(\/|$)|\.(tf|tfvars)$/.test(p.toLowerCase())),
  hasCI: infraFiles.some((p) => p.startsWith('.github/workflows/') || /(^|\/)(\.gitlab-ci\.yml|jenkinsfile)$/.test(p.toLowerCase())),
  infraFiles,
};

const dataPipeline = {
  schemaFiles: fileNodes.filter((n) => /\.(sql|graphql|gql|proto|prisma)$/.test((n.filePath || '').toLowerCase())).map((n) => n.filePath),
  migrationFiles: fileNodes.filter((n) => /(^|\/)migrations?(\/|$)/i.test(n.filePath || '')).map((n) => n.filePath),
  dataModelFiles: fileNodes.filter((n) => /(^|\/)(models?|entities)(\/|$)/i.test(n.filePath || '') || (n.tags || []).some((tag) => /数据模型|data-model|model/i.test(tag))).map((n) => n.filePath),
  apiHandlerFiles: fileNodes.filter((n) => /(^|\/)(routes?|controllers?|handlers?|api)(\/|$)/i.test(n.filePath || '') || (n.tags || []).some((tag) => /api-handler|控制器|端点/i.test(tag))).map((n) => n.filePath),
};

const documentNodes = fileNodes.filter((node) => node.type === 'document' || /\.(md|rst)$/i.test(node.filePath || ''));
const documentedGroups = new Set();
for (const node of documentNodes) {
  const docPath = (node.filePath || '').toLowerCase();
  const docText = `${node.summary || ''} ${(node.tags || []).join(' ')}`.toLowerCase();
  const group = groupById.get(node.id);
  if (group) documentedGroups.add(group);
  for (const name of Object.keys(directoryGroups)) {
    if (docPath.includes(`/${name.toLowerCase()}/`) || docText.includes(name.toLowerCase())) documentedGroups.add(name);
  }
}
const allGroups = Object.keys(directoryGroups);
const docCoverage = {
  groupsWithDocs: documentedGroups.size,
  totalGroups: allGroups.length,
  coverageRatio: allGroups.length ? documentedGroups.size / allGroups.length : 0,
  undocumentedGroups: allGroups.filter((group) => !documentedGroups.has(group)),
};

const output = {
  scriptCompleted: true,
  commonPrefix,
  directoryGroups,
  nodeTypeGroups,
  importAdjacency: adjacency,
  crossCategoryEdges,
  interGroupImports,
  intraGroupDensity,
  patternMatches,
  deploymentTopology,
  dataPipeline,
  docCoverage,
  dependencyDirection,
  fileStats: {
    totalFileNodes: fileNodes.length,
    filesPerGroup: Object.fromEntries(Object.entries(directoryGroups).map(([group, members]) => [group, members.length])),
    nodeTypeCounts: Object.fromEntries(Object.entries(nodeTypeGroups).map(([type, members]) => [type, members.length])),
  },
  fileFanIn,
  fileFanOut,
};

try {
  fs.writeFileSync(outputPath, `${JSON.stringify(output, null, 2)}\n`);
} catch (error) {
  die(`failed to write output: ${error.message}`);
}
