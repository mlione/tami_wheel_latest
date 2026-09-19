#!/usr/bin/env node
const fs = require('fs');
const root = process.argv[2];
const ua = `${root}/.ua`;
const scan = JSON.parse(fs.readFileSync(`${ua}/intermediate/scan-result.json`, 'utf8'));
const graphSource = `${ua}/intermediate/assembled-graph.json`;
fs.copyFileSync(graphSource, `${ua}/knowledge-graph.json`);
const input = {
  projectRoot: root,
  filePaths: scan.files.map((file) => file.path),
  gitCommitHash: 'cdfa8d21e2e2a07aa99b7bb83d371fb26dc1df51',
};
fs.writeFileSync(`${ua}/intermediate/fingerprint-input.json`, `${JSON.stringify(input, null, 2)}\n`);
console.log(`Prepared final graph and ${input.filePaths.length} fingerprint paths`);
