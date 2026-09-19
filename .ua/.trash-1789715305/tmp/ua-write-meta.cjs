#!/usr/bin/env node
const fs = require('fs');
const root = process.argv[2];
const scan = JSON.parse(fs.readFileSync(`${root}/.ua/intermediate/scan-result.json`, 'utf8'));
const meta = {
  lastAnalyzedAt: new Date().toISOString(),
  gitCommitHash: 'cdfa8d21e2e2a07aa99b7bb83d371fb26dc1df51',
  version: '1.0.0',
  analyzedFiles: scan.files.length,
};
fs.writeFileSync(`${root}/.ua/meta.json`, `${JSON.stringify(meta, null, 2)}\n`);
console.log(`Wrote metadata for ${meta.analyzedFiles} files`);
