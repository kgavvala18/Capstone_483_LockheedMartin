// preload.js

const { contextBridge } = require('electron')
const Mousetrap = require('mousetrap')

// Expose Mousetrap to the renderer process safely.
contextBridge.exposeInMainWorld('Mousetrap', Mousetrap)

// Also expose other APIs as needed.
contextBridge.exposeInMainWorld('versions', {
  node: () => process.versions.node,
  chrome: () => process.versions.chrome,
  electron: () => process.versions.electron,
  ping: () => ipcRenderer.invoke('ping')
})
