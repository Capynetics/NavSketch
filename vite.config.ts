import { defineConfig, type Plugin, type ViteDevServer } from 'vite'
import react from '@vitejs/plugin-react'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import fs from 'node:fs/promises'
import type { IncomingMessage, ServerResponse } from 'node:http'

const projectRoot = path.dirname(fileURLToPath(import.meta.url))
const worldFilePath = path.join(projectRoot, 'src/simulation/world.json')
const scenariosDirPath = path.join(projectRoot, 'src/simulation/scenarios')

function worldJsonWriterPlugin(): Plugin {
  return {
    name: 'world-json-writer',
    configureServer(server: ViteDevServer) {
      server.middlewares.use((req, res, next) => {
        if (req.method !== 'POST' || req.url !== '/__world-json') {
          next()
          return
        }

        const bodyChunks: Buffer[] = []
        const typedReq = req as IncomingMessage
        const typedRes = res as ServerResponse

        typedReq.on('data', (chunk: Buffer) => {
          bodyChunks.push(chunk)
        })

        typedReq.on('end', async () => {
          try {
            const rawBody = Buffer.concat(bodyChunks).toString('utf-8')
            const body = JSON.parse(rawBody) as {
              parameters: unknown
              scenarioName: string
            }

            const scenarioFilePath = path.join(scenariosDirPath, `${body.scenarioName}.json`)
            const scenarioRaw = await fs.readFile(scenarioFilePath, 'utf-8')
            const scenario = JSON.parse(scenarioRaw)

            let existingWorld: Record<string, unknown> = {}
            try {
              const existingRaw = await fs.readFile(worldFilePath, 'utf-8')
              existingWorld = JSON.parse(existingRaw) as Record<string, unknown>
            } catch {
              existingWorld = {}
            }

            const mergedWorld = {
              ...existingWorld,
              parameters: body.parameters,
              scenario,
            }

            const formatted = `${JSON.stringify(mergedWorld, null, 2)}\n`
            await fs.writeFile(worldFilePath, formatted, 'utf-8')

            typedRes.statusCode = 200
            typedRes.setHeader('Content-Type', 'application/json')
            typedRes.end(JSON.stringify({ ok: true }))
          } catch {
            typedRes.statusCode = 400
            typedRes.setHeader('Content-Type', 'application/json')
            typedRes.end(JSON.stringify({ ok: false }))
          }
        })
      })
    },
  }
}

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), worldJsonWriterPlugin()],
})
