import { NextRequest, NextResponse } from "next/server"
import dgram from "node:dgram"

export const runtime = "nodejs"

interface ServoCommandBody {
  servoId: number
  value: number
  durationMs?: number
}

function isValidNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value)
}

export async function POST(req: NextRequest) {
  let body: ServoCommandBody

  try {
    body = (await req.json()) as ServoCommandBody
  } catch {
    return NextResponse.json({ error: "Invalid JSON body" }, { status: 400 })
  }

  if (!isValidNumber(body.servoId) || !isValidNumber(body.value)) {
    return NextResponse.json(
      { error: "servoId and value must be numbers" },
      { status: 400 }
    )
  }

  const durationMs = isValidNumber(body.durationMs) ? body.durationMs : 300

  const payload = JSON.stringify({
    servo_id: Math.trunc(body.servoId),
    value: body.value,
    duration_ms: Math.max(0, Math.trunc(durationMs)),
  })

  const host = process.env.ROBOT_UDP_HOST ?? "127.0.0.1"
  const port = Number(process.env.ROBOT_UDP_PORT ?? "9999")

  if (!Number.isInteger(port) || port <= 0 || port > 65535) {
    return NextResponse.json(
      { error: "ROBOT_UDP_PORT must be a valid port number" },
      { status: 500 }
    )
  }

  const socket = dgram.createSocket("udp4")

  try {
    await new Promise<void>((resolve, reject) => {
      socket.send(Buffer.from(payload, "utf8"), port, host, (err) => {
        if (err) {
          reject(err)
          return
        }
        resolve()
      })
    })
  } catch (error) {
    return NextResponse.json(
      { error: `Failed to send UDP command: ${String(error)}` },
      { status: 502 }
    )
  } finally {
    socket.close()
  }

  return NextResponse.json({ ok: true })
}
