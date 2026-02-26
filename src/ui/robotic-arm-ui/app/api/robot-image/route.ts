import sharp from "sharp"
import { NextResponse } from "next/server"

const IMAGE_URL =
  "https://hebbkx1anhila5yf.public.blob.vercel-storage.com/Gemini_Generated_Image_8ln5uz8ln5uz8ln5.png-1f0HEqZ2N2syK4b4d7cdjryplPIBII.jpeg"

export async function GET() {
  try {
    const response = await fetch(IMAGE_URL)
    const arrayBuffer = await response.arrayBuffer()
    const inputBuffer = Buffer.from(arrayBuffer)

    const { data, info } = await sharp(inputBuffer)
      .ensureAlpha()
      .raw()
      .toBuffer({ resolveWithObject: true })

    const pixels = Buffer.from(data)

    for (let i = 0; i < pixels.length; i += 4) {
      const r = pixels[i]
      const g = pixels[i + 1]
      const b = pixels[i + 2]

      const max = Math.max(r, g, b)
      const min = Math.min(r, g, b)
      const diff = max - min
      const brightness = (r + g + b) / 3

      // Remove checkered background: low saturation + high brightness
      if (diff < 30 && brightness > 160) {
        pixels[i + 3] = 0
      }
      // Gradual fade for edge pixels
      else if (diff < 40 && brightness > 140) {
        const factor = ((brightness - 140) / 20) * ((40 - diff) / 10)
        pixels[i + 3] = Math.round(255 * (1 - Math.min(factor, 1)))
      }
    }

    const outputBuffer = await sharp(pixels, {
      raw: {
        width: info.width,
        height: info.height,
        channels: 4,
      },
    })
      .png()
      .toBuffer()

    return new NextResponse(outputBuffer, {
      headers: {
        "Content-Type": "image/png",
        "Cache-Control": "public, max-age=31536000, immutable",
      },
    })
  } catch (error) {
    console.error("Failed to process image:", error)
    return NextResponse.json(
      { error: "Failed to process image" },
      { status: 500 }
    )
  }
}
