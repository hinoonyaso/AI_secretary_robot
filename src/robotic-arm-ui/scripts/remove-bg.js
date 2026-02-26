import sharp from 'sharp';
import fs from 'fs';

// Fetch image from the blob URL since local filesystem is sandboxed
const imageUrl = 'https://hebbkx1anhila5yf.public.blob.vercel-storage.com/Gemini_Generated_Image_8ln5uz8ln5uz8ln5.png-1f0HEqZ2N2syK4b4d7cdjryplPIBII.jpeg';

console.log('[v0] Fetching image from URL...');
const response = await fetch(imageUrl);
const arrayBuffer = await response.arrayBuffer();
const inputBuffer = Buffer.from(arrayBuffer);
console.log('[v0] Downloaded image, size:', inputBuffer.length);

async function removeBg() {
  const image = sharp(inputBuffer);
  const metadata = await image.metadata();
  const { width, height } = metadata;

  console.log(`[v0] Image size: ${width}x${height}`);

  const { data, info } = await image
    .ensureAlpha()
    .raw()
    .toBuffer({ resolveWithObject: true });

  console.log(`[v0] Channels: ${info.channels}, Pixels: ${info.width * info.height}`);

  const pixels = Buffer.from(data);

  let removedCount = 0;

  for (let i = 0; i < pixels.length; i += 4) {
    const r = pixels[i];
    const g = pixels[i + 1];
    const b = pixels[i + 2];

    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    const diff = max - min;
    const brightness = (r + g + b) / 3;

    // Remove low-saturation bright pixels (checkered white/gray background)
    if (diff < 25 && brightness > 170) {
      pixels[i + 3] = 0;
      removedCount++;
    }
  }

  console.log(`[v0] Removed ${removedCount} background pixels`);

  const outputBuffer = await sharp(pixels, {
    raw: {
      width: info.width,
      height: info.height,
      channels: 4,
    },
  })
    .png()
    .toBuffer();

  // Write the file locally so we can copy it back
  fs.writeFileSync('robot-arm-clean.png', outputBuffer);
  console.log(`[v0] Saved clean image: robot-arm-clean.png (${outputBuffer.length} bytes)`);
  console.log(`[v0] BASE64_START${outputBuffer.toString('base64')}BASE64_END`);
}

removeBg().catch((err) => {
  console.error('[v0] Error:', err);
  process.exit(1);
});
