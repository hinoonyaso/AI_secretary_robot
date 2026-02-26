import sharp from 'sharp';
import path from 'path';

const inputPath = path.resolve('public/images/robot-arm.png');
const outputPath = path.resolve('public/images/robot-arm-clean.png');

async function removeBg() {
  const image = sharp(inputPath);
  const metadata = await image.metadata();
  const { width, height } = metadata;

  console.log(`[v0] Image size: ${width}x${height}`);

  // Get raw pixel data (RGBA)
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

    // The checkered background consists of light gray (~204,204,204) and white (~255,255,255) pixels
    // Detect near-gray and near-white pixels where R, G, B are very close to each other (low saturation)
    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    const diff = max - min;
    const brightness = (r + g + b) / 3;

    // Remove pixels that are:
    // - Low saturation (gray/white) AND bright enough to be background
    // - This catches the checkered pattern (white ~255 and light gray ~190-210)
    if (diff < 25 && brightness > 170) {
      pixels[i + 3] = 0; // Set alpha to 0 (transparent)
      removedCount++;
    }
  }

  console.log(`[v0] Removed ${removedCount} background pixels`);

  await sharp(pixels, {
    raw: {
      width: info.width,
      height: info.height,
      channels: 4,
    },
  })
    .png()
    .toFile(outputPath);

  console.log(`[v0] Saved clean image to ${outputPath}`);
}

removeBg().catch((err) => {
  console.error('[v0] Error:', err);
  process.exit(1);
});
