import type { Metadata, Viewport } from 'next'
import { Inter, JetBrains_Mono } from 'next/font/google'

import './globals.css'

const _inter = Inter({ subsets: ['latin'], variable: '--font-inter' })
const _jetbrains = JetBrains_Mono({ subsets: ['latin'], variable: '--font-jetbrains' })

export const metadata: Metadata = {
  title: 'Robot Servo Controller',
  description: 'Control each servo motor of a robotic arm with precision sliders',
}

export const viewport: Viewport = {
  themeColor: '#1a5c32',
}

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode
}>) {
  return (
    <html lang="ko">
      <body className={`${_inter.variable} ${_jetbrains.variable} font-sans antialiased`}>{children}</body>
    </html>
  )
}
