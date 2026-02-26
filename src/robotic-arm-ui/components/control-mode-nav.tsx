"use client"

import Link from "next/link"
import { Button } from "@/components/ui/button"

interface ControlModeNavProps {
  currentMode: "servo" | "axis-lock"
}

export function ControlModeNav({ currentMode }: ControlModeNavProps) {
  return (
    <div className="flex items-center gap-2">
      <Button
        asChild
        size="sm"
        variant={currentMode === "servo" ? "default" : "outline"}
        className="font-mono text-xs"
      >
        <Link href="/">Servo Mode</Link>
      </Button>
      <Button
        asChild
        size="sm"
        variant={currentMode === "axis-lock" ? "default" : "outline"}
        className="font-mono text-xs"
      >
        <Link href="/axis-lock">Axis Lock Mode</Link>
      </Button>
    </div>
  )
}
