"use client"

import { cn } from "@/lib/utils"

interface ConnectionStatusProps {
  status: "connected" | "disconnected" | "connecting"
}

export function ConnectionStatus({ status }: ConnectionStatusProps) {
  return (
    <div className="flex items-center gap-2">
      <div className="relative flex h-2.5 w-2.5 items-center justify-center">
        <div
          className={cn(
            "absolute h-full w-full rounded-full",
            status === "connected" && "bg-primary animate-ping opacity-75",
            status === "connecting" && "bg-yellow-500 animate-ping opacity-75",
            status === "disconnected" && "bg-destructive"
          )}
        />
        <div
          className={cn(
            "relative h-2 w-2 rounded-full",
            status === "connected" && "bg-primary",
            status === "connecting" && "bg-yellow-500",
            status === "disconnected" && "bg-destructive"
          )}
        />
      </div>
      <span className="text-xs font-mono text-muted-foreground capitalize">
        {status}
      </span>
    </div>
  )
}
