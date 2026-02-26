"use client"

import { Slider } from "@/components/ui/slider"
import { cn } from "@/lib/utils"

interface ServoSliderProps {
  id: number
  label: string
  value: number
  min: number
  max: number
  unit?: string
  icon: React.ReactNode
  onChange: (value: number) => void
}

export function ServoSlider({
  id,
  label,
  value,
  min,
  max,
  unit = "\u00B0",
  icon,
  onChange,
}: ServoSliderProps) {
  return (
    <div className="group rounded-xl border border-border bg-card p-4 transition-all hover:border-primary/40 hover:bg-card/80">
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-3">
          <div className="flex h-9 w-9 items-center justify-center rounded-lg bg-primary/10 text-primary">
            {icon}
          </div>
          <div>
            <p className="text-xs font-mono text-muted-foreground tracking-wider uppercase">
              {"Servo #"}{id}
            </p>
            <p className="text-sm font-medium text-foreground">{label}</p>
          </div>
        </div>
        <div className="flex items-center gap-1.5">
          <input
            type="number"
            value={value}
            min={min}
            max={max}
            onChange={(e) => {
              const v = Number(e.target.value)
              if (v >= min && v <= max) onChange(v)
            }}
            className={cn(
              "w-16 rounded-md border border-border bg-secondary px-2 py-1 text-right font-mono text-sm text-foreground",
              "focus:outline-none focus:ring-1 focus:ring-primary focus:border-primary",
              "[appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
            )}
            aria-label={`${label} angle value`}
          />
          <span className="text-xs font-mono text-muted-foreground">{unit}</span>
        </div>
      </div>

      <div className="relative">
        <Slider
          value={[value]}
          min={min}
          max={max}
          step={1}
          onValueChange={(vals) => onChange(vals[0])}
          className="w-full"
          aria-label={`${label} control slider`}
        />
        <div className="flex justify-between mt-1.5">
          <span className="text-[10px] font-mono text-muted-foreground">{min}{unit}</span>
          <span className="text-[10px] font-mono text-muted-foreground">{max}{unit}</span>
        </div>
      </div>
    </div>
  )
}
