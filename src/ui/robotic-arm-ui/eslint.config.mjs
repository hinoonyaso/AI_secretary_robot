import nextVitals from "eslint-config-next/core-web-vitals"
import nextTs from "eslint-config-next/typescript"

const eslintConfig = [
  ...nextVitals,
  ...nextTs,
  {
    ignores: [
      ".next/**",
      "node_modules/**",
      "ros2_ws/**",
      "tsconfig.tsbuildinfo",
    ],
  },
]

export default eslintConfig
