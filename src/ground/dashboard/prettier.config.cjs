module.exports = {
  semi: true,
  singleQuote: true,
  tabWidth: 2,
  trailingComma: 'es5',
  printWidth: 100,
  bracketSpacing: true,
  arrowParens: 'always',
  endOfLine: 'lf',
  plugins: ['prettier-plugin-tailwindcss'],
  tailwindFunctions: ['cn', 'cva'],
  overrides: [
    {
      files: ['*.md', '*.mdx'],
      options: {
        printWidth: 120,
        proseWrap: 'always',
      },
    },
    {
      files: ['*.yaml', '*.yml'],
      options: {
        tabWidth: 2,
      },
    },
    {
      files: ['*.json', '*.jsonc'],
      options: {
        tabWidth: 2,
      },
    },
  ],
}