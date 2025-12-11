#!/bin/bash

# Vercel Deployment Fix Script
# Run this before deploying to ensure clean build

echo "🧹 Cleaning Docusaurus cache..."
cd frontend
npm run clear

echo "📦 Building production version..."
npm run build

echo "✅ Build complete!"
echo ""
echo "📋 Next steps:"
echo "1. git add ."
echo "2. git commit -m 'Fix: Force clean build for Vercel'"
echo "3. git push origin main"
echo "4. In Vercel Dashboard: Redeploy WITHOUT cache"
echo ""
echo "🔗 Vercel Dashboard: https://vercel.com/dashboard"
