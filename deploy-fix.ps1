# Vercel Deployment Fix Script (Windows PowerShell)
# Run this before deploying to ensure clean build

Write-Host "🧹 Cleaning Docusaurus cache..." -ForegroundColor Yellow
Set-Location frontend
npm run clear

Write-Host "`n📦 Building production version..." -ForegroundColor Yellow
npm run build

Write-Host "`n✅ Build complete!" -ForegroundColor Green
Write-Host ""
Write-Host "📋 Next steps:" -ForegroundColor Cyan
Write-Host "1. git add ."
Write-Host "2. git commit -m 'Fix: Force clean build for Vercel'"
Write-Host "3. git push origin main"
Write-Host "4. In Vercel Dashboard: Redeploy WITHOUT cache"
Write-Host ""
Write-Host "🔗 Vercel Dashboard: https://vercel.com/dashboard" -ForegroundColor Blue

Set-Location ..
