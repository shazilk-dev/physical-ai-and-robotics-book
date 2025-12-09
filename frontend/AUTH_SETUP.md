# Authentication Setup Guide

## Step 1: Create Neon Database

1. Go to [Neon Console](https://console.neon.tech/)
2. Click "Create Project"
3. Name: `physical-ai-robotics-book`
4. Region: Choose closest to your users
5. Copy the connection string

## Step 2: Run Database Schema

1. Open Neon SQL Editor
2. Paste contents of `database/schema.sql`
3. Click "Run" to create tables

## Step 3: Configure Environment Variables

1. Copy `.env.local.example` to `.env.local`:

   ```bash
   cp .env.local.example .env.local
   ```

2. Fill in the values:

   ```env
   # Generate secret with: openssl rand -base64 32
   BETTER_AUTH_SECRET=your-generated-secret-here

   BETTER_AUTH_URL=http://localhost:3000

   # Paste your Neon connection string
   DATABASE_URL=postgresql://username:password@ep-xxx.neon.tech/dbname?sslmode=require
   ```

3. **Important**: Add `.env.local` to `.gitignore` (already done)

## Step 4: Update Docusaurus Config

The auth API route is already created at `/src/pages/api/auth/[...all].ts`.

Docusaurus will automatically handle API routes in the `src/pages/api/` directory.

## Step 5: Test Authentication

1. Start dev server:

   ```bash
   npm run start
   ```

2. The auth endpoints will be available at:
   - `http://localhost:3000/api/auth/sign-in`
   - `http://localhost:3000/api/auth/sign-up`
   - `http://localhost:3000/api/auth/sign-out`

## Step 6: Build UI Components (Next Task)

Now we'll create:

- Login modal
- Signup modal with background questions
- Auth buttons in navbar
- User profile dropdown

## Troubleshooting

### Connection Issues

- Verify DATABASE_URL is correct
- Check Neon project is not paused (free tier pauses after inactivity)
- Ensure WebSocket connections are allowed (Neon requires ws)

### Build Errors

- Make sure all packages are installed: `npm install`
- Check Node version: `node --version` (should be 18+)
- Clear cache: `npm run clear` then `npm run start`

### CORS Errors

- Verify `trustedOrigins` in `src/lib/auth.ts` includes your domain
- Add Vercel URL to trustedOrigins before deploying

## Production Deployment (Vercel)

1. Add environment variables in Vercel dashboard:

   - `BETTER_AUTH_SECRET` (generate new one for production)
   - `BETTER_AUTH_URL` (your Vercel URL)
   - `DATABASE_URL` (your Neon connection string)

2. Deploy:
   ```bash
   vercel --prod
   ```

## Security Checklist

- [x] `.env.local` in `.gitignore`
- [ ] Enable email verification for production (`requireEmailVerification: true`)
- [ ] Use strong secret (32+ characters random)
- [ ] Enable HTTPS in production
- [ ] Add rate limiting to auth endpoints
- [ ] Implement CAPTCHA for signup (optional)
