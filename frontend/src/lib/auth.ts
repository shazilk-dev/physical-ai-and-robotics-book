import { betterAuth } from "better-auth";
import { Pool, neonConfig } from "@neondatabase/serverless";
import ws from "ws";

// Enable WebSocket for Neon serverless
neonConfig.webSocketConstructor = ws;

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

export const auth = betterAuth({
  database: {
    provider: "postgresql",
    pool: pool,
  },

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },

  user: {
    // Additional user fields for signup questions
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
        defaultValue: "",
      },
      hardwareBackground: {
        type: "string",
        required: false,
        defaultValue: "",
      },
      pythonExperience: {
        type: "string",
        required: false,
        defaultValue: "beginner",
      },
      ros2Experience: {
        type: "string",
        required: false,
        defaultValue: "beginner",
      },
      roboticsExperience: {
        type: "string",
        required: false,
        defaultValue: "beginner",
      },
      preferredLanguage: {
        type: "string",
        required: false,
        defaultValue: "en",
      },
    },
  },

  trustedOrigins: [
    "http://localhost:3000",
    "https://your-vercel-domain.vercel.app",
  ],

  secret: process.env.BETTER_AUTH_SECRET!,
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3000",
});

export type Session = typeof auth.$Infer.Session.session;
export type User = typeof auth.$Infer.Session.user;
