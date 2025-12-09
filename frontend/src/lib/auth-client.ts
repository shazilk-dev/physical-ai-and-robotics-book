import { createAuthClient } from "better-auth/react";

const BACKEND_URL = "http://localhost:8000";

export const authClient = createAuthClient({
  baseURL:
    typeof window !== "undefined"
      ? window.location.hostname === "localhost"
        ? BACKEND_URL
        : window.location.origin
      : BACKEND_URL,
});

export const { signIn, signUp, signOut, useSession, $Infer } = authClient;
