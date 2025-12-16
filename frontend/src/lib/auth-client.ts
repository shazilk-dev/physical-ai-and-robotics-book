import { createAuthClient } from "better-auth/react";

// Backend URL - detect environment safely in browser
const getBackendUrl = () => {
  // Check if we have a custom API URL set (from environment variables)
  if (typeof window !== 'undefined' && (window as any).docusaurus?.siteConfig?.customFields?.apiUrl) {
    const apiUrl = (window as any).docusaurus.siteConfig.customFields.apiUrl;
    // Remove /api/v1 suffix to get base URL for auth
    return apiUrl.replace('/api/v1', '');
  }

  // Check if running on production domain (Vercel)
  if (typeof window !== 'undefined' &&
      (window.location.hostname.includes('vercel.app') ||
       window.location.hostname.includes('physical-ai-robotics-book'))) {
    return 'https://physical-ai-and-robotics-book.onrender.com';
  }

  // Default to localhost for development
  return 'http://localhost:8000';
};

const BACKEND_URL = typeof window !== 'undefined' ? getBackendUrl() : 'http://localhost:8000';

export const authClient = createAuthClient({
  baseURL: BACKEND_URL,
});

export const { signIn, signUp, signOut, useSession, $Infer } = authClient;
