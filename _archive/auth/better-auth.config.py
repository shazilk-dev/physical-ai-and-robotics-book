"""
Better-auth Configuration
Authentication setup for Physical AI Textbook
"""

from better_auth import BetterAuth
from better_auth.providers import EmailProvider, GoogleProvider, GitHubProvider

# Initialize Better-auth
auth = BetterAuth(
    database_url="postgresql://...",  # From environment
    secret="your-secret-key",  # From environment
    providers=[
        EmailProvider(
            smtp_host="smtp.gmail.com",
            smtp_port=587,
            smtp_user="your-email@gmail.com",
            smtp_password="your-app-password"
        ),
        GoogleProvider(
            client_id="your-google-client-id",
            client_secret="your-google-client-secret",
            redirect_uri="http://localhost:3000/auth/callback/google"
        ),
        GitHubProvider(
            client_id="your-github-client-id",
            client_secret="your-github-client-secret",
            redirect_uri="http://localhost:3000/auth/callback/github"
        )
    ]
)

# User signup flow configuration
SIGNUP_FLOW = {
    "collect_background": True,
    "required_fields": ["email", "username", "password"],
    "optional_fields": ["full_name"],
    "background_questions": "signup-questions.json"
}
