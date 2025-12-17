"""
Seed All LLM Provider Collections
Automatically detects configured providers and seeds their vector collections
"""
import os
import sys
import subprocess
from pathlib import Path
from dotenv import load_dotenv

# Fix Windows console encoding for emojis
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv(Path(__file__).parent.parent / ".env")


def check_provider_configured(provider: str) -> bool:
    """Check if provider has API key configured"""
    key_map = {
        'openai': 'OPENAI_API_KEY',
        'gemini': 'GEMINI_API_KEY',
        'qwen': 'QWEN_API_KEY'
    }
    api_key = os.getenv(key_map[provider])
    return bool(api_key and len(api_key) > 0)


def check_sdk_installed(provider: str) -> bool:
    """Check if provider SDK is installed"""
    try:
        if provider == 'openai':
            import openai
            return True
        elif provider == 'gemini':
            import google.generativeai
            return True
        elif provider == 'qwen':
            # Qwen uses standard OpenAI client with custom base URL
            import openai
            return True
    except ImportError:
        return False


def install_sdk(provider: str):
    """Install missing SDK"""
    sdk_map = {
        'openai': 'openai',
        'gemini': 'google-generativeai',
        'qwen': 'openai'  # Qwen uses OpenAI SDK
    }

    package = sdk_map[provider]
    print(f"📦 Installing {package}...")
    subprocess.run([sys.executable, "-m", "pip", "install", package], check=True)
    print(f"✅ {package} installed successfully")


def seed_provider(provider: str):
    """Seed vector collection for specific provider"""
    print(f"\n{'='*60}")
    print(f"🌱 Seeding {provider.upper()} Collection")
    print(f"{'='*60}\n")

    # Set environment variable for this provider
    env = os.environ.copy()
    env['LLM_PROVIDER'] = provider

    # Run seed script
    seed_script = Path(__file__).parent / "seed_vector_db.py"
    result = subprocess.run(
        [sys.executable, str(seed_script)],
        env=env,
        capture_output=False,
        text=True
    )

    if result.returncode == 0:
        print(f"\n✅ {provider.upper()} collection seeded successfully!")
    else:
        print(f"\n❌ {provider.upper()} seeding failed with code {result.returncode}")
        return False

    return True


def main():
    """Main function"""
    print("🚀 Multi-Provider Vector Database Seeding")
    print("="*60)
    print("\nThis script will seed vector collections for all configured providers.")
    print("Each provider requires its own collection due to different embedding dimensions.\n")

    providers = ['openai', 'gemini', 'qwen']

    # Check which providers are configured
    print("📋 Checking Provider Configuration:\n")
    configured_providers = []

    for provider in providers:
        has_key = check_provider_configured(provider)
        has_sdk = check_sdk_installed(provider)

        status = "✅ Ready" if (has_key and has_sdk) else "⚠️ Not Ready"
        print(f"  {provider.upper()}")
        print(f"    API Key: {'✅ Configured' if has_key else '❌ Missing'}")
        print(f"    SDK: {'✅ Installed' if has_sdk else '❌ Not Installed'}")
        print(f"    Status: {status}\n")

        if has_key:
            if not has_sdk:
                print(f"  📦 Installing SDK for {provider}...")
                try:
                    install_sdk(provider)
                    configured_providers.append(provider)
                except Exception as e:
                    print(f"  ❌ Failed to install SDK: {e}")
            else:
                configured_providers.append(provider)

    if not configured_providers:
        print("\n❌ No providers configured!")
        print("\nPlease add API keys to backend/.env:")
        print("  OPENAI_API_KEY=sk-...")
        print("  GEMINI_API_KEY=AIza...")
        print("  QWEN_API_KEY=sk-...")
        sys.exit(1)

    # Confirm before proceeding
    print(f"\n📊 Will seed {len(configured_providers)} provider(s): {', '.join([p.upper() for p in configured_providers])}")
    print(f"\n⚠️  This will overwrite existing collections!")

    response = input("\nProceed with seeding? (y/n): ").strip().lower()
    if response != 'y':
        print("\n🛑 Seeding cancelled.")
        sys.exit(0)

    # Seed each provider
    results = {}
    for provider in configured_providers:
        success = seed_provider(provider)
        results[provider] = success

    # Summary
    print("\n" + "="*60)
    print("📊 Seeding Summary")
    print("="*60 + "\n")

    for provider, success in results.items():
        status = "✅ Success" if success else "❌ Failed"
        print(f"  {provider.upper()}: {status}")

    successful = sum(results.values())
    total = len(results)

    print(f"\n🎯 {successful}/{total} providers seeded successfully")

    if successful == total:
        print("\n✅ All provider collections are ready!")
        print("\nTest with:")
        print("  curl -X POST http://localhost:8000/api/v1/query \\")
        print("    -H 'Content-Type: application/json' \\")
        print("    -d '{\"query\": \"What is ROS 2?\", \"provider\": \"openai\"}'")
    else:
        print("\n⚠️  Some providers failed to seed. Check errors above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
