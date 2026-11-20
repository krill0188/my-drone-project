#!/usr/bin/env bash
# Initialize Python virtual environments for ground tools and dashboard

set -euo pipefail

PROJECT_ROOT="/my-drone-project"

echo "=== Initializing Python Virtual Environments ==="

# Ground tools venv
echo "Creating ground tools venv..."
python3.11 -m venv "$PROJECT_ROOT/.venv_ground"
source "$PROJECT_ROOT/.venv_ground/bin/activate"
pip install --upgrade pip setuptools wheel
pip install -e "$PROJECT_ROOT"  # Install drone-swarm-tools in dev mode
pip install pytest pytest-cov pytest-asyncio black ruff mypy pre-commit
deactivate

# Dashboard venv (separate for web dependencies)
echo "Creating dashboard venv..."
python3.11 -m venv "$PROJECT_ROOT/.venv_dashboard"
source "$PROJECT_ROOT/.venv_dashboard/bin/activate"
pip install --upgrade pip setuptools wheel
pip install fastapi uvicorn websockets paho-mqtt pydantic pyyaml
pip install pytest pytest-cov pytest-asyncio black ruff mypy
deactivate

# Companion tools venv (for log_analyzer, config_generator, etc.)
echo "Creating companion tools venv..."
python3.11 -m venv "$PROJECT_ROOT/.venv_companion_tools"
source "$PROJECT_ROOT/.venv_companion_tools/bin/activate"
pip install --upgrade pip setuptools wheel
pip install numpy scipy matplotlib pyyaml ulog rosbag2_py
deactivate

# Install pre-commit hooks
echo "Installing pre-commit hooks..."
cd "$PROJECT_ROOT"
"$PROJECT_ROOT/.venv_ground/bin/pre-commit" install
"$PROJECT_ROOT/.venv_ground/bin/pre-commit" install --hook-type commit-msg

echo "=== Python environments initialized ==="
echo "Ground tools: $PROJECT_ROOT/.venv_ground"
echo "Dashboard: $PROJECT_ROOT/.venv_dashboard"
echo "Companion tools: $PROJECT_ROOT/.venv_companion_tools"