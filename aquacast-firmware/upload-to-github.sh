#!/bin/bash
# AquaCast Repository Setup Script
# Run this on your local machine after downloading the repository

echo "=================================="
echo "AquaCast GitHub Upload Script"
echo "=================================="
echo ""

# Step 1: Navigate to repository
echo "Step 1: Navigate to the aquacast-firmware directory"
echo "Run: cd aquacast-firmware"
echo ""
read -p "Press Enter when you're in the directory..."

# Step 2: Initialize git
echo ""
echo "Step 2: Initialize Git repository"
git init
echo ""

# Step 3: Add all files
echo "Step 3: Add all files to git"
git add .
echo ""

# Step 4: Create initial commit
echo "Step 4: Create initial commit"
git commit -m "Initial commit: Gate 7 firmware v0.1.0

- Complete ESP32 firmware with state machine
- Multi-sensor integration (depth, GPS, turbidity, pH)
- Automated depth-based actuation system
- Comprehensive hardware documentation with BOM
- Testing infrastructure and examples
- Team collaboration guidelines

Team: Victoria Yang, Joyce Chou, Chang Li, Shareef Jasim
Course: TECHIN 540 AU 2025 - University of Washington GIX"
echo ""

# Step 5: Add remote
echo "Step 5: Add GitHub remote"
echo ""
read -p "Enter your GitHub repository URL (e.g., https://github.com/username/aquacast-firmware.git): " REPO_URL
git remote add origin "$REPO_URL"
echo ""

# Step 6: Set main branch
echo "Step 6: Set main branch"
git branch -M main
echo ""

# Step 7: Push to GitHub
echo "Step 7: Push to GitHub"
echo "You'll be prompted for your GitHub credentials..."
git push -u origin main
echo ""

echo "=================================="
echo "✅ Repository uploaded successfully!"
echo "=================================="
echo ""
echo "Next steps:"
echo "1. Visit your GitHub repository"
echo "2. Share the link with your team"
echo "3. Team members can clone with: git clone $REPO_URL"
echo ""
