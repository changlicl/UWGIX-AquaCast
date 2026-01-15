# Manual GitHub Upload Instructions

## Prerequisites

Before you start, you need:
1. ✅ GitHub account
2. ✅ Git installed on your computer
3. ✅ The aquacast-firmware folder downloaded to your computer

---

## Step-by-Step Instructions

### 1. Create GitHub Repository (On GitHub Website)

1. Go to https://github.com
2. Click the **"+"** icon (top right) → **"New repository"**
3. Fill in:
   - **Repository name**: `aquacast-firmware`
   - **Description**: "Autonomous drone-deployable water sampling device for near-shore ocean monitoring"
   - **Public** or **Private** (your choice)
   - **DO NOT** check "Initialize with README" (we already have one)
4. Click **"Create repository"**
5. **Copy the repository URL** (looks like: `https://github.com/YOUR-USERNAME/aquacast-firmware.git`)

---

### 2. Open Terminal/Command Prompt

**Windows**:
- Press `Win + R`, type `cmd`, press Enter
- Or search for "Command Prompt" or "PowerShell"

**Mac**:
- Press `Cmd + Space`, type "Terminal", press Enter

**Linux**:
- Press `Ctrl + Alt + T`

---

### 3. Navigate to Your Repository Folder

```bash
# Replace this path with where you downloaded the folder
cd /path/to/aquacast-firmware

# For example:
# Windows: cd C:\Users\YourName\Downloads\aquacast-firmware
# Mac/Linux: cd ~/Downloads/aquacast-firmware
```

Verify you're in the right place:
```bash
# List files - you should see README.md, firmware/, hardware/, etc.
ls    # Mac/Linux
dir   # Windows
```

---

### 4. Initialize Git Repository

```bash
git init
```

Expected output:
```
Initialized empty Git repository in /path/to/aquacast-firmware/.git/
```

---

### 5. Configure Git (First Time Only)

If this is your first time using Git on this computer:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

---

### 6. Add All Files to Git

```bash
git add .
```

Check what will be committed:
```bash
git status
```

You should see a long list of files in green.

---

### 7. Create Initial Commit

```bash
git commit -m "Initial commit: Gate 7 firmware v0.1.0"
```

Expected output:
```
[main (root-commit) abc1234] Initial commit: Gate 7 firmware v0.1.0
 XX files changed, XXXX insertions(+)
 create mode 100644 README.md
 create mode 100644 firmware/main/main.ino
 ...
```

---

### 8. Connect to GitHub

Replace `YOUR-USERNAME` and paste the URL you copied in Step 1:

```bash
git remote add origin https://github.com/YOUR-USERNAME/aquacast-firmware.git
```

---

### 9. Set Main Branch

```bash
git branch -M main
```

---

### 10. Push to GitHub

**Option A: HTTPS (Recommended for beginners)**

```bash
git push -u origin main
```

You'll be prompted for credentials:
- **Username**: Your GitHub username
- **Password**: Your GitHub Personal Access Token (NOT your GitHub password)

**How to get a token**:
1. GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate new token → Name it "AquaCast" → Select `repo` scope → Generate
3. Copy the token (starts with `ghp_...`)
4. Paste it when prompted for password

**Option B: SSH (If you have SSH keys set up)**

First, change the remote URL to SSH:
```bash
git remote set-url origin git@github.com:YOUR-USERNAME/aquacast-firmware.git
```

Then push:
```bash
git push -u origin main
```

---

### 11. Verify Upload

1. Go to your GitHub repository URL
2. Refresh the page
3. You should see all your files!

---

## Common Issues & Solutions

### Issue: "fatal: not a git repository"
**Solution**: Make sure you ran `git init` and you're in the correct directory.

### Issue: "remote origin already exists"
**Solution**: 
```bash
git remote remove origin
git remote add origin https://github.com/YOUR-USERNAME/aquacast-firmware.git
```

### Issue: "Authentication failed"
**Solution**: 
- Make sure you're using a Personal Access Token, NOT your GitHub password
- Token must have `repo` permissions
- On Windows, check Credential Manager and remove old GitHub credentials

### Issue: "Updates were rejected because the remote contains work"
**Solution**: 
```bash
git pull origin main --allow-unrelated-histories
git push -u origin main
```

---

## Alternative: Using GitHub Desktop (GUI)

If you prefer a graphical interface:

1. Download **GitHub Desktop**: https://desktop.github.com/
2. Install and sign in with your GitHub account
3. Click **File** → **Add Local Repository**
4. Select the `aquacast-firmware` folder
5. Click **Publish repository** button
6. Choose "AquaCast Firmware" as name and click Publish

---

## After Successful Upload

### Share with Your Team

Send them the repository URL:
```
https://github.com/YOUR-USERNAME/aquacast-firmware
```

### Team Members Can Clone

```bash
git clone https://github.com/YOUR-USERNAME/aquacast-firmware.git
cd aquacast-firmware
```

### Make Changes

```bash
# Create a branch
git checkout -b feature/your-feature

# Make changes to files
# ...

# Commit changes
git add .
git commit -m "feat: description of changes"

# Push branch
git push origin feature/your-feature

# Create Pull Request on GitHub
```

---

## Quick Command Summary

For future reference:

```bash
# One-time setup
cd aquacast-firmware
git init
git add .
git commit -m "Initial commit"
git remote add origin https://github.com/YOUR-USERNAME/aquacast-firmware.git
git branch -M main
git push -u origin main

# Daily workflow
git status                    # Check what changed
git add .                     # Stage changes
git commit -m "message"       # Commit changes
git push                      # Push to GitHub
git pull                      # Get latest changes
```

---

## Need Help?

- **Git Documentation**: https://git-scm.com/doc
- **GitHub Guides**: https://guides.github.com/
- **Git Basics**: https://www.atlassian.com/git/tutorials

---

**Generated**: January 14, 2025  
**For**: AquaCast Team - TECHIN 540 AU 2025
