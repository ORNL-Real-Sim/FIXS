import subprocess
import shutil
import sys
import os
import urllib.request

# checks if given command is available in the system PATH.
def is_on_path(command):
    return shutil.which(command) is not None

# runs a shell command and captures its output.
def run_command(command):
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, text=True)
        print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f" Command failed:\n{e.stderr}")
        return False

# downloads the Miniconda installer for Windows if it's not already installed.
# returns the path to the downloaded installer file, or none if download fails.
#will prompt for download
def download_miniconda():
    url = "https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe"
    output = "MinicondaInstaller.exe"

    print(f"\n Downloading Miniconda from {url} ...")
    try:
        urllib.request.urlretrieve(url, output)
        print(f" Miniconda downloaded to {output}")
        return output
    except Exception as e:
        print(f" Failed to download Miniconda: {e}")
        return None

# launches the Miniconda installer executable.
# user must manually follow the prompts to complete installation.
def install_miniconda(installer_path):
    print(" Launching Miniconda installer...")
    try:
        subprocess.run([installer_path], check=True)
        print(" Miniconda installer launched. Follow the prompts to complete installation.")
    except Exception as e:
        print(f" Failed to launch installer: {e}")

# checks that Conda is installed and available in PATH.
# if not found, prompts the user to download and install Miniconda.
# returns True if Conda is available, False otherwise.
def ensure_conda():
    print("Checking if Conda is available...")
    if not is_on_path("conda"):
        print("Conda is not found in PATH.")
        #select yes when prompted to download
        choice = input("Do you want to download and install Miniconda now? (y/n): ").strip().lower()
        if choice == 'y':
            installer = download_miniconda()
            if installer:
                install_miniconda(installer)
                #close IDE and reopen to ensure PATH was changed
                print("\n After installing Miniconda, please restart your terminal and IDE, then run this script again.")
        else:
            #check troubleshooting in setupGuide.md
            print(" Skipping Conda installation. Cannot proceed without Conda.")
        return False
    else:
        print(" Conda is available.")
        return True

# checks if Python is installed in Conda.
# returns True if successful.
def ensure_python():
    print("\n Checking if Python is installed in Conda...")
    command = "conda list python"
    return run_command(command)


# checks if SUMO (Simulation of Urban Mobility) is installed.
# if not found, prompts the user to install it via Conda.
# returns True if installed, False otherwise.
def ensure_sumo():
    print("\n Checking if SUMO is installed...")
    if is_on_path("sumo"):
        print(f" SUMO is installed at: {shutil.which('sumo')}")
        return True
    else:
        print(" SUMO is not found.")
        choice = input("Would you like to install SUMO via Conda? (y/n): ").strip().lower()
        if choice == 'y':
            return run_command("conda install -y -c conda-forge sumo")
        else:
            print(" SUMO installation skipped.")
            return False

# checks required Python packages are installed.
# attempts to import each package; if missing, installs it via Conda (or pip if specified).
def install_python_packages(packages, use_conda=True):
    # maps Python import names to Conda/Pip package names
    package_map = {
        "easydict": "easydict",
        "numpy": "numpy",
        "yaml": "pyyaml",
        "ruamel.yaml": "ruamel_yaml",
        "pandas": "pandas",
        "matplotlib": "matplotlib",
        "traci": "traci",
        "sumolib": "sumo"
    }

    for import_name, install_name in package_map.items():
        try:
            __import__(import_name)
            print(f"{import_name} is already installed.")
        except ImportError:
            print(f"{import_name} is missing. Installing...")
            if use_conda:
                command = f"conda install -y -c conda-forge {install_name}"
            else:
                command = f"pip install {install_name}"
            success = run_command(command)
            if not success:
                #check troubleshooting in setupGuide.md
                print(f"Failed to install {install_name}. Please install it manually.")

# main function that runs the entire setup process.
# 1. ensures Conda is installed.
# 2. checks Python in Conda.
# 3. checks SUMO.
# 4. installs required Python packages.
def main():
    print("Environment Setup Script (Windows + Conda)\n")

    if not ensure_conda():
        return

    ensure_python()
    ensure_sumo()

    required_packages = [
        "easydict",
        "numpy",
        "pyyaml",
        "ruamel.yaml",
        "pandas",
        "matplotlib",
        "traci",
        "sumolib"
    ]

    install_python_packages(required_packages, use_conda=True)

    print("\nEnvironment Setup check completed.")

if __name__ == "__main__":
    main()
