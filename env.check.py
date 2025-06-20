import subprocess
import shutil
import sys
import os
import urllib.request

def is_on_path(command):
    return shutil.which(command) is not None

def run_command(command):
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, text=True)
        print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f" Command failed:\n{e.stderr}")
        return False

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

def install_miniconda(installer_path):
    print(" Launching Miniconda installer...")
    try:
        subprocess.run([installer_path], check=True)
        print(" Miniconda installer launched. Follow the prompts to complete installation.")
    except Exception as e:
        print(f" Failed to launch installer: {e}")

def ensure_conda():
    print("Checking if Conda is available...")
    if not is_on_path("conda"):
        print("Conda is not found in PATH.")
        choice = input("Do you want to download and install Miniconda now? (y/n): ").strip().lower()
        if choice == 'y':
            installer = download_miniconda()
            if installer:
                install_miniconda(installer)
                print("\n After installing Miniconda, please restart your terminal and IDE, then run this script again.")
        else:
            print(" Skipping Conda installation. Cannot proceed without Conda.")
        return False
    else:
        print(" Conda is available.")
        return True

def ensure_python():
    print("\n Checking if Python is installed in Conda...")
    command = "conda list python"
    return run_command(command)

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

def install_python_packages(packages, use_conda=True):
    print("\nChecking and installing required Python packages...")
    for pkg in packages:
        try:
            __import__(pkg)
            print(f"{pkg} is already installed.")
        except ImportError:
            print(f"{pkg} is missing. Installing...")
            if use_conda:
                command = f"conda install -y -c conda-forge {pkg}"
            else:
                command = f"pip install {pkg}"
            success = run_command(command)
            if not success:
                print(f"Failed to install {pkg}.")

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
