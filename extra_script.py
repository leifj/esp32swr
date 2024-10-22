Import("env")
import os

def before_build_spiffs(source, target, env):
    print("Removing old SPIFFS image...")
    env.Execute("rm -rf data")

    print("Copying web app to SPIFFS...")
    env.Execute("cp -pr web data")    

env.AddPreAction("$BUILD_DIR/spiffs.bin", before_build_spiffs)
