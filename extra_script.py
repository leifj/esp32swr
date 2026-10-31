Import("env")
import os
import shutil

# OTA password is taken from the environment so it is not committed to git.
# Never fall back to an empty value: the update server treats an empty
# username/password as "no authentication" and exposes OTA to the network.
ota_password = os.environ.get("OTA_PASSWORD") or "ota"
if not os.environ.get("OTA_PASSWORD"):
    print("WARNING: OTA_PASSWORD not set, using the default OTA password")
env.Append(BUILD_FLAGS=['-DOTA_PASSWORD=\\"%s\\"' % ota_password])


def sync_web_to_data():
    """Copy web/ -> data/ so the SPIFFS image always matches the sources.

    This runs when the script is loaded, i.e. on every build invocation,
    before PlatformIO decides whether spiffs.bin is up to date. A pre-action
    hook on spiffs.bin is not enough: if data/ is stale the builder considers
    the image current and the hook never fires, uploading an old image.
    """
    web = os.path.join(env["PROJECT_DIR"], "web")
    data = os.path.join(env["PROJECT_DIR"], "data")
    if not os.path.isdir(web):
        print("WARNING: no web/ directory, leaving data/ untouched")
        return
    print("Syncing web/ -> data/ ...")
    if os.path.isdir(data):
        shutil.rmtree(data)
    shutil.copytree(web, data)


sync_web_to_data()
