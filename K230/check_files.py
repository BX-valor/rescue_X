"""Print K230 deployment directory contents for import debugging."""

import os
import sys


DEPLOY_DIR = "/sdcard/rescue_x"

if DEPLOY_DIR not in sys.path:
    sys.path.append(DEPLOY_DIR)

print("__file__ =", globals().get("__file__", "<none>"))
print("cwd =", os.getcwd() if hasattr(os, "getcwd") else "<unknown>")
print("sys.path =", sys.path)

try:
    print("%s:" % DEPLOY_DIR)
    for name in os.listdir(DEPLOY_DIR):
        print(" -", name)
        if name.endswith(".kmodel") and " " in name:
            print("   WARNING: model filename contains a space")
except Exception as error:
    print("listdir error:", error)

for module_name in ("rescue_protocol", "yolo_sender"):
    try:
        __import__(module_name)
        print("import %s: OK" % module_name)
    except SyntaxError as error:
        print("import %s: SyntaxError" % module_name)
        print("  args:", error.args)
        print("  line:", getattr(error, "lineno", "<unknown>"))
        print("  text:", getattr(error, "text", "<unknown>"))
    except Exception as error:
        print("import %s: %s" % (module_name, error))
